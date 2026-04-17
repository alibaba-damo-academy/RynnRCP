# comm_plugin/rynnbot_plugin/rynnbot.py

"""
Rynnbot communication plugin implementation.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~comm_plugin.rynnbot_plugin.rynnbot.RynnBot`, an
:class:`comm_plugin.base.RcpPlugin` that connects a device to the Rynnbot platform
via MQTT (control/telemetry) and WebSocket (protobuf data exchange).

The plugin:
- resolves device credentials from args / YAML config / environment variables
- authenticates via :class:`~comm_plugin.rynnbot_plugin.auth.RynnAuthClient`
- starts :class:`~comm_plugin.rynnbot_plugin.mqtt_client.RynnMqttClient` and
  :class:`~comm_plugin.rynnbot_plugin.ws_client.RynnWebsocketClient`
- normalizes inbound messages into a single queue and dispatches them in
  :meth:`RynnBot.loop_forever`
- bridges platform requests to :class:`rcp_core.RcpCore` tools (actions, images,
  state queries), with bounded executors to avoid unbounded WS task buildup
- periodically reports device properties via MQTT in a background monitor thread
"""

from __future__ import annotations

from typing import Any, List, Dict, Optional
import concurrent.futures
from enum import IntEnum

import os
import io
import gzip
import json
import time
import yaml
import socket
import threading
import numpy as np
from PIL import Image
from queue import Queue, Empty
from dataclasses import dataclass

from rcp_core import RcpCore
from comm_plugin import RcpPlugin

from .auth import RynnAuthClient
from .proto_codec import RynnProtoCodec
from .mqtt_client import RynnMqttClient
from .ws_client import RynnWebsocketClient
from .bound_executor import RynnBoundedExecutor
from .oss_manager import OSSManager, OSSCredential
from rcp_core.common.utils.logger import server_logger

logger = server_logger()

DEVICE_BUSY_CODE = 10001
JSON_PARSE_ERROR_CODE = 10002
UPLOAD_FAILD = 4021


class OccupyType(IntEnum):
    """Occupy type"""

    UNKNOWN = 0
    ACTION = 1
    TELE_DATA_COLL = 2


@dataclass
class RynnBotConfig:
    product_key: Optional[str] = None
    device_name: Optional[str] = None
    device_secret: Optional[str] = None
    http_url: Optional[str] = None
    udp_port: Optional[int] = None


def _load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _deep_get(d: Dict[str, Any], keys: list[str], default=None):
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


class RynnBot(RcpPlugin):
    """
    - Plugin responsible for connecting to the RynnBot platform.
    - Internally constructs RynnAuthClient / RynnMqttClient / RynnWebsocketClient.
    - MQTT / WS messages are unified into a queue and consumed by loop_forever.
    """

    def __init__(
        self,
        device_name: Optional[str] = None,
        device_secret: Optional[str] = None,
        product_key: Optional[str] = None,
        http_url: Optional[str] = None,
        udp_port: Optional[int] = None,
        config_file: Optional[str] = None,
    ) -> None:
        self.rcp_core: Optional[RcpCore] = None

        cfg = self._resolve_config(
            device_name=device_name,
            device_secret=device_secret,
            product_key=product_key,
            http_url=http_url,
            udp_port=udp_port,
            config_file=config_file,
        )

        self.http_url = (cfg.http_url or "").rstrip("/")
        self.endpoint_mqtt = "/connect/mqtt"
        self.endpoint_websocket = "/connect/webSocket"
        self.port = 1883
        self.udp_port = cfg.udp_port

        self.product_key = cfg.product_key
        self.device_name = cfg.device_name
        self.device_secret = cfg.device_secret

        self._occupied = False
        self._occupied_lock = threading.Lock()
        self._occupy_type: OccupyType = OccupyType.UNKNOWN
        self._occupied_id = None

        self._msg_queue: "Queue[Dict[str, Any]]" = Queue()
        self._running = True

        self.auth = RynnAuthClient(
            http_url=self.http_url,
            product_key=self.product_key,
            device_name=self.device_name,
            device_secret=self.device_secret,
        )

        self.proto = RynnProtoCodec()

        self.mqtt_client = RynnMqttClient(
            auth=self.auth,
            endpoint_mqtt=self.endpoint_mqtt,
            mqtt_port=self.port,
            on_message_callback=self.handle_message,
        )
        self.ws_client = RynnWebsocketClient(
            auth=self.auth,
            endpoint_websocket=self.endpoint_websocket,
            on_message_callback=self.handle_message,
        )

        # Thread pool used to handle WS business logic
        self._ws_executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)

        # Bounded executors: prevent unbounded task accumulation.
        self._action_exec = RynnBoundedExecutor(self._ws_executor, max_in_flight=1)
        self._query_exec = RynnBoundedExecutor(self._ws_executor, max_in_flight=16)

        # Device monitor
        self._device_monitor_thread: Optional[threading.Thread] = None
        self._device_monitor_running = False
        self._device_info_cache: Dict[str, Any] = {}

        # Tele data collection session state
        self._data_coll_lock = threading.Lock()
        self._data_coll = {
            "task_id": None,
            "sub_task_id": None,
            "round_number": None,
            "data_coll_id": None,
            "recording": False,
            "last_episode_dir": None,
            "fps": None,
        }

        # Skill record session state
        self._skill_record_lock = threading.Lock()
        self._skill_record = {
            "record_id": None,
            "recording": False,
            "start_time": None,
            "max_duration_sec": 600,  # Default 10 minutes timeout
        }

        # WS state streaming thread for tele_data_coll
        self._ws_state_stream_thread: Optional[threading.Thread] = None
        self._ws_state_stream_running = False
        self._ws_state_stream_lock = threading.Lock()
        self._ws_state_stream_seq: int = 0

        # Run export/upload in background to avoid blocking loop_forever
        self._tele_exec = concurrent.futures.ThreadPoolExecutor(max_workers=1)

        # UDP socket for forwarding WS messages (reuse to avoid frequent create/destroy)
        self._udp_socket: Optional[socket.socket] = None
        self._udp_socket_lock = threading.Lock()
        if self.udp_port:
            try:
                self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                logger.info(f"[RynnBot] UDP socket created for broadcasting to port {self.udp_port}")
            except Exception as e:
                logger.error(f"[RynnBot] Failed to create UDP socket: {e}")
                self._udp_socket = None

    def _resolve_config(
        self,
        *,
        device_name: Optional[str],
        device_secret: Optional[str],
        product_key: Optional[str],
        http_url: Optional[str],
        udp_port: Optional[int],
        config_file: Optional[str],
    ) -> RynnBotConfig:

        any_args_specified = any(
            v is not None
            for v in (product_key, device_name, device_secret, http_url, udp_port)
        )
        if config_file and any_args_specified:
            raise ValueError(
                "RynnBot config conflict: provide either explicit args "
                "or config_file, not both. (Env vars are allowed and take priority.)"
            )

        env_required = [
            "RYNNBOT_PRODUCT_KEY",
            "RYNNBOT_DEVICE_NAME",
            "RYNNBOT_DEVICE_SECRET",
        ]
        env_optional = ["RYNNBOT_HTTP_URL"]

        present_required = {k: (os.getenv(k) not in (None, "")) for k in env_required}
        present_optional = {k: (os.getenv(k) not in (None, "")) for k in env_optional}

        any_env = any(present_required.values()) or any(present_optional.values())
        if any_env and not all(present_required.values()):
            missing = [k for k, ok in present_required.items() if not ok]
            raise ValueError(
                "Incomplete RynnBot env config: since some RYNNBOT_* env var is set, "
                f"the required env vars must all be set: {env_required}. Missing: {missing}"
            )

        # 1) env has top priority
        if any_env:
            cfg = RynnBotConfig(
                product_key=os.getenv("RYNNBOT_PRODUCT_KEY"),
                device_name=os.getenv("RYNNBOT_DEVICE_NAME"),
                device_secret=os.getenv("RYNNBOT_DEVICE_SECRET"),
                http_url=os.getenv("RYNNBOT_HTTP_URL"),
            )
            if not cfg.http_url:
                cfg.http_url = "https://robot-access.damo-academy.com"

            udp_port_env = os.getenv("UDP_PORT")
            if udp_port_env:
                cfg.udp_port = int(udp_port_env)
            else:
                port_env = os.getenv("PORT")
                if port_env:
                    cfg.udp_port = int(port_env) + 5
                else:
                    cfg.udp_port = 8888

            return cfg

        # 2) otherwise use args XOR config_file
        cfg = RynnBotConfig()

        if config_file:
            if not os.path.exists(config_file):
                raise FileNotFoundError(f"RynnBot config_file not found: {config_file}")
            raw = _load_yaml(config_file)

            def from_file(key: str, default=None):
                return _deep_get(raw, ["rynnbot", key], _deep_get(raw, [key], default))

            for k in (
                "product_key",
                "device_name",
                "device_secret",
                "http_url",
                "udp_port",
            ):
                v = from_file(k)
                if v is not None and v != "":
                    if k == "udp_port":
                        try:
                            v = int(v)
                        except (TypeError, ValueError):
                            raise ValueError(f"Invalid udp_port in config: {v!r}")
                    setattr(cfg, k, v)
        else:
            # args
            if product_key is not None:
                cfg.product_key = product_key
            if device_name is not None:
                cfg.device_name = device_name
            if device_secret is not None:
                cfg.device_secret = device_secret
            if http_url is not None:
                cfg.http_url = http_url
            if udp_port is not None:
                cfg.udp_port = int(udp_port)

        if not cfg.http_url:
            cfg.http_url = "https://robot-access.damo-academy.com"

        if not cfg.udp_port:
            cfg.udp_port = 8085

        missing = [
            k
            for k, v in {
                "product_key": cfg.product_key,
                "device_name": cfg.device_name,
                "device_secret": cfg.device_secret,
            }.items()
            if not v
        ]
        if missing:
            raise ValueError(
                f"Missing required RynnBot config fields: {missing}. "
                f"Provide explicit args, or config_file, or env vars "
                f"(RYNNBOT_PRODUCT_KEY/RYNNBOT_DEVICE_NAME/RYNNBOT_DEVICE_SECRET[/RYNNBOT_HTTP_URL])."
            )

        return cfg

    # ===========================
    # RcpPlugin lifecycle
    # ===========================

    def start(self) -> None:
        self.loop_forever()

    def bind_rcp_core(self, rcp_core: RcpCore) -> None:
        self.rcp_core = rcp_core
        
        self.mqtt_client.start()
        self.ws_client.start()
        self._start_device_monitor_thread()

    def list_tools(self) -> List[str]:
        if self.rcp_core is None:
            raise RuntimeError("rcp_core is not bound. Call bind_rcp_core(...) first.")
        return self.rcp_core.tool_list()

    def call_tool(self, name: str, *args, **kwargs) -> Any:
        if self.rcp_core is None:
            raise RuntimeError("rcp_core is not bound. Call bind_rcp_core(...) first.")
        return self.rcp_core.tool_call(name, *args, **kwargs)

    def stop(self) -> None:
        self._running = False
        self._device_monitor_running = False

        # Stop WS state stream thread
        self._stop_ws_state_stream()

        try:
            self.mqtt_client.close()
        except Exception as e:
            logger.error(f"[RynnBot] mqtt_client.close() error: {e}", exc_info=True)

        try:
            self.ws_client.close()
        except Exception as e:
            logger.error(f"[RynnBot] ws_client.close() error: {e}", exc_info=True)

        # Close UDP socket
        with self._udp_socket_lock:
            if self._udp_socket:
                try:
                    self._udp_socket.close()
                    logger.info("[RynnBot] UDP socket closed")
                except Exception as e:
                    logger.error(f"[RynnBot] UDP socket close error: {e}")
                finally:
                    self._udp_socket = None

        # Shutdown thread pools
        try:
            self._ws_executor.shutdown(wait=True)
            logger.info("[RynnBot] WS executor shutdown")
        except Exception as e:
            logger.error(f"[RynnBot] WS executor shutdown error: {e}", exc_info=True)

        try:
            self._tele_exec.shutdown(wait=True)
            logger.info("[RynnBot] Tele executor shutdown")
        except Exception as e:
            logger.error(f"[RynnBot] Tele executor shutdown error: {e}", exc_info=True)

    # ===========================
    # Basic send helpers
    # ===========================

    def _mqtt_send(self, topic: str, payload: str, qos: int = 1) -> None:
        self.mqtt_client.publish(topic, payload, qos=qos)

    def ws_send_data_packet(self, packet: Any) -> None:
        raw = self.proto.serialize_data_packet(packet)
        self.ws_client.send_bytes(raw)

    def _ws_send_packet(self, pkg_type, inner_msg, req_packet) -> None:
        """Build DataPacket with request id and send via WS."""
        resp_packet = self.proto.build_data_packet(
            pkg_type=pkg_type,
            inner_msg=inner_msg,
            id=req_packet.common_part_attr.id,
        )
        self.ws_send_data_packet(resp_packet)

    # ===========================
    # WS reply helpers (PB)
    # ===========================

    def _ws_send_action_finish(
        self,
        req_packet,
        code: int,
        error_msg: str,
        execute_steps: int = 0,
        expect_steps: int = 0,
    ) -> None:
        finish_msg = self.proto.FinishActionChunk()
        finish_msg.code = int(code)
        finish_msg.error_msg = str(error_msg or "")
        finish_msg.execute_steps = int(execute_steps)
        finish_msg.expect_steps = int(expect_steps)
        self._ws_send_packet(
            self.proto.PackageType.ACTION_FINISH, finish_msg, req_packet
        )

    def _reply_action_busy(self, packet, msg: str) -> None:
        self._ws_send_action_finish(
            packet, code=-1, error_msg=msg, execute_steps=0, expect_steps=0
        )

    def _reply_empty_image(self, packet) -> None:
        self._ws_send_packet(
            self.proto.PackageType.IMAGE_DATA, self.proto.MultiImage(), packet
        )

    def _reply_empty_state(self, packet) -> None:
        self._ws_send_packet(
            self.proto.PackageType.STATE_DATA, self.proto.MultiState(), packet
        )

    # ===========================
    # Message entry
    # ===========================

    def handle_message(self, msg: Dict[str, Any]) -> None:
        self._msg_queue.put(msg)

    def loop_forever(self) -> None:
        while self._running:
            try:
                msg = self._msg_queue.get(timeout=1)
            except Empty:
                # Check skill recording timeout even when no messages
                self._check_skill_record_timeout()
                continue
            try:
                self._process_message(msg)
            except Exception as e:
                logger.error("[RynnBot] _process_message() error:", e)

    def _process_message(self, msg: Dict[str, Any]) -> None:
        channel = msg.get("channel")
        if channel == "mqtt":
            self._handle_mqtt_message(msg["topic"], msg["payload"])
        elif channel == "ws":
            self._handle_ws_message(msg["payload"])
        else:
            raise Exception("Unknown channel")

    # ===========================
    # MQTT handlers
    # ===========================

    def _handle_mqtt_message(self, topic: str, payload: str) -> None:
        logger.info(f"[RynnBot] MQTT message: {topic}")
        if topic.endswith("/acquire_device"):
            self.handle_acquire_device(topic, payload)
        elif topic.endswith("/release_device"):
            self.handle_release_device(topic, payload)
        elif topic.endswith("/tele_data_coll:start_round"):
            self.handle_data_coll_start(topic, payload)
        elif topic.endswith("/tele_data_coll:stop_round"):
            self.handle_data_coll_stop(topic, payload)
        elif topic.endswith("/tele_data_coll:upload"):
            self.handle_upload(topic, payload)
        elif topic.endswith("/skill_execute_record_coll:start_record"):
            self.handle_skill_record_start(topic, payload)
        elif topic.endswith("/skill_execute_record_coll:stop_record"):
            self.handle_skill_record_stop(topic, payload)
        else:
            logger.warning(f"[RynnBot] Unhandled MQTT topic: {topic}, skipping")

    def _parse_json_or_reply_error(
        self, payload: str, send_topic: str, op_name: str
    ) -> Optional[dict]:
        try:
            return json.loads(payload)
        except Exception as e:
            logger.error(f"[RynnBot][{op_name}] parse error: {e}")
            error_resp = {
                "jsonrpc": "2.0",
                "id": "-1",
                "error": {
                    "code": JSON_PARSE_ERROR_CODE,
                    "message": f"Failed to parse JSON: {e}",
                },
            }
            self._mqtt_send(send_topic, json.dumps(error_resp))
            return None

    def _parse_json_no_reply(self, payload: str, op_name: str) -> Optional[dict]:
        try:
            return json.loads(payload)
        except Exception as e:
            logger.error(f"[RynnBot][{op_name}] parse error: {e}")
            return None

    @staticmethod
    def _make_base_response(request_json: dict) -> dict:
        return {
            "jsonrpc": "2.0",
            "id": request_json.get("id", "-1"),
            "result": {"$TIME": int(time.time() * 1000)},
        }

    def handle_acquire_device(self, topic: str, payload: str) -> None:
        send_topic = topic.replace("/request/", "/response/")
        request_json = self._parse_json_or_reply_error(payload, send_topic, "acquire")
        if request_json is None:
            return

        resp = self._make_base_response(request_json)

        params = request_json.get("params")
        raw_type = params.get("type")
        self._occupied_id = params.get("id")

        try:
            occupy_type = OccupyType(int(raw_type))
        except (ValueError, TypeError):
            occupy_type = OccupyType.UNKNOWN

        with self._occupied_lock:
            if self._occupied:
                resp["error"] = {"code": DEVICE_BUSY_CODE, "message": "Device is busy"}
                logger.warning("[RynnBot] acquire: device already occupied")
            else:
                self._occupied = True
                self._occupy_type = occupy_type
                logger.info(
                    f"[RynnBot] acquire: success, occupy_type={self._occupy_type.name}({int(self._occupy_type)})"
                )

        self._mqtt_send(send_topic, json.dumps(resp))

    def handle_release_device(self, topic: str, payload: str) -> None:
        send_topic = topic.replace("/request/", "/response/")
        request_json = self._parse_json_or_reply_error(payload, send_topic, "release")
        if request_json is None:
            return

        resp = self._make_base_response(request_json)
        with self._occupied_lock:
            self._occupied = False
            self._occupy_type = OccupyType.UNKNOWN
            logger.info("[RynnBot] release: success")

        # Force stop skill recording if still running
        with self._skill_record_lock:
            if self._skill_record["recording"]:
                logger.warning("[RynnBot] release_device: force stopping skill recording")
                try:
                    self.call_tool("stop_skill_record")
                    with self._skill_record_lock:
                        self._skill_record.update(
                            {
                                "recording": False,
                                "record_id": None,
                                "start_time": None,
                            }
                        )
                    logger.info("[RynnBot] release_device: skill recording stopped")
                except Exception as e:
                    logger.error(f"[RynnBot] release_device: force stop skill record failed: {e}")
                    # Still clear the state even if stop fails
                    self._skill_record.update(
                        {
                            "recording": False,
                            "record_id": None,
                            "start_time": None,
                        }
                    )

        self._mqtt_send(send_topic, json.dumps(resp))

    def handle_data_coll_start(self, topic: str, payload: str) -> None:
        send_topic = topic.replace("/request/", "/response/")
        request_json = self._parse_json_or_reply_error(
            payload, send_topic, "data_coll_start"
        )
        if request_json is None:
            return

        # occupancy check
        params = request_json.get("params")

        task_id = params.get("task_id")
        sub_task_id = params.get("sub_task_id")
        round_number = params.get("round_number")

        task_keys = params.get("task_keys")
        fps = params.get("fps")
        task_prompt = params.get("task_prompt")
        task_description = params.get("task_desc")

        data_coll_id = f"{task_id}/{sub_task_id}"

        logger.info(
            "[RynnBot][data_coll_start] "
            f"task_id={task_id}, sub_task_id={sub_task_id}, round_number={round_number}, "
            f"fps={fps}, task_prompt={task_prompt}, keys={task_keys}"
        )

        # Start collection
        try:
            r = self.call_tool(
                "start_data_collection",
                keys=task_keys,
                task_description=task_description,
                task_prompt=task_prompt,
                fps=float(fps),
                round_number=int(round_number),
                data_coll_id=data_coll_id,
            )
        except Exception as e:
            resp = self._make_base_response(request_json)
            resp["error"] = {
                "code": -1,
                "message": f"start_data_collection exception: {e}",
            }
            self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))
            return

        if not r.get("success"):
            resp = self._make_base_response(request_json)
            resp["error"] = {
                "code": -1,
                "message": f"start_data_collection failed: {r.get('message', '')}",
            }
            self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))
            return

        episode_dir = r.get("result").get("episode_dir")

        with self._data_coll_lock:
            self._data_coll.update(
                {
                    "task_id": task_id,
                    "sub_task_id": sub_task_id,
                    "round_number": int(round_number),
                    "data_coll_id": data_coll_id,
                    "recording": True,
                    "last_episode_dir": episode_dir,
                    "fps": 5.0,
                }
            )

        # Start WS state streaming thread
        self._start_ws_state_stream()

        resp = self._make_base_response(request_json)
        self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))

    def handle_data_coll_stop(self, topic: str, payload: str) -> None:
        send_topic = topic.replace("/request/", "/response/")
        request_json = self._parse_json_or_reply_error(
            payload, send_topic, "data_coll_stop"
        )
        if request_json is None:
            return

        logger.info("[RynnBot][data_coll_stop] stop requested")

        try:
            r = self.call_tool("stop_data_collection")
        except Exception as e:
            resp = self._make_base_response(request_json)
            resp["error"] = {
                "code": -1,
                "message": f"stop_data_collection exception: {e}",
            }
            self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))
            return

        if not r.get("success"):
            resp = self._make_base_response(request_json)
            resp["error"] = {
                "code": -1,
                "message": f"stop_data_collection failed: {r.get('message', '')}",
            }
            self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))
            return

        with self._data_coll_lock:
            self._data_coll["recording"] = False
            self._data_coll["fps"] = None

        # Stop WS state streaming thread
        self._stop_ws_state_stream()

        resp = self._make_base_response(request_json)
        self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))

    def handle_upload(self, topic: str, payload: str) -> None:
        """Unified upload handler that dispatches based on upload_type.
        
        - If upload_type == 'skill_execute': dispatch to handle_skill_record_upload
        - Otherwise: handle as tele_data_coll upload
        """
        send_topic = topic.replace("/request/", "/response/")
        request_json = self._parse_json_or_reply_error(
            payload, send_topic, "upload"
        )
        if request_json is None:
            return

        params = request_json.get("params") or {}
        upload_type = params.get("upload_type")
        
        logger.info(
            f"[RynnBot][upload] received upload request, upload_type={upload_type}"
        )
        
        if upload_type == "skill_execute":
            logger.info("[RynnBot][upload] dispatching to skill_record_upload")
            self.handle_skill_record_upload(topic, payload)
        else:
            # Default: handle as tele_data_coll upload
            self.handle_data_coll_upload(topic, payload)

    def handle_data_coll_upload(self, topic: str, payload: str) -> None:
        send_topic = topic.replace("/request/", "/response/")
        request_json = self._parse_json_or_reply_error(
            payload, send_topic, "data_coll_upload"
        )
        if request_json is None:
            return

        params = request_json.get("params") or {}
        task_id = params.get("task_id")
        sub_task_id = params.get("sub_task_id")
        oss_credential = params.get("oss_credential") or {}

        logger.info(
            f"[RynnBot][data_coll_upload] params received: "
            f"task_id={task_id}, sub_task_id={sub_task_id}, oss_credential keys={list(oss_credential.keys()) if oss_credential else None}"
        )

        if not task_id or not sub_task_id:
            logger.error(
                f"[RynnBot][data_coll_upload] missing task_id/sub_task_id in params: {params}"
            )
            resp = self._make_base_response(request_json)
            resp["error"] = {"code": -1, "message": "missing task_id/sub_task_id"}
            self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))
            return

        data_coll_id = f"{task_id}/{sub_task_id}"

        # 1) reply rrpc immediately
        resp = self._make_base_response(request_json)
        self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))

        logger.info(
            "[RynnBot][tele_data_coll:upload] accepted, will export+upload in background: "
            f"task_id={task_id}, sub_task_id={sub_task_id}, data_coll_id={data_coll_id}"
        )

        # 2) run heavy work in background
        def job():
            if self.rcp_core is None:
                raise RuntimeError("rcp_core not bound")

            # export -> get zip_path
            export_resp: Dict[str, Any] = self.call_tool(
                "export_task_episodes", data_coll_id=data_coll_id
            )
            if not export_resp.get("success"):
                raise RuntimeError(
                    f"export_task_episodes failed: {export_resp.get('message', '')}"
                )

            zip_path = (export_resp.get("result") or {}).get("zip_path")
            if not zip_path or not os.path.isfile(zip_path):
                raise RuntimeError(f"zip_path missing or not a file: {zip_path}")

            self._oss_upload_zip(zip_path, oss_credential, log_tag="tele_data_coll", oss_key="import_dataset.zip")

            # 3) post occupancy_complete
            self._post_upload_event(
                "occupancy_complete",
                {"occupancy_id": str(self._occupied_id)},
                qos=1,
            )
            logger.info(
                f"[RynnBot] occupancy_complete posted: occupancy_id={self._occupied_id}"
            )

        self._submit_upload_job(job, log_tag="tele_data_coll:upload")

    # ===========================
    # skill_execute_record_coll handlers
    # ===========================

    def handle_skill_record_start(self, topic: str, payload: str) -> None:
        """Handle skill_execute_record_coll:start_record request."""
        send_topic = topic.replace("/request/", "/response/")
        request_json = self._parse_json_or_reply_error(payload, send_topic, "skill_record_start")
        if request_json is None:
            return

        params = request_json.get("params") or {}
        record_id = params.get("record_id")
        task_keys = params.get("task_keys")
        fps = params.get("fps")
        task_prompt = params.get("task_prompt", "")
        task_description = params.get("task_description", "")

        logger.info(
            "[RynnBot][skill_record_start] "
            f"record_id={record_id}, fps={fps}, task_prompt={task_prompt}, keys={task_keys}"
        )

        try:
            r = self.call_tool(
                "start_skill_record",
                record_id=record_id,
                task_keys=task_keys,
                fps=float(fps),
                task_prompt=task_prompt,
                task_description=task_description,
            )
        except Exception as e:
            resp = self._make_base_response(request_json)
            resp["error"] = {"code": -1, "message": f"start_skill_record exception: {e}"}
            self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))
            return

        if not r.get("success"):
            resp = self._make_base_response(request_json)
            resp["error"] = {"code": -1, "message": f"start_skill_record failed: {r.get('message', '')}"}
            self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))
            return

        # Set skill record state
        with self._skill_record_lock:
            self._skill_record.update(
                {
                    "record_id": record_id,
                    "recording": True,
                    "start_time": time.time(),
                }
            )

        resp = self._make_base_response(request_json)
        resp["result"] = r.get("result", {})
        self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))

    def handle_skill_record_stop(self, topic: str, payload: str) -> None:
        """Handle skill_execute_record_coll:stop_record request."""
        send_topic = topic.replace("/request/", "/response/")
        request_json = self._parse_json_or_reply_error(payload, send_topic, "skill_record_stop")
        if request_json is None:
            return

        logger.info("[RynnBot][skill_record_stop] stop requested")

        try:
            r = self.call_tool("stop_skill_record")
        except Exception as e:
            resp = self._make_base_response(request_json)
            resp["error"] = {"code": -1, "message": f"stop_skill_record exception: {e}"}
            self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))
            return

        if not r.get("success"):
            resp = self._make_base_response(request_json)
            resp["error"] = {"code": -1, "message": f"stop_skill_record failed: {r.get('message', '')}"}
            self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))
            return

        # Clear skill record state
        with self._skill_record_lock:
            self._skill_record.update(
                {
                    "recording": False,
                    "record_id": None,
                    "start_time": None,
                }
            )

        resp = self._make_base_response(request_json)
        resp["result"] = r.get("result", {})
        self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))

    def handle_skill_record_upload(self, topic: str, payload: str) -> None:
        """Handle skill_execute_record_coll:upload request (runs in background).

        Flow:
          1. Reply RRPC immediately.
          2. For each record_id: export -> zip -> upload to OSS.
          3. Post occupancy_complete event.
        """
        send_topic = topic.replace("/request/", "/response/")
        request_json = self._parse_json_or_reply_error(payload, send_topic, "skill_record_upload")
        if request_json is None:
            return

        params = request_json.get("params") or {}
        
        logger.info(f"[RynnBot][skill_record_upload] params received: {params}")
        
        # Support both 'record_ids' and 'recordIdList' (camelCase from client)
        record_ids = params.get("record_ids") or params.get("recordIdList")
        
        # Parse record_ids if it's a JSON string (e.g., '["id1", "id2"]')
        if isinstance(record_ids, str):
            try:
                record_ids = json.loads(record_ids)
            except Exception as e:
                logger.warning(f"[RynnBot][skill_record_upload] failed to parse record_ids: {e}")
                # Keep as string, will be handled by validation below
        
        oss_credential = params.get("oss_credential") or {}

        logger.info(
            f"[RynnBot][skill_record_upload] params received: "
            f"record_ids={record_ids}, oss_credential keys={list(oss_credential.keys()) if oss_credential else None}"
        )

        # Accept both a single string and a list
        if isinstance(record_ids, str):
            record_ids = [record_ids]

        if not record_ids:
            logger.error(f"[RynnBot][skill_record_upload] missing record_ids in params: {params}")
            resp = self._make_base_response(request_json)
            resp["error"] = {"code": -1, "message": "record_ids required"}
            self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))
            return

        logger.info(f"[RynnBot][skill_record_upload] accepted record_ids={record_ids}")

        # 1) Reply immediately
        resp = self._make_base_response(request_json)
        self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))

        # 2) Run export + upload in background
        def job():
            logger.info(f"[RynnBot][skill_record_upload] exporting record_ids={record_ids}")
            logger.info(f"[RynnBot][skill_record_upload] oss_credential keys: {list(oss_credential.keys()) if oss_credential else 'NULL'}")

            # Export all records in a single call
            export_resp = self.call_tool("export_skill_record", record_ids=record_ids)
            if not export_resp.get("success"):
                raise RuntimeError(
                    f"export_skill_record failed: {export_resp.get('message', '')}"
                )

            # Use the zip_path returned by export_skill_record
            zip_path = (export_resp.get("result") or {}).get("zip_path", "")
            if not zip_path or not os.path.exists(zip_path):
                raise RuntimeError(f"zip file not found")

            logger.info(f"[RynnBot][skill_record_upload] using zip: {zip_path}")
            
            # Unzip and upload files to OSS with directory structure
            logger.info(f"[RynnBot][skill_record_upload] unzipping and uploading to OSS...")
            self._oss_upload_unzipped_files(zip_path, oss_credential, log_tag="skill_record_upload")

            # 3) Post occupancy_complete after upload
            self._post_upload_event(
                "occupancy_complete",
                {"occupancy_id": str(self._occupied_id)},
                qos=1,
            )
            logger.info(
                f"[RynnBot][skill_record_upload] occupancy_complete posted: occupancy_id={self._occupied_id}"
            )

        self._submit_upload_job(job, log_tag="skill_record_upload")

    def _oss_upload_zip(
        self,
        zip_path: str,
        oss_credential: Dict[str, Any],
        log_tag: str = "",
        oss_key: Optional[str] = None,
    ) -> None:
        """Upload a zip file to OSS and delete it locally afterwards.

        Args:
            zip_path: Local path to the zip file.
            oss_credential: OSS credentials dict.
            log_tag: Tag for logging.
            oss_key: Key (filename) to use in OSS. If None, uses basename of zip_path.
        """
        cred = OSSCredential(
            access_key_id=str(oss_credential.get("access_key_id") or oss_credential.get("accessKeyId", "")),
            access_key_secret=str(oss_credential.get("access_key_secret") or oss_credential.get("accessKeySecret", "")),
            security_token=str(oss_credential.get("security_token") or oss_credential.get("securityToken", "")),
            bucket=str(oss_credential.get("bucket", "")),
            prefix=str(oss_credential.get("prefix", "")),
            endpoint=str(oss_credential.get("endpoint", "")),
            region=str(oss_credential.get("region", "")),
            expires_at=str(oss_credential.get("expires_at") or oss_credential.get("expiresAt", "")),
        )
        mgr = OSSManager(cred)
        key = oss_key if oss_key else os.path.basename(zip_path)
        tag = f"[{log_tag}] " if log_tag else ""

        def progress(n: int, written: int, total: int):
            if total:
                rate = int(100 * written / total)
                if rate in (0, 25, 50, 75, 100):
                    logger.info(f"[RynnBot][OSS] {tag}upload {rate}% ({written}/{total})")

        up_res = mgr.multipart_upload_file(
            file_path=zip_path,
            key=key,
            part_size=5 * 1024 * 1024,
            progress_fn=progress,
            abort_on_fail=True,
        )
        logger.info(f"[RynnBot][OSS] {tag}upload done: {up_res}")

        try:
            os.remove(zip_path)
            logger.info(f"[RynnBot] {tag}deleted local zip: {zip_path}")
        except Exception as e:
            logger.warning(f"[RynnBot] {tag}failed to delete local zip {zip_path}: {e}")

    def _oss_upload_unzipped_files(
        self,
        zip_path: str,
        oss_credential: Dict[str, Any],
        log_tag: str = "",
    ) -> None:
        """Unzip a file and upload all contents to OSS preserving directory structure.
        
        This method streams files directly from the zip archive to OSS without
        extracting them to disk, saving disk space for large files.
        
        Args:
            zip_path: Local path to the zip file.
            oss_credential: OSS credentials dict.
            log_tag: Tag for logging.
        """
        import zipfile
        
        cred = OSSCredential(
            access_key_id=str(oss_credential.get("access_key_id") or oss_credential.get("accessKeyId", "")),
            access_key_secret=str(oss_credential.get("access_key_secret") or oss_credential.get("accessKeySecret", "")),
            security_token=str(oss_credential.get("security_token") or oss_credential.get("securityToken", "")),
            bucket=str(oss_credential.get("bucket", "")),
            prefix=str(oss_credential.get("prefix", "")),
            endpoint=str(oss_credential.get("endpoint", "")),
            region=str(oss_credential.get("region", "")),
            expires_at=str(oss_credential.get("expires_at") or oss_credential.get("expiresAt", "")),
        )
        mgr = OSSManager(cred)
        tag = f"[{log_tag}] " if log_tag else ""
        
        uploaded_count = 0
        total_bytes = 0
        
        try:
            # Open zip file and stream contents to OSS
            logger.info(f"[RynnBot][OSS] {tag}opening zip file: {zip_path}")
            with zipfile.ZipFile(zip_path, 'r') as zip_ref:
                for info in zip_ref.infolist():
                    # Skip directories
                    if info.is_dir():
                        continue
                    
                    filename = info.filename
                    file_size = info.file_size
                    
                    # Build OSS key: remove top-level directory from zip filename
                    # filename example: skill_execute_record/24d6ddf74c9c4840b35b9431597cb71c/observation.images.front.mp4
                    # Remove the first directory level (skill_execute_record/) to get relative path
                    if '/' in filename:
                        # Remove top-level directory
                        relative_path = filename.split('/', 1)[1] if filename.count('/') > 0 else filename
                    else:
                        relative_path = filename
                    
                    # OSS key will be automatically prefixed with cred.prefix by OSSManager
                    oss_key = relative_path.replace("\\", "/")
                    full_oss_key = os.path.join(cred.prefix, oss_key).replace("\\", "/") if cred.prefix else oss_key
                    
                    logger.info(f"[RynnBot][OSS] {tag}uploading {filename} ({file_size} bytes) -> oss://{cred.bucket}/{full_oss_key}")
                    
                    # Read file from zip and upload directly from memory
                    with zip_ref.open(filename) as zip_file_handle:
                        # Read entire file into memory (avoid disk I/O)
                        file_data = zip_file_handle.read()
                        
                        # Upload using multipart upload from bytes (no temp file needed)
                        up_res = mgr.multipart_upload_from_bytes(
                            data=file_data,
                            key=oss_key,
                            part_size=5 * 1024 * 1024,  # 5MB per part
                        )
                        
                        logger.info(f"[RynnBot][OSS] {tag}uploaded {filename}: {up_res}")
                        uploaded_count += 1
                        total_bytes += file_size
            
            logger.info(f"[RynnBot][OSS] {tag}total files uploaded: {uploaded_count}, total bytes: {total_bytes / (1024*1024):.2f} MB")
            
        finally:
            # Delete local zip file
            try:
                os.remove(zip_path)
                logger.info(f"[RynnBot][OSS] {tag}deleted local zip: {zip_path}")
            except Exception as e:
                logger.warning(f"[RynnBot][OSS] {tag}failed to delete local zip {zip_path}: {e}")

    def _submit_upload_job(
        self,
        job_fn,
        log_tag: str = "",
    ) -> None:
        """Submit an upload job to background executor, post occupancy events on finish/error."""
        fut = self._tele_exec.submit(job_fn)

        def _done_cb(f: concurrent.futures.Future):
            try:
                f.result()
            except Exception as e:
                logger.error(f"[RynnBot][{log_tag}] background job failed: {e}")
                
                # Clear data_coll state on failure
                with self._data_coll_lock:
                    if self._data_coll["recording"]:
                        logger.warning("[RynnBot] upload failed, clearing data_coll state")
                        self._data_coll.update({
                            "recording": False,
                            "task_id": None,
                            "sub_task_id": None,
                            "round_number": None,
                            "data_coll_id": None,
                        })
                
                # Clear skill record state on failure
                with self._skill_record_lock:
                    if self._skill_record["recording"]:
                        logger.warning("[RynnBot] upload failed, clearing skill_record state")
                        self._skill_record.update({
                            "recording": False,
                            "record_id": None,
                            "start_time": None,
                        })
                
                # Release device on failure
                with self._occupied_lock:
                    if self._occupied:
                        logger.warning("[RynnBot] upload failed, releasing device")
                        self._occupied = False
                        self._occupy_type = OccupyType.UNKNOWN
                
                # Post occupancy error event
                try:
                    self._post_occupancy_error(error_code=UPLOAD_FAILD, error_msg=str(e), qos=1)
                    logger.info("[RynnBot] occupancy_error posted")
                except Exception as ee:
                    logger.error(f"[RynnBot] occupancy_error post failed: {ee}")

        fut.add_done_callback(_done_cb)

    def _check_skill_record_timeout(self) -> None:
        """Check if skill recording has exceeded max duration."""
        with self._skill_record_lock:
            if not self._skill_record["recording"]:
                return
            
            elapsed = time.time() - self._skill_record["start_time"]
            max_duration = self._skill_record.get("max_duration_sec", 600)
            
            if elapsed > max_duration:
                logger.warning(
                    f"[RynnBot][skill_record] Recording timeout: "
                    f"elapsed={elapsed:.1f}s > max={max_duration}s, auto-stopping"
                )
                # Try to auto-stop
                try:
                    self.call_tool("stop_skill_record")
                    self._skill_record.update(
                        {
                            "recording": False,
                            "record_id": None,
                            "start_time": None,
                        }
                    )
                    logger.info("[RynnBot][skill_record] Auto-stopped due to timeout")
                except Exception as e:
                    logger.error(f"[RynnBot][skill_record] Auto-stop failed: {e}")
                    # Still clear state to prevent infinite loop
                    self._skill_record.update(
                        {
                            "recording": False,
                            "record_id": None,
                            "start_time": None,
                        }
                    )

    def _post_upload_event(
        self, event_name: str, params: Dict[str, Any], qos: int = 1
    ) -> None:
        """Post upload event."""
        now_ms = int(time.time() * 1000)
        payload: Dict[str, Any] = {
            "jsonrpc": "2.0",
            "id": str(now_ms),
            "method": f"dm.event.{event_name}.post",
            "params": {"$TIME": now_ms, **(params or {})},
        }
        topic = f"/sys/{self.product_key}/{self.device_name}/dm/event/{event_name}/post"
        self._mqtt_send(topic, json.dumps(payload, ensure_ascii=False), qos=qos)

    def _post_occupancy_error(
        self,
        *,
        error_code: int,
        error_msg: str = "",
        qos: int = 1,
    ) -> None:
        """report error during occupancy."""
        now_ms = int(time.time() * 1000)
        payload: Dict[str, Any] = {
            "jsonrpc": "2.0",
            "id": self._occupied_id,
            "method": "dm.event.occupancy_error.post",
            "params": {
                "$TIME": now_ms,
                "error_code": int(error_code),
            },
        }
        if error_msg:
            payload["params"]["error_msg"] = str(error_msg)

        topic = (
            f"/sys/{self.product_key}/{self.device_name}/dm/event/occupancy_error/post"
        )
        self._mqtt_send(topic, json.dumps(payload, ensure_ascii=False), qos=qos)

    # ===========================
    # WS handlers
    # ===========================

    def _udp_forward_ws_raw(self, payload: Any) -> None:
        """Forward raw payload to UDP port."""
        if not self.udp_port:
            logger.warning("[RynnBot] udp_port not configured, skip UDP forward")
            return

        with self._udp_socket_lock:
            if self._udp_socket is None:
                logger.warning("[RynnBot] UDP socket not available, skip UDP forward")
                return

            try:
                self._udp_socket.sendto(payload, ("localhost", self.udp_port))
                logger.info(
                    f"[RynnBot] forwarded WS payload (raw) to UDP localhost:{self.udp_port}"
                )
            except Exception as e:
                logger.error(f"[RynnBot] UDP forward failed: {e}")
                # Try to recreate socket on error
                try:
                    self._udp_socket.close()
                except Exception:
                    pass
                try:
                    self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    self._udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                    logger.info("[RynnBot] UDP socket recreated after error")
                except Exception as e2:
                    logger.error(f"[RynnBot] Failed to recreate UDP socket: {e2}")
                    self._udp_socket = None

    def _handle_ws_message(self, payload: Any) -> None:
        """Parse DataPacket and dispatch by type (bounded async)."""
        try:
            packet = self.proto.parse_data_packet(payload)
        except Exception as e:
            logger.error(f"[RynnBot] parse DataPacket error: {e}")
            return

        pkg_type, inner = self.proto.parse_inner_payload(packet)

        if pkg_type == self.proto.PackageType.ACTION_DATA:
            self._action_exec.submit(
                self._handle_ws_action_data_impl, inner, packet, timeout=10
            )

        elif pkg_type == self.proto.PackageType.REQ_IMAGE:
            self._query_exec.submit(
                self._handle_ws_req_image_impl, inner, packet, timeout=5
            )

        elif pkg_type == self.proto.PackageType.REQ_STATE:
            self._query_exec.submit(
                self._handle_ws_req_state_impl, inner, packet, timeout=5
            )

        elif pkg_type == self.proto.PackageType.RESV:
            self._udp_forward_ws_raw(inner)

        else:
            logger.warning(f"[RynnBot] Unhandled package type: {pkg_type}")

    def _proto_dtype_to_np(self, dtype_proto) -> Optional[np.dtype]:
        mapping = {
            self.proto.DataType.UINT8: np.uint8,
            self.proto.DataType.INT8: np.int8,
            self.proto.DataType.UINT16: np.uint16,
            self.proto.DataType.INT16: np.int16,
            self.proto.DataType.INT32: np.int32,
            self.proto.DataType.FLOAT32: np.float32,
            self.proto.DataType.FLOAT64: np.float64,
        }
        return mapping.get(dtype_proto)

    def _handle_ws_action_data_impl(self, multi_action, packet) -> None:

        action = multi_action.action_list[0]
        arr_msg = action.action_data

        np_dtype = self._proto_dtype_to_np(arr_msg.dtype)
        if np_dtype is None:
            self._ws_send_action_finish(
                packet,
                code=-1,
                error_msg=f"unsupported dtype: {arr_msg.dtype}",
                execute_steps=0,
                expect_steps=0,
            )
            return

        shape = list(arr_msg.shape)
        data = np.frombuffer(arr_msg.data, dtype=np_dtype).reshape(shape)

        logger.info(
            f"[RynnBot][WS][ACTION] id={action.id}, name={action.name}, "
            f"shape={data.shape}, rate={action.action_rate} Hz"
        )

        step_num = int(data.shape[0])
        fps = int(action.action_rate) if getattr(action, "action_rate", 0) else 30
        action_chunk = {action.name: data.tolist()}

        logger.info(
            f"[RynnBot][WS][ACTION] step_num={step_num}, fps={fps}, use name='{action.name}'"
        )

        try:
            result = self.call_tool(
                "run_action_chunk", action_chunk=action_chunk, fps=fps
            )
        except Exception as e:
            logger.error(f"[RynnBot][WS][ACTION] run_action_chunk error: {e}")
            import traceback

            traceback.print_exc()
            self._ws_send_action_finish(
                packet,
                code=-1,
                error_msg=str(e),
                execute_steps=0,
                expect_steps=step_num,
            )
            return

        success = bool(result.get("success", False))
        message = result.get("message", "")
        result_payload = result.get("result", {}) or {}

        self._ws_send_action_finish(
            packet,
            code=0 if success else -1,
            error_msg=message,
            execute_steps=int(result_payload.get("frames_sent", 0)),
            expect_steps=int(result_payload.get("expect_frames", step_num)),
        )

    def _handle_ws_req_image_impl(self, req_image, packet) -> None:
        logger.info(f"[RynnBot][WS][REQ_IMAGE] request image")

        image_opts: Dict[str, Dict[str, Any]] = {
            cam.camera_name: {"encoding": "png"} for cam in req_image.camera
        }

        try:
            resp: Dict[str, Any] = self.call_tool("get_image", image_opts=image_opts)
            if not resp.get("success"):
                logger.error(f"[RynnBot][WS][REQ_IMAGE] get_image failed: {resp}")
                raise RuntimeError(resp.get("message", "get_image failed"))
            images_dict: Dict[str, bytes] = resp.get("result", {}) or {}
        except Exception as e:
            logger.error(f"[RynnBot][WS][REQ_IMAGE] get_image error: {e}")
            self._reply_empty_image(packet)
            return

        multi_image = self.proto.MultiImage()
        for cam in req_image.camera:
            cam_key = cam.camera_name
            encoded = images_dict.get(cam_key)
            if encoded is None:
                logger.error(f"[RynnBot][WS][REQ_IMAGE] missing image: {cam_key}")
                continue

            try:
                img = Image.open(io.BytesIO(encoded)).convert("RGB")
                rgb_arr = np.array(img, dtype=np.uint8)
            except Exception as e:
                logger.error(f"[RynnBot][WS][REQ_IMAGE] decode failed: {cam_key}, {e}")
                continue

            try:
                buf = io.BytesIO()
                np.save(buf, rgb_arr)
                buf.seek(0)
                compressed = gzip.compress(buf.getvalue(), compresslevel=1)
            except Exception as e:
                logger.error(
                    f"[RynnBot][WS][REQ_IMAGE] np.save+gzip failed: {cam_key}, {e}"
                )
                continue

            img_msg = multi_image.image_list.add()
            img_msg.id = cam.camera_id
            img_msg.name = cam.camera_name or cam_key
            img_msg.format = "npy_gzip"
            img_msg.image_data.data = compressed
            img_msg.image_data.shape[:] = list(rgb_arr.shape)
            img_msg.image_data.dtype = self.proto.DataType.UINT8

            logger.info(
                f"[RynnBot][WS][REQ_IMAGE] {cam_key}, shape={rgb_arr.shape}, bytes={len(compressed)}"
            )

        self._ws_send_packet(self.proto.PackageType.IMAGE_DATA, multi_image, packet)

    def _handle_ws_req_state_impl(self, req_state, packet) -> None:
        logger.info(f"[RynnBot][WS][REQ_STATE] robots:{len(req_state.robot)}")

        try:
            resp: Dict[str, Any] = self.call_tool("get_state")
            if not resp.get("success"):
                raise RuntimeError(resp.get("message", "get_state failed"))
            state_result: Dict[str, Any] = resp.get("result", {}) or {}
        except Exception as e:
            logger.error(f"[RynnBot][WS][REQ_STATE] get_state error: {e}", exc_info=True)
            self._reply_empty_state(packet)
            return

        logger.info(f"[RynnBot][WS][REQ_STATE] state_result:{state_result}")

        multi_state = self.proto.MultiState()
        if "observation.state" in state_result:
            val = state_result["observation.state"]
            arr = np.array(val, dtype=np.float32)

            logger.info(
                f"[RynnBot][WS][REQ_STATE] observation.state, shape={arr.shape}, val = {val}"
            )

            s_msg = multi_state.state_list.add()
            s_msg.id = 1
            s_msg.name = "observation.state"
            s_msg.state_data.data = arr.tobytes()
            s_msg.state_data.shape[:] = list(arr.shape)
            s_msg.state_data.dtype = self.proto.DataType.FLOAT32

            logger.info(
                f"[RynnBot][WS][REQ_STATE] observation.state, shape={arr.shape}, "
                f"bytes={len(s_msg.state_data.data)}"
            )

        else:
            logger.warning(
                f"[RynnBot][WS][REQ_STATE] observation.state not in result, send empty MultiState"
            )

        self._ws_send_packet(self.proto.PackageType.STATE_DATA, multi_state, packet)

    # ===========================
    # Device monitor
    # ===========================

    def _start_device_monitor_thread(self, interval_sec: float = 5.0) -> None:
        if self._device_monitor_thread is not None:
            return

        self._device_monitor_running = True

        def loop():
            while self._device_monitor_running:
                try:
                    self._report_device_properties()
                except Exception as e:
                    logger.error(f"[RynnBot][DeviceMonitor] report error: {e}", exc_info=True)
                time.sleep(interval_sec)

        t = threading.Thread(target=loop, daemon=True)
        t.start()
        self._device_monitor_thread = t

    def _report_device_properties(self) -> None:
        """Call get_device_info and report the result"""
        if self.rcp_core is None:
            logger.warning(
                "[RynnBot][DeviceMonitor] rcp_core is not bound, skip reporting"
            )
            return

        try:
            resp: Dict[str, Any] = self.call_tool("get_device_info")
        except Exception as e:
            logger.warning(f"[RynnBot][DeviceMonitor] get_device_info call failed: {e}")
            return

        if not isinstance(resp, dict) or not resp.get("success"):
            logger.warning(
                "[RynnBot][DeviceMonitor] get_device_info failed:",
                resp.get("message"),
            )
            return

        params = resp.get("result", {}) or {}
        if not isinstance(params, dict):
            logger.warning(
                "[RynnBot][DeviceMonitor] invalid result from get_device_info"
            )
            return

        self._device_info_cache.update(params)

        msg_id = int(time.time() * 1000)
        json_rpc_msg: Dict[str, Any] = {
            "jsonrpc": "2.0",
            "id": str(msg_id),
            "method": "dm.property.post",
            "params": params,
        }

        topic = f"sys/{self.product_key}/{self.device_name}/dm/property/post"
        self._mqtt_send(topic, json.dumps(json_rpc_msg, ensure_ascii=False), qos=1)

    # ===========================
    # WS State Streaming for tele_data_coll
    # ===========================

    def _start_ws_state_stream(self) -> None:
        """Start the WS state streaming thread."""
        with self._ws_state_stream_lock:
            if self._ws_state_stream_running:
                logger.warning("[RynnBot][WSStateStream] already running, skip start")
                return

            self._ws_state_stream_running = True
            self._ws_state_stream_seq = 0  # Reset sequence counter
            t = threading.Thread(target=self._ws_state_stream_loop, daemon=True)
            t.start()
            self._ws_state_stream_thread = t
            logger.info("[RynnBot][WSStateStream] thread started")

    def _stop_ws_state_stream(self) -> None:
        """Stop the WS state streaming thread."""
        with self._ws_state_stream_lock:
            self._ws_state_stream_running = False

        if self._ws_state_stream_thread is not None:
            self._ws_state_stream_thread.join(timeout=2.0)
            self._ws_state_stream_thread = None
            logger.info("[RynnBot][WSStateStream] thread stopped")

    def _ws_state_stream_loop(self) -> None:
        """Loop that periodically sends STATE_DATA via WS."""
        while True:
            with self._ws_state_stream_lock:
                if not self._ws_state_stream_running:
                    break

            # Get fps from data_coll
            with self._data_coll_lock:
                fps = self._data_coll.get("fps")
                recording = self._data_coll.get("recording", False)

            if not fps or not recording:
                logger.info("[RynnBot][WSStateStream] fps not set or not recording, stop streaming")
                break

            interval = 1.0 / fps

            # Get state and send via WS
            try:
                self._send_state_via_ws()
            except Exception as e:
                logger.error(f"[RynnBot][WSStateStream] send state error: {e}", exc_info=True)

            time.sleep(interval)

        logger.info("[RynnBot][WSStateStream] loop ended")

    def _send_state_via_ws(self) -> None:
        """Get current state and send as STATE_DATA via WS."""
        if self.rcp_core is None:
            return

        try:
            resp: Dict[str, Any] = self.call_tool("get_state")
            if not resp.get("success"):
                raise RuntimeError(resp.get("message", "get_state failed"))
            state_result: Dict[str, Any] = resp.get("result", {}) or {}
        except Exception as e:
            logger.error(f"[RynnBot][WSStateStream] get_state error: {e}")
            return

        multi_state = self.proto.MultiState()
        if "observation.state" in state_result:
            val = state_result["observation.state"]
            arr = np.array(val, dtype=np.float32)

            s_msg = multi_state.state_list.add()
            s_msg.id = 1
            s_msg.name = "observation.state"
            s_msg.state_data.data = arr.tobytes()
            s_msg.state_data.shape[:] = list(arr.shape)
            s_msg.state_data.dtype = self.proto.DataType.FLOAT32

        # Build and send DataPacket with incrementing sequence id
        with self._ws_state_stream_lock:
            seq_id = self._ws_state_stream_seq
            self._ws_state_stream_seq += 1

        packet = self.proto.build_data_packet(
            pkg_type=self.proto.PackageType.STATE_DATA,
            inner_msg=multi_state,
            id=seq_id,
        )
        self.ws_send_data_packet(packet)
        logger.debug(f"[RynnBot][WSStateStream] sent STATE_DATA, seq={seq_id}, shape={arr.shape if 'observation.state' in state_result else 'empty'}")
