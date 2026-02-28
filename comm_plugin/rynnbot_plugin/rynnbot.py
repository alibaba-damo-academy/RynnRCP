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

import os
import io
import gzip
import json
import time
import yaml
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
from rcp_core.common.utils.logger import server_logger

logger = server_logger()

DEVICE_BUSY_CODE = 10001
JSON_PARSE_ERROR_CODE = 10002


@dataclass
class RynnBotConfig:
    product_key: Optional[str] = None
    device_name: Optional[str] = None
    device_secret: Optional[str] = None
    http_url: Optional[str] = None


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
        config_file: Optional[str] = None,
    ) -> None:
        self.rcp_core: Optional[RcpCore] = None

        cfg = self._resolve_config(
            device_name=device_name,
            device_secret=device_secret,
            product_key=product_key,
            http_url=http_url,
            config_file=config_file,
        )

        self.http_url = (cfg.http_url or "").rstrip("/")
        self.endpoint_mqtt = "/connect/mqtt"
        self.endpoint_websocket = "/connect/webSocket"
        self.port = 1883

        self.product_key = cfg.product_key
        self.device_name = cfg.device_name
        self.device_secret = cfg.device_secret

        self._occupied = False
        self._occupied_lock = threading.Lock()

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

        # Start MQTT and WebSocket clients immediately
        self.mqtt_client.start()
        self.ws_client.start()

        # Thread pool used to handle WS business logic
        self._ws_executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)

        # Bounded executors: prevent unbounded task accumulation.
        self._action_exec = RynnBoundedExecutor(self._ws_executor, max_in_flight=1)
        self._query_exec = RynnBoundedExecutor(self._ws_executor, max_in_flight=16)

        # Device monitor
        self._device_monitor_thread: Optional[threading.Thread] = None
        self._device_monitor_running = False
        self._device_info_cache: Dict[str, Any] = {}
        self._start_device_monitor_thread()

    def _resolve_config(
        self,
        *,
        device_name: Optional[str],
        device_secret: Optional[str],
        product_key: Optional[str],
        http_url: Optional[str],
        config_file: Optional[str],
    ) -> RynnBotConfig:

        # ---- args XOR config_file ----
        any_args_specified = any(
            v is not None for v in (product_key, device_name, device_secret, http_url)
        )
        if config_file and any_args_specified:
            raise ValueError(
                "RynnBot config conflict: provide either explicit args "
                "or config_file, not both. (Env vars are allowed and take priority.)"
            )

        # ---- env all-or-nothing (required trio) ----
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
            return cfg

        # 2) otherwise use args XOR config_file
        cfg = RynnBotConfig()

        if config_file:
            if not os.path.exists(config_file):
                raise FileNotFoundError(f"RynnBot config_file not found: {config_file}")
            raw = _load_yaml(config_file)

            def from_file(key: str, default=None):
                return _deep_get(raw, ["rynnbot", key], _deep_get(raw, [key], default))

            for k in ("product_key", "device_name", "device_secret", "http_url"):
                v = from_file(k)
                if v is not None and v != "":
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

        if not cfg.http_url:
            cfg.http_url = "https://robot-access.damo-academy.com"

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

        try:
            self.mqtt_client.close()
        except Exception as e:
            logger.error("[RynnBot] mqtt_client.close() error:", e)

        try:
            self.ws_client.close()
        except Exception as e:
            logger.error("[RynnBot] ws_client.close() error:", e)

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
        with self._occupied_lock:
            if self._occupied:
                resp["error"] = {"code": DEVICE_BUSY_CODE, "message": "Device is busy"}
                logger.warning("[RynnBot] acquire: device already occupied")
            else:
                self._occupied = True
                logger.info("[RynnBot] acquire: success")

        self._mqtt_send(send_topic, json.dumps(resp))

    def handle_release_device(self, topic: str, payload: str) -> None:
        send_topic = topic.replace("/request/", "/response/")
        request_json = self._parse_json_or_reply_error(payload, send_topic, "release")
        if request_json is None:
            return

        resp = self._make_base_response(request_json)
        with self._occupied_lock:
            self._occupied = False
            logger.info("[RynnBot] release: success")
        self._mqtt_send(send_topic, json.dumps(resp))

    # ===========================
    # WS handlers
    # ===========================

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
            logger.error("[RynnBot][WS][REQ_STATE] get_state error:", e)
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
                    logger.error("[RynnBot][DeviceMonitor] report error:", e)
                time.sleep(interval_sec)

        t = threading.Thread(target=loop, daemon=True)
        t.start()
        self._device_monitor_thread = t

    def _report_device_properties(self) -> None:
        """调用 get_device_info 并通过 MQTT 上报设备属性"""
        if self.rcp_core is None:
            logger.warning(
                "[RynnBot][DeviceMonitor] rcp_core is not bound, skip reporting"
            )
            return

        try:
            resp: Dict[str, Any] = self.call_tool("get_device_info")
        except Exception as e:
            logger.warning("[RynnBot][DeviceMonitor] get_device_info call failed:", e)
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
