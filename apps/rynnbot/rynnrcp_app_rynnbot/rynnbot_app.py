"""
RynnBotApp - Connects one RynnRCP Server to the Rynnbot cloud platform.

The app:
- Resolves device credentials from the app config
- Authenticates via RynnAuthClient (HMAC-SHA1)
- Starts MQTT client (control/telemetry) and WebSocket client (data exchange)
- Normalizes inbound messages into a queue and dispatches them
- Bridges platform requests to Interface tools (actions, images, state queries)
- Reports device properties via MQTT in a background monitor thread
"""

from __future__ import annotations

import argparse
import base64
import concurrent.futures
import gzip
import io
import json
import logging
import platform
import socket
import sys
import threading
import time
from enum import IntEnum
from queue import Empty, Queue
from typing import Any, Dict, List, Optional

import numpy as np
from PIL import Image

from rynnrcp.interface.client import ClientInterface
from rynnrcp.interface.protocol_client import RcpProtocolClient, ServerManifest, resolve_server_manifest
from rynnrcp.utils.logging import configure_logging
from rynnrcp.utils.user_paths import (
    data_dir,
    local_root_from_config,
    log_file_from_config,
    resolve_robot_path,
)
from rynnrcp_app_common import AppLifecycle
from rynnrcp_app_common.protocol_key_mapping import (
    protocol_to_rynnbot_key,
    protocol_to_rynnbot_mapping,
    rynnbot_to_protocol_key,
)

from .auth import RynnAuthClient
from .bound_executor import RynnBoundedExecutor
from .capture_manager import RynnBotCaptureManager
from .config import load_yaml, merge_default_config, resolve_rynnbot_config
from .mqtt_client import RynnMqttClient
from .proto_codec import RynnProtoCodec
from .ws_client import RynnWebsocketClient

logger = logging.getLogger(__name__)

DEVICE_BUSY_CODE = 10001
JSON_PARSE_ERROR_CODE = 10002


class OccupyType(IntEnum):
    """Occupy type."""
    UNKNOWN = 0
    ACTION = 1
    TELE_DATA_COLL = 2


class RynnBotApp(AppLifecycle):
    """RynnBot cloud platform communication app.

    Connects device to Rynnbot platform via MQTT + WebSocket, bridging
    platform commands to a configured RynnRCP Server.

    Config dict keys:
        product_key: Rynnbot product key
        device_name: Device identifier
        device_secret: Authentication secret
        http_url: Platform HTTP endpoint (default: https://robot-access.damo-academy.com)
        udp_port: Optional UDP broadcast port
    """

    VERSION = "0.1.0"
    DESCRIPTION = "Rynnbot cloud platform communication bridge"
    AUTHOR = "RynnRCP Team"

    def __init__(self, name: str = "rynnbot", config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(name, merge_default_config(config))
        self._app_root = local_root_from_config(self.config)
        storage_cfg = self.config.get("storage") if isinstance(self.config.get("storage"), dict) else {}
        raw_capture_root = storage_cfg.get("raw_capture_root") if isinstance(storage_cfg, dict) else None
        self._raw_capture_root = resolve_robot_path(
            str(raw_capture_root) if raw_capture_root else data_dir(self._app_root) / "raw_captures",
            self._app_root,
        )
        self._raw_capture_root.mkdir(parents=True, exist_ok=True)
        self._manifest: ServerManifest | None = None
        self._server_client: RcpProtocolClient | None = None
        self._loop_running = False

        cfg = resolve_rynnbot_config(self.config)

        self._http_url = (cfg.http_url or "").rstrip("/")
        self._endpoint_mqtt = "/connect/mqtt"
        self._endpoint_websocket = "/connect/webSocket"
        self._mqtt_port = 1883
        self._udp_port = cfg.udp_port

        self._product_key = cfg.product_key
        self._device_name_cfg = cfg.device_name
        self._device_secret = cfg.device_secret
        self._mqtt_connect_timeout_s = cfg.mqtt_connect_timeout_s
        self._mqtt_connect_attempts = cfg.mqtt_connect_attempts
        self._device_monitor_interval_s = cfg.device_monitor_interval_s
        self._image_upload_codec = cfg.image_upload_codec  # "npy_gzip" | "jpeg"
        self._image_jpeg_quality = cfg.image_jpeg_quality

        # State
        self._occupied = False
        self._occupied_lock = threading.Lock()
        self._occupy_type: OccupyType = OccupyType.UNKNOWN
        self._occupied_id = None

        self._msg_queue: "Queue[Dict[str, Any]]" = Queue()

        # Auth client
        self._auth = RynnAuthClient(
            http_url=self._http_url,
            product_key=self._product_key,
            device_name=self._device_name_cfg,
            device_secret=self._device_secret,
            request_timeout_s=cfg.auth_timeout_s,
            max_attempts=cfg.auth_max_attempts,
        )

        # Proto codec
        self._proto = RynnProtoCodec()

        # MQTT client
        self._mqtt_client = RynnMqttClient(
            auth=self._auth,
            endpoint_mqtt=self._endpoint_mqtt,
            mqtt_port=self._mqtt_port,
            on_message_callback=self._handle_message,
        )

        # WebSocket client
        self._ws_client = RynnWebsocketClient(
            auth=self._auth,
            endpoint_websocket=self._endpoint_websocket,
            on_message_callback=self._handle_message,
        )

        # Thread pools
        self._ws_executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)
        self._action_exec = RynnBoundedExecutor(self._ws_executor, max_in_flight=1)
        self._query_exec = RynnBoundedExecutor(self._ws_executor, max_in_flight=16)

        # Device monitor
        self._device_monitor_thread: Optional[threading.Thread] = None
        self._device_monitor_running = False
        self._device_info_cache: Dict[str, Any] = {}
        self._camera_info_cache: List[Dict[str, Any]] = []

        # WS state streaming
        self._ws_state_stream_thread: Optional[threading.Thread] = None
        self._ws_state_stream_running = False
        self._ws_state_stream_lock = threading.Lock()
        self._ws_state_stream_seq: int = 0

        # Background export executor
        self._tele_exec = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self._capture_manager = RynnBotCaptureManager(
            config=self.config,
            raw_capture_root=self._raw_capture_root,
            server_client_provider=lambda: self._server_client,
            mqtt_send=self._mqtt_send,
            parse_json_or_reply_error=self._parse_json_or_reply_error,
            make_base_response=self._make_base_response,
            start_ws_state_stream=self._start_ws_state_stream,
            stop_ws_state_stream=self._stop_ws_state_stream,
            post_upload_event=self._post_upload_event,
            post_occupancy_error=self._post_occupancy_error,
            occupied_id_provider=lambda: self._occupied_id,
            clear_occupancy=self._clear_occupancy_after_upload_failure,
            executor=self._tele_exec,
            cloud_to_protocol_key=self._cloud_to_protocol_key,
            protocol_to_cloud_mapping=self._protocol_to_cloud_mapping,
        )

        # UDP socket for forwarding
        self._udp_socket: Optional[socket.socket] = None
        self._udp_socket_lock = threading.Lock()
        if self._udp_port:
            try:
                self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                logger.info("[RynnBot] UDP socket created for broadcasting to port %d", self._udp_port)
            except Exception as e:
                logger.error("[RynnBot] Failed to create UDP socket: %s", e, exc_info=True)
                self._udp_socket = None

        # Loop thread
        self._loop_thread: Optional[threading.Thread] = None

    def connect_server(self) -> None:
        """Resolve the configured RynnRCP robot and create a protocol client."""
        if self._server_client is None:
            target = _target_robot_config(self.config)
            request_timeout_ms = int(target.get("request_timeout_ms") or 1000)
            interface = ClientInterface.with_defaults(local_registry=True, mdns=True)
            self._manifest = resolve_server_manifest(
                robot_id=str(target["robot_id"]),
                embodiment_type=str(target.get("embodiment_type") or "") or None,
                interface=interface,
                request_timeout_ms=request_timeout_ms,
            )
            if self._manifest.endpoint is None:
                raise RuntimeError(f"robot_id {self._manifest.robot_id!r} has no endpoint")
            self._server_client = RcpProtocolClient(interface.connect(self._manifest.endpoint))
        self._logger.info(
            "Connected RynnBot app to robot_id=%s device=%s product=%s",
            self._manifest.robot_id if self._manifest else "",
            self._device_name_cfg,
            self._product_key,
        )

    def start(self) -> None:
        """Start the message processing loop."""
        if self._running:
            return
        self._log_video_encoder_self_check()

        try:
            if self._server_client is None:
                self.connect_server()
            self._mqtt_client.start(
                connect_timeout_s=self._mqtt_connect_timeout_s,
                connect_attempts=self._mqtt_connect_attempts,
            )
            self._ws_client.start()
            self._start_device_monitor_thread()

            self._loop_running = True
            self._mark_started()

            self._loop_thread = threading.Thread(
                target=self._loop_forever, daemon=True, name="rynnbot-loop"
            )
            self._loop_thread.start()
            self._logger.info("Started")
        except Exception:
            self._logger.error("RynnBot startup failed; closing external clients", exc_info=True)
            self._loop_running = False
            self._device_monitor_running = False
            self._ws_state_stream_running = False
            self._cleanup_external_clients()
            raise

    def _log_video_encoder_self_check(self) -> None:
        try:
            from rynnrcp_app_common.hardware_codec import log_video_encoder_self_check

            log_video_encoder_self_check(
                self._logger,
                preferred=self.config.get("video_encoder"),
                smoke_test=bool(self.config.get("video_encoder_smoke_test", True)),
                prefix="RynnBotEncoder",
            )
        except Exception as exc:
            self._logger.warning("Video encoder self-check failed: %s", exc, exc_info=True)

    def stop(self) -> None:
        """Stop the app and release external clients."""
        if not self._running:
            if self._server_client is not None and hasattr(self._server_client, "close"):
                self._server_client.close()
                self._server_client = None
            return

        self._loop_running = False
        self._mark_stopped()
        self._device_monitor_running = False

        # Stop WS state stream
        self._stop_ws_state_stream()

        self._cleanup_external_clients()
        if self._server_client is not None and hasattr(self._server_client, "close"):
            self._server_client.close()
            self._server_client = None

        # Close UDP socket
        with self._udp_socket_lock:
            if self._udp_socket:
                try:
                    self._udp_socket.close()
                except Exception:
                    pass
                self._udp_socket = None

        # Shutdown executors
        try:
            self._ws_executor.shutdown(wait=False)
        except Exception:
            pass
        try:
            self._tele_exec.shutdown(wait=False)
        except Exception:
            pass

        # Join loop thread
        if self._loop_thread and self._loop_thread.is_alive():
            self._loop_thread.join(timeout=3.0)
        self._loop_thread = None

        self._logger.info("Stopped")

    def _cleanup_external_clients(self) -> None:
        try:
            self._mqtt_client.close()
        except Exception as e:
            logger.error("[RynnBot] mqtt_client.close() error: %s", e, exc_info=True)

        try:
            self._ws_client.close()
        except Exception as e:
            logger.error("[RynnBot] ws_client.close() error: %s", e, exc_info=True)

    # ─── Tool Access ─────────────────────────────────────────────────

    def list_tools(self) -> List[str]:
        """List available protocol tools on the server."""
        if self._server_client is None:
            return []
        return list(self._list_server_tools().keys())

    def call_tool(self, name: str, params: Optional[Dict[str, Any]] = None) -> Any:
        """Call a protocol method on the server."""
        if self._server_client is None:
            raise RuntimeError("server_client not available")
        response = self._server_client.request(name, params or {})
        if response.ok:
            return response.payload
        return {"success": False, "message": response.message, "result": None}

    def _list_server_tools(self) -> Dict[str, Dict[str, Any]]:
        if self._server_client is None:
            return {}
        response = self._server_client.list_tools()
        if not response.ok or not isinstance(response.payload, dict):
            raise RuntimeError(response.message or "list_tools failed")
        return dict(response.payload)

    def _observation_names_by_type(self, observation_type: str) -> List[str]:
        if self._manifest is None:
            return []
        return [
            str(item["name"])
            for item in self._manifest.observations
            if item.get("name") and item.get("type") == observation_type
        ]

    def _action_name(self, requested: str = "") -> str:
        if self._manifest is None:
            return ""
        action_names = [str(item["name"]) for item in self._manifest.actions if item.get("name")]
        if requested:
            mapped = self._cloud_to_protocol_key(requested)
            if mapped in action_names:
                return mapped
        for item in self._manifest.actions:
            if item.get("type") == "joint_position" and item.get("name"):
                return str(item["name"])
        return action_names[0] if action_names else ""

    def _state_observation_name(self) -> str:
        names = self._observation_names_by_type("joint_state")
        return names[0] if names else ""

    def _image_observation_names(self) -> List[str]:
        return self._observation_names_by_type("image")

    def _protocol_names(self) -> List[str]:
        if self._manifest is None:
            return []
        return [
            *[str(item["name"]) for item in self._manifest.observations if item.get("name")],
            *[str(item["name"]) for item in self._manifest.actions if item.get("name")],
        ]

    def _key_mapping_config(self) -> Dict[str, Any]:
        value = self.config.get("key_mapping") or self.config.get("export_key_mapping")
        return dict(value) if isinstance(value, dict) else {}

    def _cloud_to_protocol_key(self, name: str) -> str:
        return rynnbot_to_protocol_key(
            str(name),
            self._protocol_names(),
            custom=self._key_mapping_config(),
        )

    def _protocol_to_cloud_key(self, name: str) -> str:
        return protocol_to_rynnbot_key(str(name), custom=self._key_mapping_config())

    def _protocol_to_cloud_mapping(self, names: List[str]) -> Dict[str, str]:
        return protocol_to_rynnbot_mapping(names, custom=self._key_mapping_config())

    def _get_observations(self, names: List[str]) -> Dict[str, Any]:
        if self._server_client is None:
            logger.warning("[RynnBot][OBS] get_observations skipped: server client is not ready")
            return {}
        started = time.perf_counter()
        response = self._server_client.get_observations(names)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        if not response.ok or not isinstance(response.payload, dict):
            logger.warning(
                "[RynnBot][OBS] get_observations failed: names=%s ok=%s message=%s elapsed=%.1fms payload_type=%s",
                names,
                response.ok,
                response.message,
                elapsed_ms,
                type(response.payload).__name__,
            )
            return {}
        observations = response.payload.get("observations")
        if not isinstance(observations, list):
            logger.warning(
                "[RynnBot][OBS] get_observations returned invalid payload: names=%s elapsed=%.1fms payload_keys=%s",
                names,
                elapsed_ms,
                sorted(response.payload.keys()),
            )
            return {}
        values: Dict[str, Any] = {}
        for item in observations:
            if isinstance(item, dict) and item.get("name"):
                values[str(item["name"])] = item.get("value")
        missing = [name for name in names if name not in values]
        logger.info(
            "[RynnBot][OBS] get_observations names=%s returned=%s missing=%s elapsed=%.1fms summary=%s",
            names,
            sorted(values.keys()),
            missing,
            elapsed_ms,
            {key: _observation_value_summary(value) for key, value in values.items()},
        )
        return values

    # ─── Message Handling ────────────────────────────────────────────

    def _handle_message(self, msg: Dict[str, Any]) -> None:
        """Callback for incoming MQTT/WS messages. Enqueue for processing."""
        self._msg_queue.put(msg)

    def _loop_forever(self) -> None:
        """Main message processing loop (runs in background thread)."""
        while self._loop_running:
            try:
                msg = self._msg_queue.get(timeout=0.5)
                self._dispatch_message(msg)
            except Empty:
                self._capture_manager.check_skill_record_timeout()
                continue
            except Exception as e:
                logger.error("[RynnBot] Loop error: %s", e, exc_info=True)

    def _dispatch_message(self, msg: Dict[str, Any]) -> None:
        """Dispatch a queued message based on channel type."""
        channel = msg.get("channel")
        if channel == "mqtt":
            self._handle_mqtt_message(msg)
        elif channel == "ws":
            self._handle_ws_message(msg)

    def _handle_mqtt_message(self, msg: Dict[str, Any]) -> None:
        """Handle MQTT control messages."""
        topic = msg.get("topic", "")
        payload = msg.get("payload", "")

        logger.info("[RynnBot] MQTT message: %s", topic)
        if topic.endswith("/acquire_device"):
            self._on_acquire_device(topic, payload)
        elif topic.endswith("/release_device"):
            self._on_release_device(topic, payload)
        elif topic.endswith("/tele_data_coll:start_round"):
            self._on_tele_data_coll_start(topic, payload)
        elif topic.endswith("/tele_data_coll:stop_round"):
            self._on_tele_data_coll_stop(topic, payload)
        elif topic.endswith("/tele_data_coll:upload"):
            self._on_tele_data_coll_upload(topic, payload)
        elif topic.endswith("/skill_execute_record_coll:start_record"):
            self._on_skill_record_start(topic, payload)
        elif topic.endswith("/skill_execute_record_coll:stop_record"):
            self._on_skill_record_stop(topic, payload)
        elif topic.endswith("/skill_execute_record_coll:upload"):
            self._on_skill_record_upload(topic, payload)
        else:
            logger.warning("[RynnBot] Unhandled MQTT topic: %s", topic)

    def _handle_ws_message(self, msg: Dict[str, Any]) -> None:
        """Handle WebSocket binary messages (protobuf)."""
        payload = msg.get("payload")
        if not isinstance(payload, bytes):
            return

        try:
            packet = self._proto.parse_data_packet(payload)
            pkg_type, inner = self._proto.parse_inner_payload(packet)
            logger.debug(
                "[RynnBot] WS packet: type=%s id=%s bytes=%d",
                pkg_type,
                packet.common_part_attr.id,
                len(payload),
            )
            self._dispatch_ws_packet(pkg_type, inner, packet)
        except Exception as e:
            logger.error("[RynnBot] WS parse error: %s", e, exc_info=True)

    def _dispatch_ws_packet(self, pkg_type: int, inner: Any, packet: Any) -> None:
        """Dispatch a decoded WS protobuf packet."""
        PackageType = self._proto.PackageType

        if pkg_type == PackageType.ACTION_DATA:
            self._action_exec.submit(self._execute_action, inner, packet)
        elif pkg_type == PackageType.REQ_IMAGE:
            self._query_exec.submit(self._send_images, inner, packet)
        elif pkg_type == PackageType.REQ_STATE:
            self._query_exec.submit(self._send_state, inner, packet)
        elif pkg_type == PackageType.RESV:
            self._udp_forward_ws_raw(inner)
        else:
            logger.warning("[RynnBot] Unhandled WS package type: %s", pkg_type)

    # ─── MQTT Handlers ───────────────────────────────────────────────

    def _on_acquire_device(self, topic: str, payload: str) -> None:
        """Handle device acquire request."""
        send_topic = topic.replace("/request/", "/response/")
        request_json = self._parse_json_or_reply_error(payload, send_topic, "acquire")
        if request_json is None:
            return

        resp = self._make_base_response(request_json)
        params = request_json.get("params") or {}
        raw_type = params.get("type")
        self._occupied_id = params.get("id")
        try:
            occupy_type = OccupyType(int(raw_type))
        except (TypeError, ValueError):
            occupy_type = OccupyType.UNKNOWN

        with self._occupied_lock:
            if self._occupied:
                resp["error"] = {"code": DEVICE_BUSY_CODE, "message": "Device is busy"}
                logger.warning("[RynnBot] acquire: device already occupied")
            else:
                self._occupied = True
                self._occupy_type = occupy_type
                logger.info(
                    "[RynnBot] acquire: success, occupy_type=%s(%d)",
                    self._occupy_type.name,
                    int(self._occupy_type),
                )

        self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))

    def _on_release_device(self, topic: str, payload: str) -> None:
        """Handle device release request."""
        send_topic = topic.replace("/request/", "/response/")
        request_json = self._parse_json_or_reply_error(payload, send_topic, "release")
        if request_json is None:
            return

        resp = self._make_base_response(request_json)
        with self._occupied_lock:
            self._occupied = False
            self._occupy_type = OccupyType.UNKNOWN
            self._occupied_id = None
            logger.info("[RynnBot] release: success")
        self._capture_manager.force_stop_collection_on_release()
        self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))

    def _on_tele_data_coll_start(self, topic: str, payload: str) -> None:
        """Handle tele data collection start."""
        logger.info("[RynnBot][tele_data_coll:start] topic=%s summary=%s", topic, _mqtt_payload_summary(payload))
        self._capture_manager.start_collection_from_mqtt(topic, payload, kind="tele_data_coll")

    def _on_tele_data_coll_stop(self, topic: str, payload: str) -> None:
        """Handle tele data collection stop."""
        logger.info("[RynnBot][tele_data_coll:stop] topic=%s summary=%s", topic, _mqtt_payload_summary(payload))
        self._capture_manager.stop_collection_from_mqtt(topic, payload, kind="tele_data_coll")

    def _on_tele_data_coll_upload(self, topic: str, payload: str) -> None:
        """Handle tele data collection upload."""
        kind = "skill_record" if self._capture_manager.payload_upload_type(payload) == "skill_execute" else "tele_data_coll"
        logger.info(
            "[RynnBot][tele_data_coll:upload] topic=%s routed_kind=%s summary=%s",
            topic,
            kind,
            _mqtt_payload_summary(payload),
        )
        self._capture_manager.encode_capture_from_mqtt(topic, payload, kind=kind)

    def _on_skill_record_start(self, topic: str, payload: str) -> None:
        """Handle skill record start."""
        logger.info("[RynnBot][skill_record:start] topic=%s summary=%s", topic, _mqtt_payload_summary(payload))
        self._capture_manager.start_collection_from_mqtt(topic, payload, kind="skill_record")

    def _on_skill_record_stop(self, topic: str, payload: str) -> None:
        """Handle skill record stop."""
        logger.info("[RynnBot][skill_record:stop] topic=%s summary=%s", topic, _mqtt_payload_summary(payload))
        self._capture_manager.stop_collection_from_mqtt(topic, payload, kind="skill_record")

    def _on_skill_record_upload(self, topic: str, payload: str) -> None:
        """Handle skill record upload."""
        logger.info("[RynnBot][skill_record:upload] topic=%s summary=%s", topic, _mqtt_payload_summary(payload))
        self._capture_manager.encode_capture_from_mqtt(topic, payload, kind="skill_record")

    def _start_ws_state_stream(self) -> None:
        with self._ws_state_stream_lock:
            if self._ws_state_stream_running:
                return
            self._ws_state_stream_running = True
            self._ws_state_stream_seq = 0
            self._ws_state_stream_thread = threading.Thread(
                target=self._ws_state_stream_loop,
                daemon=True,
                name="rynnbot-state-stream",
            )
            self._ws_state_stream_thread.start()
        logger.info("[RynnBot][WSStateStream] thread started")

    def _stop_ws_state_stream(self) -> None:
        with self._ws_state_stream_lock:
            self._ws_state_stream_running = False
            thread = self._ws_state_stream_thread
        if thread is not None and thread is not threading.current_thread() and thread.is_alive():
            thread.join(timeout=2.0)
        with self._ws_state_stream_lock:
            if self._ws_state_stream_thread is thread:
                self._ws_state_stream_thread = None

    def _ws_state_stream_loop(self) -> None:
        try:
            while True:
                with self._ws_state_stream_lock:
                    if not self._ws_state_stream_running:
                        break
                state = self._capture_manager.capture_state("tele_data_coll")
                recording = bool(state.get("recording"))
                fps = state.get("fps")
                if not recording or not fps:
                    break
                try:
                    self._send_state_via_ws()
                except Exception as exc:
                    logger.warning("[RynnBot][WSStateStream] send state failed: %s", exc, exc_info=True)
                time.sleep(1.0 / max(float(fps), 0.1))
        finally:
            with self._ws_state_stream_lock:
                self._ws_state_stream_running = False
                if self._ws_state_stream_thread is threading.current_thread():
                    self._ws_state_stream_thread = None
            logger.info("[RynnBot][WSStateStream] loop ended")

    def _send_state_via_ws(self) -> None:
        state_name = self._state_observation_name()
        if self._server_client is None or not state_name:
            return
        obs = self._get_observations([state_name])
        values = _joint_positions(obs.get(state_name))
        multi = self._proto.MultiState()
        if values is not None:
            arr = np.asarray(values, dtype=np.float32)
            item = multi.state_list.add()
            item.id = 1
            item.name = self._protocol_to_cloud_key(state_name)
            item.state_data.data = arr.tobytes()
            item.state_data.shape.extend(list(arr.shape))
            item.state_data.dtype = self._proto.DataType.FLOAT32
        with self._ws_state_stream_lock:
            seq_id = self._ws_state_stream_seq
            self._ws_state_stream_seq += 1
        packet = self._proto.build_data_packet(self._proto.PackageType.STATE_DATA, multi, id=seq_id)
        self._ws_client.send_bytes(self._proto.serialize_data_packet(packet))

    # ─── WS Action/Query Handlers ────────────────────────────────────

    def _execute_action(self, action_msg: Any, packet: Any | None = None) -> None:
        """Execute an action received via WS."""
        if self._server_client is None:
            return
        frames_sent = 0
        expect_frames = 0
        error_msg = ""
        try:
            action_list = action_msg.action_list if hasattr(action_msg, 'action_list') else []
            for action in action_list:
                arr_msg = action.action_data
                np_dtype = self._proto_dtype_to_np(arr_msg.dtype)
                if np_dtype is None:
                    error_msg = f"unsupported dtype: {arr_msg.dtype}"
                    continue
                shape = list(arr_msg.shape)
                data = np.frombuffer(arr_msg.data, dtype=np_dtype)
                if shape:
                    data = data.reshape(shape)
                else:
                    shape = list(data.shape)
                action_payload = data.astype(float).tolist()
                if not action_payload:
                    continue
                logger.info(
                    "[RynnBot][WS][ACTION] id=%s name=%s shape=%s rate=%s",
                    action.id,
                    action.name,
                    shape,
                    getattr(action, "action_rate", 0),
                )
                action_name = self._action_name(str(getattr(action, "name", "") or ""))
                if not action_name:
                    error_msg = "no protocol action is available"
                    continue
                frames = _action_frames(action_payload)
                result = self._server_client.run_action_chunk(
                    action_name,
                    frames,
                    frame_rate=int(getattr(action, "action_rate", 30) or 30),
                )
                expect_frames += len(frames)
                if result.ok:
                    frames_sent += len(frames)
                else:
                    error_msg = result.message or "run_action_chunk failed"
        except Exception as e:
            error_msg = str(e)
            logger.error("[RynnBot] Execute action failed: %s", e, exc_info=True)
        finally:
            self._send_action_finish(
                packet=packet,
                code=0 if not error_msg else -1,
                error_msg=error_msg,
                execute_steps=frames_sent,
                expect_steps=expect_frames,
            )

    def _send_images(self, req: Any, packet: Any | None = None) -> None:
        """Send images in response to a WS request."""
        if self._server_client is None:
            return
        requested = [cam.camera_name for cam in getattr(req, "camera", [])]
        try:
            image_keys = self._requested_image_keys(req)
            logger.info(
                "[RynnBot][WS][REQ_IMAGE] request cameras=%s resolved_keys=%s available_keys=%s aliases=%s",
                requested,
                image_keys,
                self._image_observation_names(),
                self._image_observation_aliases(),
            )
            if image_keys:
                self._send_ws_images(self._get_observations(image_keys), req=req, packet=packet)
            else:
                logger.warning(
                    "[RynnBot][WS][REQ_IMAGE] no image key resolved: requested=%s available_keys=%s",
                    requested,
                    self._image_observation_names(),
                )
                self._reply_empty_image(packet)
        except Exception as e:
            logger.error("[RynnBot] Send images failed: %s", e, exc_info=True)
            self._reply_empty_image(packet)

    def _send_state(self, req: Any, packet: Any | None = None) -> None:
        """Send state in response to a WS request."""
        if self._server_client is None:
            return
        logger.debug(
            "[RynnBot][WS][REQ_STATE] request robots=%d",
            len(getattr(req, "robot", [])),
        )
        try:
            state_name = self._state_observation_name()
            if state_name:
                self._send_ws_state(self._get_observations([state_name]), packet=packet)
        except Exception as e:
            logger.error("[RynnBot] Send state failed: %s", e, exc_info=True)
            self._reply_empty_state(packet)

    def _send_ws_images(self, images: Any, req: Any | None = None, packet: Any | None = None) -> None:
        """Encode images to protobuf and send via WebSocket."""
        obs = images if isinstance(images, dict) else {}
        multi = self._proto.MultiImage()
        requested = [cam.camera_name for cam in getattr(req, "camera", []) if cam.camera_name]
        image_items = [
            (protocol_name, obs.get(protocol_name), cloud_name)
            for cloud_name in requested
            if (protocol_name := self._image_observation_name(str(cloud_name)))
        ] if requested else [
            (key, value, key) for key, value in obs.items() if str(key) in self._image_observation_names()
        ]
        camera_by_name = {
            cam.camera_name: cam for cam in getattr(req, "camera", []) if cam.camera_name
        }
        sent = 0
        for key, value, cloud_name in image_items:
            if not isinstance(value, dict):
                logger.warning(
                    "[RynnBot][WS][REQ_IMAGE] skip %s: value is not an image object, summary=%s",
                    key,
                    _observation_value_summary(value),
                )
                continue
            raw = _coerce_image_bytes(value.get("image"))
            if raw is None:
                logger.warning(
                    "[RynnBot][WS][REQ_IMAGE] skip %s: missing image bytes, value_summary=%s",
                    key,
                    _observation_value_summary(value),
                )
                continue
            try:
                if self._image_upload_codec == "jpeg":
                    # JPEG mode: upload compressed bytes directly, skip npy+gzip
                    encoding = str(value.get("encoding") or "")
                    if encoding in ("jpg", "jpeg"):
                        # Already JPEG, use raw bytes directly
                        encoded = raw
                        img = Image.open(io.BytesIO(encoded))
                        shape = [int(img.height), int(img.width), 3]
                    else:
                        # Re-encode to JPEG
                        img = Image.open(io.BytesIO(raw)).convert("RGB")
                        jpeg_buf = io.BytesIO()
                        img.save(jpeg_buf, format="JPEG", quality=self._image_jpeg_quality)
                        encoded = jpeg_buf.getvalue()
                        shape = [int(img.height), int(img.width), 3]
                    image_format = "jpeg"
                else:
                    # npy_gzip mode: decode → numpy → gzip
                    img = Image.open(io.BytesIO(raw)).convert("RGB")
                    arr = np.asarray(img, dtype=np.uint8)
                    buf = io.BytesIO()
                    np.save(buf, arr)
                    compressed = gzip.compress(buf.getvalue(), compresslevel=1)
                    encoded = compressed
                    shape = list(arr.shape)
                    image_format = "npy_gzip"
            except Exception:
                encoded = raw
                shape = [len(raw)]
                image_format = str(value.get("encoding", "bytes"))

            response_name = self._protocol_to_cloud_key(str(key))
            req_cam = camera_by_name.get(str(cloud_name or "")) or camera_by_name.get(response_name)
            item = multi.image_list.add()
            item.id = int(req_cam.camera_id) if req_cam is not None else len(multi.image_list)
            item.name = response_name
            item.format = image_format
            item.image_data.data = encoded
            item.image_data.shape.extend(shape)
            item.image_data.dtype = self._proto.DataType.UINT8
            sent += 1
            logger.info(
                "[RynnBot][WS][REQ_IMAGE] response %s -> %s format=%s shape=%s raw_bytes=%d encoded_bytes=%d",
                key,
                response_name,
                image_format,
                shape,
                len(raw),
                len(encoded),
            )
        if sent == 0:
            logger.warning(
                "[RynnBot][WS][REQ_IMAGE] response has no images: requested=%s image_items=%s obs_keys=%s",
                requested,
                [(key, cloud_name) for key, _, cloud_name in image_items],
                sorted(obs.keys()),
            )
        self._ws_send_packet(self._proto.PackageType.IMAGE_DATA, multi, packet)

    def _requested_image_keys(self, req: Any | None) -> List[str]:
        requested = [
            str(cam.camera_name)
            for cam in getattr(req, "camera", [])
            if getattr(cam, "camera_name", None)
        ]
        if requested:
            return [name for item in requested if (name := self._image_observation_name(item))]
        configured = self.config.get("image_keys")
        if configured:
            return [name for item in configured if (name := self._image_observation_name(str(item)))]
        return self._image_observation_names()

    def _image_observation_name(self, requested: str) -> str:
        aliases = self._image_observation_aliases()
        mapped = aliases.get(str(requested), self._cloud_to_protocol_key(str(requested)))
        return mapped if mapped in set(self._image_observation_names()) else ""

    def _image_observation_aliases(self) -> Dict[str, str]:
        aliases: Dict[str, str] = {}
        for name in self._image_observation_names():
            aliases[name] = name
            parts = name.split(".")
            if len(parts) == 3 and parts[0] == "observation" and parts[2] == "image":
                component = parts[1]
                short = component.removesuffix("_camera")
                aliases[component] = name
                aliases[short] = name
                aliases[f"observation.images.{component}"] = name
                aliases[f"observation.images.{short}"] = name
        return aliases

    def _send_ws_state(self, state: Any, packet: Any | None = None) -> None:
        """Encode state to protobuf and send via WebSocket."""
        obs = state if isinstance(state, dict) else {}
        state_name = self._state_observation_name()
        values = _joint_positions(obs.get(state_name))
        multi = self._proto.MultiState()
        if values is not None:
            arr = np.asarray(values, dtype=np.float32)
            item = multi.state_list.add()
            item.id = 1
            item.name = self._protocol_to_cloud_key(state_name)
            item.state_data.data = arr.tobytes()
            item.state_data.shape.extend(list(arr.shape))
            item.state_data.dtype = self._proto.DataType.FLOAT32
            logger.info(
                "[RynnBot][WS][REQ_STATE] %s shape=%s bytes=%d",
                state_name,
                list(arr.shape),
                len(item.state_data.data),
            )
        else:
            logger.warning("[RynnBot][WS][REQ_STATE] joint_state observation missing")
        self._ws_send_packet(self._proto.PackageType.STATE_DATA, multi, packet)

    def _send_action_finish(
        self,
        *,
        packet: Any | None = None,
        code: int,
        error_msg: str,
        execute_steps: int,
        expect_steps: int,
    ) -> None:
        msg = self._proto.FinishActionChunk()
        msg.code = int(code)
        msg.error_msg = error_msg
        msg.execute_steps = int(execute_steps)
        msg.expect_steps = int(expect_steps)
        self._ws_send_packet(self._proto.PackageType.ACTION_FINISH, msg, packet)

    def _ws_send_packet(self, pkg_type: int, inner_msg: Any, req_packet: Any | None = None) -> None:
        packet_id = 0
        if req_packet is not None:
            packet_id = int(getattr(req_packet.common_part_attr, "id", 0))
        packet = self._proto.build_data_packet(pkg_type, inner_msg, id=packet_id)
        self._ws_client.send_bytes(self._proto.serialize_data_packet(packet))

    def _reply_empty_image(self, packet: Any | None = None) -> None:
        self._ws_send_packet(self._proto.PackageType.IMAGE_DATA, self._proto.MultiImage(), packet)

    def _reply_empty_state(self, packet: Any | None = None) -> None:
        self._ws_send_packet(self._proto.PackageType.STATE_DATA, self._proto.MultiState(), packet)

    def _udp_forward_ws_raw(self, payload: Any) -> None:
        if self._udp_socket is None or payload is None:
            return
        data = bytes(payload) if isinstance(payload, (bytes, bytearray, memoryview)) else payload
        if not isinstance(data, bytes):
            return
        with self._udp_socket_lock:
            if self._udp_socket is not None:
                self._udp_socket.sendto(data, ("localhost", int(self._udp_port)))

    def _mqtt_send(self, topic: str, payload: str, qos: int = 1) -> None:
        self._mqtt_client.publish(topic, payload, qos=qos)

    def _post_upload_event(self, event_name: str, params: Dict[str, Any], qos: int = 1) -> None:
        now_ms = int(time.time() * 1000)
        payload = {
            "jsonrpc": "2.0",
            "id": str(now_ms),
            "method": f"dm.event.{event_name}.post",
            "params": {"$TIME": now_ms, **(params or {})},
        }
        topic = f"/sys/{self._product_key}/{self._device_name_cfg}/dm/event/{event_name}/post"
        self._mqtt_send(topic, json.dumps(payload, ensure_ascii=False), qos=qos)
        logger.info("[RynnBot] upload event posted: %s params=%s", event_name, payload["params"])

    def _post_occupancy_error(self, *, error_code: int, error_msg: str = "", qos: int = 1) -> None:
        now_ms = int(time.time() * 1000)
        payload = {
            "jsonrpc": "2.0",
            "id": str(self._occupied_id or now_ms),
            "method": "dm.event.occupancy_error.post",
            "params": {
                "$TIME": now_ms,
                "error_code": int(error_code),
            },
        }
        if error_msg:
            payload["params"]["error_msg"] = str(error_msg)
        topic = f"/sys/{self._product_key}/{self._device_name_cfg}/dm/event/occupancy_error/post"
        self._mqtt_send(topic, json.dumps(payload, ensure_ascii=False), qos=qos)
        logger.info("[RynnBot] occupancy_error posted: params=%s", payload["params"])

    def _clear_occupancy_after_upload_failure(self) -> None:
        with self._occupied_lock:
            if not self._occupied:
                return
            self._occupied = False
            self._occupy_type = OccupyType.UNKNOWN
            self._occupied_id = None

    def _parse_json_or_reply_error(
        self, payload: str, send_topic: str, op_name: str
    ) -> Optional[dict]:
        try:
            return json.loads(payload)
        except Exception as exc:
            logger.error("[RynnBot][%s] parse error: %s", op_name, exc, exc_info=True)
            error_resp = {
                "jsonrpc": "2.0",
                "id": "-1",
                "error": {
                    "code": JSON_PARSE_ERROR_CODE,
                    "message": f"Failed to parse JSON: {exc}",
                },
            }
            self._mqtt_send(send_topic, json.dumps(error_resp, ensure_ascii=False))
            return None

    @staticmethod
    def _make_base_response(request_json: dict) -> dict:
        return {
            "jsonrpc": "2.0",
            "id": request_json.get("id", "-1"),
            "result": {"$TIME": int(time.time() * 1000)},
        }

    def _proto_dtype_to_np(self, dtype_proto: Any) -> Optional[np.dtype]:
        mapping = {
            self._proto.DataType.UINT8: np.uint8,
            self._proto.DataType.INT8: np.int8,
            self._proto.DataType.UINT16: np.uint16,
            self._proto.DataType.INT16: np.int16,
            self._proto.DataType.INT32: np.int32,
            self._proto.DataType.FLOAT32: np.float32,
            self._proto.DataType.FLOAT64: np.float64,
        }
        return mapping.get(dtype_proto)

    # ─── Device Monitor ──────────────────────────────────────────────

    def _start_device_monitor_thread(self) -> None:
        """Start background thread that reports device properties via MQTT."""
        self._device_monitor_running = True
        self._device_monitor_thread = threading.Thread(
            target=self._device_monitor_loop, daemon=True, name="rynnbot-monitor"
        )
        self._device_monitor_thread.start()

    def _device_monitor_loop(self) -> None:
        """Periodically report device status."""
        while self._device_monitor_running:
            try:
                self._report_device_properties()
            except Exception as e:
                logger.warning("[RynnBot][DeviceMonitor] report error: %s", e, exc_info=True)
            time.sleep(self._device_monitor_interval_s)

    def _report_device_properties(self) -> None:
        """Call get_device_info and report the result to RynnBot cloud."""
        message = self._collect_device_properties()
        if not message:
            return
        topic = f"sys/{self._product_key}/{self._device_name_cfg}/dm/property/post"
        payload = json.dumps(message, ensure_ascii=False)
        self._mqtt_client.publish(topic, payload, qos=1)
        params = message.get("params") or {}
        logger.info(
            "[RynnBot][DeviceMonitor] property report published: topic=%s keys=%s",
            topic,
            sorted(params.keys()),
        )

    def _collect_device_properties(self) -> Dict[str, Any]:
        """Collect RynnBot cloud device properties for reporting."""
        if self._server_client is None or self._manifest is None:
            return {}
        health_payload: Dict[str, Any] = {}
        try:
            health_resp = self._server_client.get_health()
        except Exception as exc:
            logger.warning("[RynnBot][DeviceMonitor] get_health call failed: %s", exc, exc_info=True)
        else:
            if health_resp.ok and isinstance(health_resp.payload, dict):
                health_payload = dict(health_resp.payload)
            else:
                logger.warning("[RynnBot][DeviceMonitor] get_health failed: %s", health_resp.message)

        params = _host_device_properties()
        params.update({
            "arm_info": self._manifest.robot_name,
            "camera_info": self._camera_info_for_cloud(),
            "robot_id": self._manifest.robot_id,
            "robot_name": self._manifest.robot_name,
            "embodiment_type": self._manifest.embodiment_type,
            "components": self._manifest.components,
            "observations": self._cloud_observation_properties(),
            "actions": self._cloud_action_properties(),
            "capabilities": self._manifest.capabilities,
            "model_refs": self._manifest.model_refs,
            "metadata": self._manifest.metadata,
            "health": health_payload,
        })

        self._device_info_cache.update(params)
        msg_id = int(time.time() * 1000)
        return {
            "jsonrpc": "2.0",
            "id": str(msg_id),
            "method": "dm.property.post",
            "params": params,
        }

    def _cloud_observation_properties(self) -> List[Dict[str, Any]]:
        return [
            {**dict(item), "name": self._protocol_to_cloud_key(str(item["name"]))}
            for item in self._manifest.observations
            if item.get("name")
        ]

    def _cloud_action_properties(self) -> List[Dict[str, Any]]:
        return [
            {**dict(item), "name": self._protocol_to_cloud_key(str(item["name"]))}
            for item in self._manifest.actions
            if item.get("name")
        ]

    def _camera_info_for_cloud(self) -> List[Dict[str, Any]]:
        if self._camera_info_cache:
            return [dict(item) for item in self._camera_info_cache]

        image_names = self._image_observation_names()
        if not image_names:
            logger.warning("[RynnBot][DeviceMonitor] no image observations in manifest")
        image_values = self._get_observations(image_names) if image_names else {}
        cameras: List[Dict[str, Any]] = []
        for item in self._manifest.observations:
            if item.get("type") != "image" or not item.get("name"):
                continue
            protocol_name = str(item["name"])
            cloud_name = self._protocol_to_cloud_key(protocol_name)
            camera: Dict[str, Any] = {
                "id": len(cameras),
                "name": cloud_name,
                "brand": str(item.get("component_name") or cloud_name),
            }
            _update_camera_size_from_observation(camera, image_values.get(protocol_name))
            cameras.append(camera)
        logger.info(
            "[RynnBot][DeviceMonitor] camera_info resolved: image_keys=%s image_values=%s cameras=%s",
            image_names,
            {key: _observation_value_summary(value) for key, value in image_values.items()},
            cameras,
        )
        if cameras and all("width" in item and "height" in item for item in cameras):
            self._camera_info_cache = [dict(item) for item in cameras]
        return cameras

    # ─── Health Check ────────────────────────────────────────────────

    def health_check(self) -> Dict[str, Any]:
        base = super().health_check()
        base.update({
            "product_key": self._product_key,
            "device_name": self._device_name_cfg,
            "mqtt_connected": hasattr(self._mqtt_client, '_connected') and self._mqtt_client._connected.is_set(),
            "occupied": self._occupied,
            "occupy_type": self._occupy_type.name if self._occupied else None,
        })
        return base


def _joint_positions(value: Any) -> Optional[List[float]]:
    if isinstance(value, dict):
        value = value.get("joint_positions")
    if value is None:
        return None
    values = np.asarray(value, dtype=np.float32).reshape(-1)
    return values.astype(float).tolist()


def _action_frames(value: Any) -> List[Dict[str, Any]]:
    arr = np.asarray(value, dtype=np.float32)
    if arr.ndim <= 1:
        return [{"joint_positions": arr.reshape(-1).astype(float).tolist()}]
    return [
        {"joint_positions": frame.reshape(-1).astype(float).tolist()}
        for frame in arr
    ]


def _coerce_image_bytes(value: Any) -> Optional[bytes]:
    if value is None:
        return None
    if isinstance(value, bytes):
        return value
    if isinstance(value, bytearray):
        return bytes(value)
    if isinstance(value, memoryview):
        return value.tobytes()
    if isinstance(value, str):
        try:
            return base64.b64decode(value, validate=True)
        except Exception:
            return value.encode("utf-8")
    if hasattr(value, "tobytes"):
        return value.tobytes()
    return None


def _observation_value_summary(value: Any) -> Dict[str, Any]:
    if isinstance(value, dict):
        summary: Dict[str, Any] = {"type": "dict", "keys": sorted(str(key) for key in value.keys())}
        if "image" in value:
            raw = _coerce_image_bytes(value.get("image"))
            summary["image_bytes"] = len(raw) if raw is not None else 0
        for key in ("width", "height", "encoding", "timestamp", "ts"):
            if key in value:
                summary[key] = value.get(key)
        joints = value.get("joint_positions")
        if joints is not None:
            try:
                summary["joint_positions_len"] = int(np.asarray(joints).reshape(-1).size)
            except Exception:
                summary["joint_positions_type"] = type(joints).__name__
        return summary
    if isinstance(value, (bytes, bytearray, memoryview)):
        return {"type": type(value).__name__, "bytes": len(value)}
    if isinstance(value, (list, tuple)):
        return {"type": type(value).__name__, "len": len(value)}
    if value is None:
        return {"type": "None"}
    return {"type": type(value).__name__}


def _update_camera_size_from_observation(camera: Dict[str, Any], value: Any) -> None:
    if not isinstance(value, dict):
        return
    width = value.get("width")
    height = value.get("height")
    if isinstance(width, bool) or isinstance(height, bool):
        return
    if isinstance(width, (int, float)) and isinstance(height, (int, float)):
        camera["width"] = int(width)
        camera["height"] = int(height)
    encoding = value.get("encoding")
    if encoding:
        camera["encoding"] = str(encoding)


def _mqtt_payload_summary(payload: Any) -> Dict[str, Any]:
    if isinstance(payload, str):
        payload_bytes = len(payload.encode("utf-8"))
        payload_text = payload
    elif isinstance(payload, (bytes, bytearray, memoryview)):
        payload_bytes = len(payload)
        payload_text = bytes(payload).decode("utf-8", errors="replace")
    else:
        payload_text = str(payload)
        payload_bytes = len(payload_text.encode("utf-8"))
    try:
        request = json.loads(payload_text)
    except Exception as exc:
        return {"payload_bytes": payload_bytes, "parse_error": str(exc)}
    if not isinstance(request, dict):
        return {"payload_bytes": payload_bytes, "payload_type": type(request).__name__}

    params = request.get("params")
    summary: Dict[str, Any] = {"payload_bytes": payload_bytes}
    if request.get("id") is not None:
        summary["id"] = request.get("id")
    if request.get("version") is not None:
        summary["version"] = request.get("version")
    if isinstance(params, dict):
        allowed = (
            "id",
            "record_id",
            "record_ids",
            "recordId",
            "recordIdList",
            "capture_id",
            "capture_dir",
            "capture_dirs",
            "data_coll_id",
            "task_id",
            "sub_task_id",
            "round_number",
            "round",
            "episode_id",
            "episode_number",
            "fps",
            "rate",
            "expected_steps",
            "max_duration_s",
            "max_duration_sec",
            "max_duration",
            "timeout_s",
            "names",
            "keys",
            "task_keys",
            "upload_type",
            "zip_name",
            "output_dir",
            "package_dir",
            "delete_uploaded_package",
            "delete_uploaded_raw_captures",
        )
        summary["params"] = {key: _brief_log_value(params[key]) for key in allowed if key in params}
    else:
        summary["params_type"] = type(params).__name__
    return summary


def _brief_log_value(value: Any) -> Any:
    if isinstance(value, str):
        return value if len(value) <= 200 else value[:197] + "..."
    if isinstance(value, (int, float, bool)) or value is None:
        return value
    if isinstance(value, (list, tuple)):
        items = list(value)
        if len(items) <= 12:
            return items
        return {"len": len(items), "items": items[:12]}
    if isinstance(value, dict):
        return {"keys": sorted(str(key) for key in value.keys())}
    return {"type": type(value).__name__}


def _host_device_properties() -> Dict[str, Any]:
    mem_total = "unknown"
    mem_used = "unknown"
    cpu_load = "unknown"
    try:
        import psutil

        vm = psutil.virtual_memory()
        mem_total = f"{vm.total / 1024:.0f}"
        mem_used = f"{(vm.total - vm.available) / 1024:.0f}"
        cpu_load = f"{psutil.cpu_percent():.2f}%"
    except Exception:
        pass
    return {
        "arch": platform.machine() or "unknown",
        "distrib_desc": _platform_description(),
        "kernel": platform.release() or "unknown",
        "mem": mem_total,
        "cpu_load": cpu_load,
        "mem_used": mem_used,
    }


def _platform_description() -> str:
    try:
        if platform.system() == "Linux":
            try:
                import distro

                return distro.name(pretty=True) or "Linux"
            except Exception:
                return platform.platform()
        if platform.system() == "Darwin":
            return f"macOS {platform.mac_ver()[0]}"
        if platform.system() == "Windows":
            return f"Windows {platform.version()}"
        return platform.system() or "unknown"
    except Exception:
        return "unknown"


def _target_robot_config(config: Dict[str, Any]) -> Dict[str, Any]:
    target = config.get("target_robot")
    if not isinstance(target, dict):
        raise ValueError("target_robot must be configured")
    robot_id = str(target.get("robot_id") or "").strip()
    if not robot_id:
        raise ValueError("target_robot.robot_id is required")
    return dict(target)


def _bind_target_robot_config(config: Dict[str, Any], server_config: str) -> Dict[str, Any]:
    merged = dict(config)
    raw = load_yaml(server_config)
    manifest = raw.get("manifest") if isinstance(raw, dict) else None
    if not isinstance(manifest, dict) or not str(manifest.get("robot_id") or "").strip():
        raise ValueError(f"{server_config} must define manifest.robot_id")
    target = dict(merged.get("target_robot") or {})
    target["robot_id"] = str(manifest["robot_id"])
    merged["target_robot"] = target
    return merged


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Run the RynnRCP RynnBot app.")
    parser.add_argument("--config", required=True, help="RynnBot app YAML config path.")
    parser.add_argument("--server-config", required=True, help="RynnRCP server YAML config used to select target server.")
    args = parser.parse_args(argv)
    config = merge_default_config(load_yaml(args.config))
    config = _bind_target_robot_config(config, args.server_config)
    configure_logging(
        level=logging.INFO,
        sinks=["stderr", "file"],
        file_path=str(log_file_from_config(config, "rynnbot.log")),
    )
    app = RynnBotApp(config=config)
    try:
        app.start()
        while app.is_running:
            time.sleep(0.5)
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"Failed to start RynnBot app: {exc}", file=sys.stderr)
        return 2
    finally:
        app.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
