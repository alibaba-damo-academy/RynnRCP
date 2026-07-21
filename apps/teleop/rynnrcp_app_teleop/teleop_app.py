"""Interface/gRPC based Teleop app.

This app talks to RCP servers only through protocol methods:

- leader server: read Observations through get_observations
- follower server: submit Actions through run_action_chunk
- follower server: record datasets through start_collection / stop_collection
"""

from __future__ import annotations

import argparse
import base64
import logging
import shutil
import socket
import statistics
import sys
import tempfile
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Deque, Optional

import yaml

from rynnrcp.interface.client import ClientInterface
from rynnrcp.interface.codec import InterfaceError
from rynnrcp.interface.protocol_client import RcpProtocolClient, ServerManifest, discover_manifests
from rynnrcp.utils import safe_name
from rynnrcp.utils.logging import configure_logging
from rynnrcp.utils.user_paths import collections_dir, log_file_from_config, local_root_from_config, resolve_robot_path, robot_root
from rynnrcp_app_common import AppLifecycle
from rynnrcp_app_common.resource_transfer import download_collection_entries, download_collection_resource

from .playback import TeleopPlaybackManager
from .recording_manager import TeleopRecordingManager
from .utils import format_bytes, payload_size, resize_jpeg_long_edge
from .web_ui import TeleopWebUI


DEFAULT_MAX_RECORD_DURATION_S = 10 * 60
_HEARTBEAT_TIMEOUT = 3.0
_RESOURCE_EPISODE_PREFIX = "rcp-resource://"


def _export_progress_message(message: str) -> str:
    text = str(message or "")
    if text.startswith("encoding "):
        return "正在编码 " + text.removeprefix("encoding ")
    if text == "encoding finished":
        return "编码完成，准备打包..."
    return text or "正在导出..."


@dataclass
class TeleopBinding:
    source: ServerManifest
    target: ServerManifest


@dataclass(frozen=True)
class ControlFlow:
    source_observation: str
    source_type: str
    target_action: str
    target_type: str


class TeleopApp(AppLifecycle):
    """Standalone teleop app that controls two Interface servers."""

    VERSION = "0.2.0"
    DESCRIPTION = "Interface-based teleoperation app"

    def __init__(self, name: str = "teleop", config: Optional[dict[str, Any]] = None) -> None:
        super().__init__(name, _merge_default_config(config))
        app_cfg = self.config.get("app") if isinstance(self.config.get("app"), dict) else {}
        self._role = "leader"
        self._host = str(app_cfg.get("web_host") or self.config.get("web_host") or "0.0.0.0")
        self._port = int(_config_value(app_cfg, self.config, "web_port", 28402))
        self._open_browser = bool(app_cfg.get("open_browser", self.config.get("open_browser", True)))
        self._control_hz = float(app_cfg.get("control_hz") or self.config.get("control_hz") or 60)
        self._state_hz = float(app_cfg.get("state_hz") or self.config.get("state_hz") or 30)
        self._status_hz = float(app_cfg.get("status_hz") or self.config.get("status_hz") or 2)
        self._image_hz = float(app_cfg.get("image_hz") or self.config.get("image_hz") or 10)
        self._connection_monitor_interval_s = float(
            app_cfg.get("connection_monitor_interval_s")
            or self.config.get("connection_monitor_interval_s")
            or 0.5
        )
        self._connection_reconnect_scan_s = float(
            app_cfg.get("connection_reconnect_scan_s")
            or self.config.get("connection_reconnect_scan_s")
            or 2.0
        )
        self._connection_monitor_timeout_ms = int(
            app_cfg.get("connection_monitor_timeout_ms")
            or self.config.get("connection_monitor_timeout_ms")
            or min(self._request_timeout_ms(), 500)
        )
        self._error_log_interval_s = float(
            app_cfg.get("error_log_interval_s")
            or self.config.get("error_log_interval_s")
            or 5.0
        )
        self._record_max_duration_s = float(
            app_cfg.get("max_record_duration_s")
            or self.config.get("max_record_duration_s")
            or DEFAULT_MAX_RECORD_DURATION_S
        )
        self._robot_root = local_root_from_config(self.config)
        self._profile_path = _optional_path(
            app_cfg.get("profile_path") or self.config.get("profile_path"),
            self._robot_root,
        )

        self._binding: TeleopBinding | None = None
        self._source_client: RcpProtocolClient | None = None
        self._target_client: RcpProtocolClient | None = None
        self._source_observation_stream: Any | None = None
        self._teleop_observation_name = ""
        self._teleop_observation_type = ""
        self._target_observation_name = ""
        self._target_observation_type = ""
        self._teleop_action_name = ""
        self._teleop_action_type = ""
        self._control_flows: list[ControlFlow] = []

        self._teleop_enabled = False
        self._stop_event = threading.Event()
        self._preview_demand_event = threading.Event()
        self._lock = threading.RLock()
        self._state_lock = threading.RLock()
        self._images_lock = threading.RLock()
        self._metrics_lock = threading.RLock()
        self._record_status_lock = threading.RLock()
        self._connection_status_lock = threading.RLock()
        self._available_buffer_keys_lock = threading.RLock()
        self._preview_demand_lock = threading.RLock()
        self._error_log_lock = threading.RLock()
        self._preview_keys_lock = threading.RLock()

        self._control_thread: threading.Thread | None = None
        self._status_thread: threading.Thread | None = None
        self._image_thread: threading.Thread | None = None
        self._monitor_thread: threading.Thread | None = None
        self._web_ui: TeleopWebUI | None = None

        self._latest_peer_state: dict[str, Any] | None = None
        self._latest_local_state: dict[str, Any] | None = None
        self._latest_peer_images: dict[str, bytes] | None = None
        self._latest_peer_state_seq = 0
        self._latest_local_state_seq = 0
        self._latest_peer_images_seq = 0
        self._latest_peer_state_at = 0.0
        self._latest_peer_images_at = 0.0
        self._last_heartbeat_recv = 0.0
        self._peer_connected = False
        self._connection_status: dict[str, dict[str, Any]] = {
            "source": _connection_status("unbound"),
            "target": _connection_status("unbound"),
        }
        self._reconnect_candidates: dict[str, ServerManifest | None] = {"source": None, "target": None}
        self._peer_preview_requested = False
        self._preview_image_keys: list[str] | None = None
        self._source_state_keys: list[str] | None = None
        self._target_state_keys: list[str] | None = None
        self._last_error = ""
        self._loop_error_logs: dict[str, dict[str, Any]] = {}
        self._frames_sent = 0
        self._last_sent_source_ts_by_flow: dict[tuple[str, str], float] = {}

        self._metric_events: dict[str, Deque[float]] = {
            "command_tx": deque(),
            "command_rx": deque(),
            "image_tx": deque(),
            "image_rx": deque(),
        }
        self._metric_bytes: dict[str, int] = {
            "command_tx_bytes": 0,
            "command_rx_bytes": 0,
            "image_tx_bytes": 0,
            "image_rx_bytes": 0,
        }
        self._action_latency_window: Deque[float] = deque(maxlen=300)
        self._action_rpc_window: Deque[float] = deque(maxlen=300)
        self._observation_rpc_window: Deque[float] = deque(maxlen=300)
        self._source_state_rpc_window: Deque[float] = deque(maxlen=300)
        self._target_state_rpc_window: Deque[float] = deque(maxlen=300)
        self._image_rpc_window: Deque[float] = deque(maxlen=300)

        self._latest_record_status: dict[str, Any] = {
            "recording": False,
            "frames_written": 0,
            "episode_dir": None,
            "episode_number": 0,
            "episode_id": "",
            "collection_id": "",
            "collection_resource": {},
        }
        self._collection_id = f"teleop_dataset_{int(time.time())}"
        self._current_episode = 0
        self._collection_config_locked = False
        self._collection_config: dict[str, Any] = {}
        self._available_buffer_keys: list[str] = []
        self._export_in_progress = False
        self._export_progress: dict[str, Any] | None = None
        self._export_result: dict[str, Any] | None = None
        record_dir = resolve_robot_path(
            str(self.config.get("record_dir") or "teleop_records"),
            self._robot_root,
        )
        self.records = TeleopRecordingManager(str(record_dir), self.config)
        self.playback = TeleopPlaybackManager(
            config=self.config,
            records=self.records,
            get_protocol_client=lambda: self._target_client,
            get_observation_name=lambda: self._target_observation_name,
            get_action_name=lambda: self._teleop_action_name,
            send_status=lambda payload: self._set_playback_status(payload),
            logger=self._logger,
        )

    # ------------------------------------------------------------------
    # Discovery and binding
    # ------------------------------------------------------------------
    def discover(self) -> list[ServerManifest]:
        manifests = discover_manifests(
            interface=self._client(),
            timeout_s=self._discovery_timeout_s(),
            request_timeout_ms=self._request_timeout_ms(),
            dedupe_instances=False,
        )
        return _dedupe_robot_manifests(manifests)

    def bind_servers(
        self,
        source_robot_id: str,
        target_robot_id: str,
        control_flows: list[dict[str, Any]] | None = None,
    ) -> TeleopBinding:
        manifests = self.discover()
        source = _select_unique(manifests, source_robot_id)
        target = _select_unique(manifests, target_robot_id)
        if source.robot_id == target.robot_id:
            raise ValueError(f"source and target robot must be different: {source.robot_id}")
        if not source.can_provide_state:
            raise ValueError(f"source robot {source.robot_id!r} does not provide observations")
        if not target.can_accept_action:
            raise ValueError(f"target robot {target.robot_id!r} does not accept actions")
        if source.embodiment_type != target.embodiment_type:
            raise ValueError(
                f"embodiment mismatch: source={source.embodiment_type!r}, target={target.embodiment_type!r}"
            )
        flows = _build_control_flows(source, target, control_flows)
        target_observation = _select_target_state_observation(target, flows[0].source_type)
        client = self._client()
        source_client = _connect_manifest(source, client)
        target_client = _connect_manifest(target, client)
        with self._lock:
            self._close_clients()
            self._binding = TeleopBinding(source=source, target=target)
            self._source_client = source_client
            self._target_client = target_client
            self._control_flows = list(flows)
            self._teleop_observation_name = flows[0].source_observation
            self._teleop_observation_type = flows[0].source_type
            self._target_observation_name = str(target_observation["name"])
            self._target_observation_type = str(target_observation["type"])
            self._teleop_action_name = flows[0].target_action
            self._teleop_action_type = flows[0].target_type
            self._set_connection_status("source", "online", source, message="")
            self._set_connection_status("target", "online", target, message="")
            self._reconnect_candidates["source"] = None
            self._reconnect_candidates["target"] = None
            self._reset_protocol_key_cache()
            self._set_available_protocol_names()
            self._save_profile()
        self._logger.info(
            "Teleop bound source=%s target=%s flows=%s target_observation=%s",
            source.robot_id,
            target.robot_id,
            [f"{flow.source_observation}->{flow.target_action}" for flow in self._control_flows],
            self._target_observation_name,
        )
        return self._binding

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start(self) -> None:
        if self._running:
            return
        self._log_video_encoder_self_check()
        self._stop_event.clear()
        self._mark_started()
        self._status_thread = threading.Thread(target=self._status_loop, daemon=True, name="teleop-status")
        self._image_thread = threading.Thread(target=self._image_loop, daemon=True, name="teleop-image")
        self._monitor_thread = threading.Thread(target=self._connection_monitor_loop, daemon=True, name="teleop-monitor")
        self._status_thread.start()
        self._image_thread.start()
        self._monitor_thread.start()
        self._start_web_ui()
        self._logger.info("Teleop app started")

    def close(self) -> None:
        self.stop()

    def stop(self) -> None:
        self.stop_teleop()
        self._stop_event.set()
        self._preview_demand_event.set()
        self.playback.shutdown()
        if self._web_ui is not None:
            self._web_ui.stop()
            self._web_ui = None
        for thread in (self._status_thread, self._image_thread, self._monitor_thread):
            if thread and thread.is_alive() and thread is not threading.current_thread():
                thread.join(timeout=2.0)
        self._status_thread = None
        self._image_thread = None
        self._monitor_thread = None
        self._close_clients()
        self._mark_stopped()
        self._logger.info("Teleop app stopped")

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "teleop_enabled": self._teleop_enabled,
                "frames_sent": self._frames_sent,
                "last_error": self._last_error,
                "metrics": self.metrics,
                "binding": None if self._binding is None else {
                    "source_robot_id": self._binding.source.robot_id,
                    "target_robot_id": self._binding.target.robot_id,
                    "embodiment_type": self._binding.source.embodiment_type,
                    "source_observation": self._teleop_observation_name,
                    "target_observation": self._target_observation_name,
                    "action": self._teleop_action_name,
                    "control_flows": self.control_flows,
                },
            }

    def run_web(self) -> None:
        self.start()
        while self._running:
            time.sleep(0.5)

    # ------------------------------------------------------------------
    # Public API consumed by Web UI
    # ------------------------------------------------------------------
    def start_teleop(self) -> None:
        with self._lock:
            if self._binding is None or self._source_client is None or self._target_client is None:
                raise RuntimeError("bind source and target before starting teleop")
            offline = [
                side
                for side in ("source", "target")
                if self.connection_status.get(side, {}).get("status") != "online"
            ]
            if offline:
                raise RuntimeError(f"server is not online: {', '.join(offline)}")
            if self._teleop_enabled:
                return
            self._wait_for_source_observations_unlocked()
            self._reset_realtime_metrics()
            self._last_sent_source_ts_by_flow.clear()
            self._teleop_enabled = True
            self._control_thread = threading.Thread(
                target=self._control_loop,
                daemon=True,
                name="teleop-control",
            )
            self._control_thread.start()
        self._logger.info("Teleop enabled")

    def stop_teleop(self) -> None:
        with self._lock:
            self._teleop_enabled = False
        self._cancel_source_observation_stream()
        thread = self._control_thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._control_thread = None
        self._reset_realtime_metrics()
        self._logger.info("Teleop disabled")

    def _wait_for_source_observations_unlocked(self, timeout_s: float = 2.0) -> None:
        source_client = self._source_client
        if source_client is None:
            raise RuntimeError("source server is not connected")
        source_names = sorted({flow.source_observation for flow in self._control_flows})
        if not source_names:
            raise RuntimeError("teleop control flows are not configured")
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        last_error = ""
        while True:
            response = source_client.get_observations(
                source_names,
                timeout_ms=self._request_timeout_ms(),
            )
            if response.ok:
                values = _observation_values(response)
                if all(name in values for name in source_names):
                    self._merge_local_observation(values)
                    return
            else:
                last_error = str(response.message or "No observation data available")
            if time.monotonic() >= deadline:
                detail = last_error or "No observation data available"
                raise RuntimeError(f"source observation is not ready: {detail}")
            self._stop_event.wait(0.05)

    def reconnect_servers(self) -> TeleopBinding:
        with self._lock:
            if self._binding is None:
                raise RuntimeError("bind source and target before reconnecting")
            source_robot_id = self._binding.source.robot_id
            target_robot_id = self._binding.target.robot_id
            self._teleop_enabled = False
        thread = self._control_thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._control_thread = None

        self._set_connection_status("source", "reconnecting", self._binding.source, message="")
        self._set_connection_status("target", "reconnecting", self._binding.target, message="")
        manifests = self.discover()
        source = _select_unique(manifests, source_robot_id)
        target = _select_unique(manifests, target_robot_id)
        raw_flows = [
            {
                "source_observation": flow.source_observation,
                "target_action": flow.target_action,
            }
            for flow in self._control_flows
        ]
        flows = _build_control_flows(source, target, raw_flows)
        target_observation = _select_target_state_observation(target, flows[0].source_type)
        client = self._client()
        source_client = _connect_manifest(source, client)
        target_client = _connect_manifest(target, client)
        with self._lock:
            self._close_clients()
            self._binding = TeleopBinding(source=source, target=target)
            self._source_client = source_client
            self._target_client = target_client
            self._control_flows = list(flows)
            self._teleop_observation_name = flows[0].source_observation
            self._teleop_observation_type = flows[0].source_type
            self._target_observation_name = str(target_observation["name"])
            self._target_observation_type = str(target_observation["type"])
            self._teleop_action_name = flows[0].target_action
            self._teleop_action_type = flows[0].target_type
            self._set_connection_status("source", "online", source, message="")
            self._set_connection_status("target", "online", target, message="")
            self._reconnect_candidates["source"] = None
            self._reconnect_candidates["target"] = None
            self._reset_protocol_key_cache()
            self._set_available_protocol_names()
            self._save_profile()
        self._logger.info("Teleop reconnected source=%s target=%s", source.robot_id, target.robot_id)
        return self._binding

    def set_preview_client_demand(self, enabled: bool) -> None:
        with self._preview_demand_lock:
            changed = self._peer_preview_requested != bool(enabled)
            self._peer_preview_requested = bool(enabled)
        if changed:
            self._preview_demand_event.set()

    def preview_key_config(self) -> dict[str, Any]:
        return {
            "options": {
                "target_images": self.target_image_items,
                "target_states": self.target_state_items,
                "source_states": self.source_state_items,
            },
            "selected": {
                "target_images": self._get_preview_image_keys(),
                "target_states": self._get_target_state_keys(),
                "source_states": self._get_source_state_keys(),
            },
        }

    def set_preview_key_config(
        self,
        *,
        target_images: Optional[list[str]] = None,
        target_states: Optional[list[str]] = None,
        source_states: Optional[list[str]] = None,
    ) -> dict[str, Any]:
        with self._preview_keys_lock:
            if target_images is not None:
                allowed = _image_observation_names(self._binding.target.observations if self._binding else [])
                self._preview_image_keys = _filter_names(_as_name_list(target_images), allowed)
            if target_states is not None:
                allowed = _state_observation_names(self._binding.target.observations if self._binding else [])
                self._target_state_keys = _filter_names(_as_name_list(target_states), allowed)
            if source_states is not None:
                allowed = _state_observation_names(self._binding.source.observations if self._binding else [])
                self._source_state_keys = _filter_names(_as_name_list(source_states), allowed)
        self._preview_demand_event.set()
        return self.preview_key_config()

    def start_record(self, keys: Optional[list[str]] = None, **kwargs: Any) -> bool:
        if self.connection_status.get("target", {}).get("status") != "online":
            raise RuntimeError("target server is not online")
        target_client = self._require_target_client()
        self._current_episode += 1
        if self._collection_config_locked:
            kwargs.update(self._collection_config)
        else:
            max_duration_s = float(kwargs.get("max_duration_s", self._record_max_duration_s))
            self._collection_config = {
                "task_prompt": kwargs.get("task_prompt", "teleop demo"),
                "task_description": kwargs.get("task_description", ""),
                "fps": float(kwargs.get("fps", 30)),
                "max_duration_s": max_duration_s,
            }
            kwargs.update(self._collection_config)
            self._collection_config_locked = True
        collection_id = str(kwargs.get("collection_id") or self._collection_id)
        self._collection_id = collection_id
        episode_id = str(kwargs.get("episode_id") or f"episode_{self._current_episode:06d}")
        selected_keys = keys or self.available_buffer_keys
        metadata = {
            "source_robot_id": self._binding.source.robot_id if self._binding else "",
            "target_robot_id": self._binding.target.robot_id if self._binding else "",
            "task_prompt": kwargs.get("task_prompt", "teleop demo"),
            "task_description": kwargs.get("task_description", ""),
        }
        response = target_client.start_collection(
            selected_keys,
            collection_id=collection_id,
            episode_id=episode_id,
            task_prompt=str(metadata["task_prompt"]),
            task_description=str(metadata["task_description"]),
            frame_rate=float(kwargs.get("fps", 30)),
            max_duration=float(kwargs.get("max_duration_s", self._record_max_duration_s)),
            metadata=metadata,
            timeout_ms=self._request_timeout_ms(),
        )
        if not response.ok:
            raise RuntimeError(response.message)
        collection = response.payload if isinstance(response.payload, dict) else {}
        with self._record_status_lock:
            self._latest_record_status.update({
                "recording": True,
                "frames_written": 0,
                "collection_id": collection.get("collection_id") or collection_id,
                "episode_id": collection.get("episode_id") or episode_id,
                "episode_dir": None,
                "collection_resource": collection.get("collection_resource") or {},
                "names": list(collection.get("names") or selected_keys),
                "episode_number": self._current_episode,
                "metadata": metadata,
            })
        return True

    def stop_record(self) -> dict[str, Any]:
        target_client = self._require_target_client()
        response = target_client.stop_collection(timeout_ms=self._request_timeout_ms())
        if not response.ok:
            raise RuntimeError(response.message)
        collection = response.payload if isinstance(response.payload, dict) else {}
        frames_written = sum(int(value) for value in dict(collection.get("per_name_counts") or {}).values())
        resource = collection.get("collection_resource") if isinstance(collection.get("collection_resource"), dict) else {}
        local_path = (resource.get("metadata") or {}).get("local_path") if isinstance(resource.get("metadata"), dict) else ""
        if local_path:
            self.records.add_collection_dir(str(local_path))
        with self._record_status_lock:
            self._latest_record_status.update(collection)
            self._latest_record_status["frames_written"] = frames_written
            self._latest_record_status["recording"] = False
            self._latest_record_status["episode_number"] = self._current_episode
            self._latest_record_status["episode_dir"] = None
        return self.record_status

    def discard_record(self, episode_number: Optional[int] = None) -> dict[str, Any]:
        with self._record_status_lock:
            path = self._latest_record_status.get("episode_dir")
            resource = self._latest_record_status.get("collection_resource")
            resource = dict(resource) if isinstance(resource, dict) else {}
        resource_id = str(resource.get("resource_id") or "")
        result = (
            self.delete_server_collection("target", resource_id)
            if resource_id
            else self.records.delete_episode(str(path)) if path
            else {"success": False, "message": "episode resource not found"}
        )
        if result.get("success"):
            with self._record_status_lock:
                self._latest_record_status["collection_resource"] = {}
                self._latest_record_status["episode_dir"] = None
        return result

    def delete_episode_ref(self, episode_ref: str) -> dict[str, Any]:
        value = str(episode_ref or "")
        if value.startswith(_RESOURCE_EPISODE_PREFIX):
            side, resource_id = _parse_resource_episode_ref(value)
            return self.delete_server_collection(side, resource_id)
        return self.records.delete_episode(value)

    def export_data(
        self,
        collection_id: Optional[str] = None,
        fps: Optional[float] = None,
        export_rynnbot_mapping: Optional[bool] = None,
        progress_callback: Optional[Callable[[float, float, str], None]] = None,
    ) -> dict[str, Any]:
        self._add_bound_server_record_roots()
        cid = collection_id or self._collection_id
        kwargs = {"_progress_callback": progress_callback} if progress_callback else {}
        if fps is not None:
            kwargs["fps"] = fps
        if export_rynnbot_mapping is not None:
            kwargs["export_rynnbot_mapping"] = export_rynnbot_mapping
        result = self.records.export_data_collection(cid, **kwargs)
        result["collection_id"] = cid
        if result.get("success"):
            self._export_result = {
                "success": True,
                "message": result.get("message", "export complete"),
                "zip_path": result.get("zip_path") or (result.get("result") or {}).get("zip_path"),
                "exported_count": len((result.get("result") or {}).get("exported") or result.get("encoded") or []),
                "exported_episode_paths": result.get("episode_paths") or [],
            }
        return result

    def start_playback(self, episode_path: str, request_id: str = "") -> bool:
        if self.connection_status.get("target", {}).get("status") != "online":
            raise RuntimeError("target server is not online")
        episode_path, cleanup_path = self._prepare_playback_episode_path(episode_path)
        self.playback.start(episode_path, cleanup_path=cleanup_path)
        return True

    def export_episode_paths(
        self,
        episode_paths: list[str],
        zip_name: str | None = None,
        **kwargs: Any,
    ) -> dict[str, Any]:
        temp_dirs: list[str] = []
        progress_callback = kwargs.get("_progress_callback")

        def emit_progress(progress: float, message: str) -> None:
            if progress_callback:
                progress_callback(progress, 100.0, message)

        def encode_progress(current: float, total: float, message: str) -> None:
            total_value = float(total or 0)
            ratio = 1.0 if total_value <= 0 else min(1.0, max(0.0, float(current) / total_value))
            if "packaging zip" in str(message):
                emit_progress(95.0, "正在打包 zip 文件...")
                return
            emit_progress(15.0 + ratio * 80.0, _export_progress_message(str(message)))

        try:
            emit_progress(0.0, "正在准备导出任务...")
            total = len(episode_paths)
            local_paths: list[str] = []
            for index, path in enumerate(episode_paths, start=1):
                local_paths.append(
                    self._ensure_export_episode_path(
                        str(path),
                        temp_dirs=temp_dirs,
                        progress_callback=progress_callback,
                        index=index,
                        total=total,
                    )
                )
            export_kwargs = dict(kwargs)
            if progress_callback:
                export_kwargs["_progress_callback"] = encode_progress
            result = self.records.export_episodes(local_paths, zip_name, **export_kwargs)
            if result.get("success"):
                emit_progress(100.0, "导出完成")
            result["episode_paths"] = list(episode_paths)
            result["exported_episode_paths"] = list(episode_paths)
            if temp_dirs and result.get("success"):
                result["server_source_cleanup_available"] = True
                result["message"] = str(result.get("message") or "") + "; server source files can be deleted after export"
            return result
        finally:
            for path in temp_dirs:
                shutil.rmtree(path, ignore_errors=True)

    def stop_playback(self, request_id: str = "") -> bool:
        self.playback.stop()
        return True

    def get_playback_status(self) -> Optional[dict[str, Any]]:
        return self.playback.status

    def get_debug_status(self, include_peer: bool = True, timeout_s: float = 1.0) -> dict[str, Any]:
        return {
            "local": {
                "role": "teleop_app",
                "teleop_enabled": self._teleop_enabled,
                "peer_connected": self.is_peer_connected,
                "metrics": self.metrics,
                "record_status": self.record_status,
                "available_buffer_keys": self.available_buffer_keys,
                "available_buffer_items": self.available_buffer_items,
                "latest_local_state_keys": sorted((self.latest_leader_state or {}).keys()),
                "latest_peer_state_keys": sorted((self.latest_follower_state or {}).keys()),
                "latest_peer_image_keys": sorted((self.latest_follower_images or {}).keys()),
            },
            "peer": None,
            "peer_error": "debug over Interface is exposed through server get_runtime_status",
        }

    def scan_records(self) -> dict[str, Any]:
        self._add_bound_server_record_roots()
        data = self.records.scan_local_records()
        source_location = self._scan_server_resources("source")
        target_location = self._scan_server_resources("target")
        app_location = {
            "key": "app",
            "label": "App 本机",
            "location_type": "app",
            "side": "app",
            "online": True,
            "robot_id": "",
            "robot_name": "",
            "endpoint": "local filesystem",
            "data_collections": list(data.get("data_collections", [])),
            "exported_zips": data.get("exported_zips", []),
            "logs": [],
            "resources": [],
            "message": "",
            "total_size": data.get("total_size", 0),
            "total_size_formatted": data.get("total_size_formatted", "0 B"),
            "export_dir": data.get("export_dir", ""),
        }
        locations = [app_location]
        merged_labels: list[str] = []
        merged_total_size = int(app_location.get("total_size") or 0)
        for location in (source_location, target_location):
            if _server_location_is_local(location):
                app_location["data_collections"] = _merge_data_collections(
                    app_location["data_collections"],
                    location.get("data_collections") or [],
                )
                app_location["logs"].extend(location.get("logs") or [])
                app_location["resources"].extend(location.get("resources") or [])
                merged_total_size += int(location.get("total_size") or 0)
                if (location.get("data_collections") or location.get("logs") or location.get("resources")):
                    merged_labels.append(str(location.get("label") or location.get("side") or "server"))
            else:
                locations.append(location)
        app_location["total_size"] = merged_total_size
        app_location["total_size_formatted"] = format_bytes(merged_total_size)
        if merged_labels:
            app_location["message"] = "已合并本机 " + "、".join(merged_labels)
        data["local"] = app_location
        data["source_server"] = source_location
        data["target_server"] = target_location
        data["server"] = target_location
        data["locations"] = locations
        return data

    def delete_server_collection(self, side: str, resource_id: str) -> dict[str, Any]:
        client = self._require_server_client(side)
        response = client.delete_collection(resource_id, timeout_ms=self._request_timeout_ms())
        if not response.ok:
            return {"success": False, "message": response.message}
        payload = response.payload if isinstance(response.payload, dict) else {}
        return {"success": bool(payload.get("deleted")), "message": response.message, "resource_id": resource_id, "side": side}

    def read_server_resource(self, side: str, resource_id: str, *, max_bytes: int = 8 * 1024 * 1024) -> bytes:
        client = self._require_server_client(side)
        chunks: list[bytes] = []
        offset = 0
        while offset < max_bytes:
            response = client.read_resource(
                resource_id,
                offset=offset,
                limit=min(64 * 1024, max_bytes - offset),
                timeout_ms=self._request_timeout_ms(),
            )
            if not response.ok:
                raise RuntimeError(response.message or "read_resource failed")
            payload = response.payload if isinstance(response.payload, dict) else {}
            data = base64.b64decode(str(payload.get("data") or ""))
            chunks.append(data)
            offset = int(payload.get("next_offset") or offset + len(data))
            if payload.get("eof"):
                break
            if not data:
                raise RuntimeError("read_resource returned an empty non-eof chunk")
        return b"".join(chunks)

    def delete_server_log_resource(self, side: str, resource_id: str) -> dict[str, Any]:
        client = self._require_server_client(side)
        info = client.get_resource_info(resource_id, timeout_ms=self._request_timeout_ms())
        if not info.ok:
            return {"success": False, "message": info.message or "get_resource_info failed"}
        resource = (info.payload or {}).get("resource") if isinstance(info.payload, dict) else {}
        if not isinstance(resource, dict) or resource.get("domain") != "log":
            return {"success": False, "message": "resource is not a log"}
        response = client.delete_resource(resource_id, timeout_ms=self._request_timeout_ms())
        if not response.ok:
            return {"success": False, "message": response.message or "delete_resource failed"}
        payload = response.payload if isinstance(response.payload, dict) else {}
        return {"success": bool(payload.get("deleted")), "message": response.message, "resource_id": resource_id, "side": side}

    def _scan_server_resources(self, side: str) -> dict[str, Any]:
        client = self._server_client_for_side(side)
        status = self.connection_status.get(side, {})
        label = "控制端 Server" if side == "source" else "被控端 Server"
        base = {
            "key": side,
            "label": label,
            "location_type": "server",
            "side": side,
            "online": status.get("status") == "online",
            "robot_id": status.get("robot_id") or "",
            "robot_name": status.get("robot_name") or "",
            "endpoint": status.get("endpoint") or "",
            "endpoint_source": status.get("endpoint_source") or "",
            "data_collections": [],
            "resources": [],
            "logs": [],
            "exported_zips": [],
            "total_size": 0,
            "total_size_formatted": "0 B",
            "message": "",
        }
        if client is None or status.get("status") != "online":
            base["message"] = f"{label} offline"
            return base
        try:
            collections = [_tag_server_resource(item, side) for item in self._list_server_resources(client, domain="data", kind="collection")]
            logs = [_tag_server_resource(item, side) for item in self._list_server_resources(client, domain="log")]
            total_size = sum(int(item.get("size_bytes") or 0) for item in [*collections, *logs])
            base.update({
                "data_collections": _server_collections_from_resources(collections, side=side),
                "resources": [*collections, *logs],
                "logs": logs,
                "total_size": total_size,
                "total_size_formatted": format_bytes(total_size),
                "message": "OK",
            })
            return base
        except Exception as exc:
            base["message"] = str(exc)
            return base

    def _server_client_for_side(self, side: str) -> RcpProtocolClient | None:
        if side == "source":
            return self._source_client
        if side == "target":
            return self._target_client
        raise ValueError("side must be source or target")

    def _add_bound_server_record_roots(self) -> None:
        if self._binding is None:
            return
        for manifest in (self._binding.source, self._binding.target):
            self.records.add_record_root(str(collections_dir(robot_root(manifest.robot_id), "manual")))

    def _require_server_client(self, side: str) -> RcpProtocolClient:
        client = self._server_client_for_side(side)
        if client is None:
            raise RuntimeError(f"{side} server is not connected")
        return client

    def _list_server_resources(
        self,
        client: RcpProtocolClient,
        *,
        domain: str | None = None,
        kind: str | None = None,
    ) -> list[dict[str, Any]]:
        resources: list[dict[str, Any]] = []
        cursor: str | None = None
        while True:
            response = client.list_resources(
                domain=domain,
                kind=kind,
                cursor=cursor,
                limit=500,
                timeout_ms=self._request_timeout_ms(),
            )
            if not response.ok:
                raise RuntimeError(response.message or "list_resources failed")
            payload = response.payload if isinstance(response.payload, dict) else {}
            resources.extend(item for item in payload.get("resources", []) if isinstance(item, dict))
            cursor = payload.get("next_cursor")
            if not cursor:
                break
        return resources

    # ------------------------------------------------------------------
    # Properties used by Web UI
    # ------------------------------------------------------------------
    @property
    def playback_in_progress(self) -> bool:
        return self.playback.in_progress

    @playback_in_progress.setter
    def playback_in_progress(self, value: bool) -> None:
        self.playback.in_progress = bool(value)

    @property
    def is_peer_connected(self) -> bool:
        return bool(self._binding and self._last_heartbeat_recv > 0 and time.time() - self._last_heartbeat_recv < _HEARTBEAT_TIMEOUT)

    @property
    def connection_status(self) -> dict[str, dict[str, Any]]:
        with self._connection_status_lock:
            return {side: dict(status) for side, status in self._connection_status.items()}

    @property
    def teleop_enabled(self) -> bool:
        return self._teleop_enabled

    @property
    def latest_follower_state(self) -> Optional[dict[str, Any]]:
        with self._state_lock:
            return dict(self._latest_peer_state) if self._latest_peer_state is not None else None

    @property
    def latest_follower_state_seq(self) -> int:
        return self._latest_peer_state_seq

    @property
    def latest_follower_state_at(self) -> float:
        return self._latest_peer_state_at

    @property
    def latest_leader_state(self) -> Optional[dict[str, Any]]:
        with self._state_lock:
            return dict(self._latest_local_state) if self._latest_local_state is not None else None

    @property
    def latest_leader_state_seq(self) -> int:
        return self._latest_local_state_seq

    @property
    def latest_follower_images(self) -> Optional[dict[str, bytes]]:
        with self._images_lock:
            return dict(self._latest_peer_images) if self._latest_peer_images is not None else None

    @property
    def latest_follower_images_seq(self) -> int:
        return self._latest_peer_images_seq

    @property
    def latest_follower_images_at(self) -> float:
        return self._latest_peer_images_at

    @property
    def record_status(self) -> dict[str, Any]:
        with self._record_status_lock:
            return dict(self._latest_record_status)

    @property
    def available_buffer_keys(self) -> list[str]:
        with self._available_buffer_keys_lock:
            return list(self._available_buffer_keys)

    @property
    def available_buffer_items(self) -> list[dict[str, Any]]:
        with self._available_buffer_keys_lock:
            keys = list(self._available_buffer_keys)
        descriptors: dict[str, dict[str, Any]] = {}
        if self._binding is not None:
            for category, items in (
                ("observation", self._binding.target.observations),
                ("action", self._binding.target.actions),
            ):
                for item in items:
                    if isinstance(item, dict) and item.get("name"):
                        descriptor = dict(item)
                        descriptor.setdefault("category", category)
                        descriptors[str(item["name"])] = descriptor
        return [_record_item_descriptor(key, descriptors.get(key)) for key in keys]

    @property
    def source_state_items(self) -> list[dict[str, Any]]:
        if self._binding is None:
            return []
        return [
            _record_item_descriptor(str(item["name"]), {**dict(item), "category": "observation"})
            for item in self._binding.source.observations
            if item.get("name") and item.get("type") != "image"
        ]

    @property
    def target_state_items(self) -> list[dict[str, Any]]:
        if self._binding is None:
            return []
        return [
            _record_item_descriptor(str(item["name"]), {**dict(item), "category": "observation"})
            for item in self._binding.target.observations
            if item.get("name") and item.get("type") != "image"
        ]

    @property
    def target_image_items(self) -> list[dict[str, Any]]:
        if self._binding is None:
            return []
        return [
            _record_item_descriptor(str(item["name"]), {**dict(item), "category": "observation"})
            for item in self._binding.target.observations
            if item.get("name") and item.get("type") == "image"
        ]

    @property
    def teleop_observation_name(self) -> str:
        return self._teleop_observation_name

    @property
    def target_observation_name(self) -> str:
        return self._target_observation_name

    @property
    def teleop_action_name(self) -> str:
        return self._teleop_action_name

    @property
    def control_flows(self) -> list[dict[str, str]]:
        return [
            {
                "source_observation": flow.source_observation,
                "source_type": flow.source_type,
                "target_action": flow.target_action,
                "target_type": flow.target_type,
            }
            for flow in self._control_flows
        ]

    @property
    def metrics(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._metrics_lock:
            for key in self._metric_events:
                self._trim_metric_events_unlocked(key, now)
            latency = list(self._action_latency_window)
            action_rpc = list(self._action_rpc_window)
            obs_rpc = list(self._observation_rpc_window)
            source_state_rpc = list(self._source_state_rpc_window)
            target_state_rpc = list(self._target_state_rpc_window)
            image_rpc = list(self._image_rpc_window)
            return {
                "role": "leader",
                "transport": "grpc",
                "command_tx_fps": self._fps_unlocked("command_tx", now),
                "command_rx_fps": self._fps_unlocked("command_rx", now),
                "image_tx_fps": self._fps_unlocked("image_tx", now),
                "image_rx_fps": self._fps_unlocked("image_rx", now),
                "command_target_fps": self._control_hz,
                "state_target_fps": self._state_hz,
                "status_target_fps": self._status_hz,
                "preview_target_fps": self._image_hz,
                "action_latency_last_ms": latency[-1] if latency else 0.0,
                "action_latency_avg_ms": statistics.fmean(latency) if latency else 0.0,
                "action_latency_p95_ms": _percentile(latency, 0.95),
                "action_latency_max_ms": max(latency) if latency else 0.0,
                "action_rpc_last_ms": action_rpc[-1] if action_rpc else 0.0,
                "action_rpc_avg_ms": statistics.fmean(action_rpc) if action_rpc else 0.0,
                "action_rpc_p95_ms": _percentile(action_rpc, 0.95),
                "observation_rpc_last_ms": obs_rpc[-1] if obs_rpc else 0.0,
                "observation_rpc_avg_ms": statistics.fmean(obs_rpc) if obs_rpc else 0.0,
                "observation_rpc_p95_ms": _percentile(obs_rpc, 0.95),
                "source_state_rpc_last_ms": source_state_rpc[-1] if source_state_rpc else 0.0,
                "source_state_rpc_avg_ms": statistics.fmean(source_state_rpc) if source_state_rpc else 0.0,
                "target_state_rpc_last_ms": target_state_rpc[-1] if target_state_rpc else 0.0,
                "target_state_rpc_avg_ms": statistics.fmean(target_state_rpc) if target_state_rpc else 0.0,
                "image_rpc_last_ms": image_rpc[-1] if image_rpc else 0.0,
                "image_rpc_avg_ms": statistics.fmean(image_rpc) if image_rpc else 0.0,
                "last_error": self._last_error,
                **dict(self._metric_bytes),
            }

    # ------------------------------------------------------------------
    # Loops
    # ------------------------------------------------------------------
    def _control_loop(self) -> None:
        while self._teleop_enabled and not self._stop_event.is_set():
            try:
                source_client = self._source_client
                target_client = self._target_client
                if source_client is None or target_client is None:
                    raise RuntimeError("teleop source/target are not connected")
                flows = list(self._control_flows)
                if not flows:
                    raise RuntimeError("teleop control flows are not configured")
                source_names = sorted({flow.source_observation for flow in flows})
                stream = source_client.subscribe_observations(
                    source_names,
                    stream_hz=self._control_hz,
                    timeout_ms=None,
                )
                self._source_observation_stream = stream
                while self._teleop_enabled and not self._stop_event.is_set():
                    started = time.perf_counter()
                    obs_start = time.perf_counter()
                    source_obs = next(stream)
                    self._record_observation_rpc((time.perf_counter() - obs_start) * 1000.0)
                    source_items = _observation_items(source_obs)
                    source_state = {
                        name: item.get("value")
                        for name, item in source_items.items()
                    }
                    self._merge_local_observation(source_state)

                    action_start = time.perf_counter()
                    command_bytes = 0
                    sent_any = False
                    for flow in flows:
                        source_item = source_items.get(flow.source_observation)
                        action = None if source_item is None else source_item.get("value")
                        if action is None:
                            raise RuntimeError(f"source observation missing {flow.source_observation}")
                        source_ts = source_item.get("timestamp") if isinstance(source_item, dict) else None
                        flow_key = (flow.source_observation, flow.target_action)
                        if isinstance(source_ts, (int, float)) and self._last_sent_source_ts_by_flow.get(flow_key) == float(source_ts):
                            continue
                        result = target_client.run_action_chunk(
                            flow.target_action,
                            [action],
                            frame_rate=self._control_hz,
                            timeout_ms=self._request_timeout_ms(),
                        )
                        if not result.ok:
                            raise RuntimeError(result.message)
                        if isinstance(source_ts, (int, float)):
                            self._last_sent_source_ts_by_flow[flow_key] = float(source_ts)
                        command_bytes += payload_size(action)
                        sent_any = True
                    action_rpc_ms = (time.perf_counter() - action_start) * 1000.0
                    if sent_any:
                        self._record_action_rpc(action_rpc_ms)
                    latency_ms = (time.perf_counter() - started) * 1000.0
                    self._record_action_latency(latency_ms)
                    if sent_any:
                        self._mark_metric("command_tx", command_bytes)
                        self._frames_sent += 1
                    self._last_heartbeat_recv = time.time()
                    self._last_error = ""
                    self._clear_loop_error("control")
            except Exception as exc:
                if not self._teleop_enabled or self._stop_event.is_set():
                    break
                self._last_error = str(exc)
                self._log_loop_error("control", "Teleop control tick failed", exc)
                self._stop_event.wait(0.05)
            finally:
                self._cancel_source_observation_stream()

    def _status_loop(self) -> None:
        state_interval = 1.0 / max(1.0, self._state_hz)
        active_status_interval = 1.0 / max(1.0, self._status_hz)
        idle_status_interval = 2.0
        last_status_refresh = 0.0
        while not self._stop_event.is_set():
            started = time.monotonic()
            try:
                source_client = self._source_client
                if source_client is not None and not self._teleop_enabled:
                    source_keys = self._get_source_state_keys()
                    if source_keys:
                        try:
                            rpc_start = time.perf_counter()
                            source_obs = source_client.get_observations(
                                source_keys,
                                timeout_ms=self._request_timeout_ms(),
                            )
                            self._record_source_state_rpc((time.perf_counter() - rpc_start) * 1000.0)
                            source_state = _observation_values(source_obs)
                            if source_state:
                                self._merge_local_observation(source_state)
                                self._clear_loop_error("source_status")
                        except Exception as exc:
                            self._last_error = f"source state read failed: {exc}"
                            self._log_loop_error("source_status", "Teleop source state read failed", exc)

                target_client = self._target_client
                if target_client is not None:
                    target_keys = self._get_target_state_keys()
                    if target_keys:
                        try:
                            rpc_start = time.perf_counter()
                            obs = target_client.get_observations(
                                target_keys,
                                timeout_ms=self._request_timeout_ms(),
                            )
                            self._record_target_state_rpc((time.perf_counter() - rpc_start) * 1000.0)
                            state = _observation_values(obs)
                            if state:
                                self._handle_target_state(state)
                                self._clear_loop_error("target_status")
                        except Exception as exc:
                            self._last_error = f"target state read failed: {exc}"
                            self._log_loop_error("target_status", "Teleop target state read failed", exc)

                now = time.monotonic()
                status_interval = active_status_interval if self._is_recording_cached() else idle_status_interval
                if target_client is not None and now - last_status_refresh >= status_interval:
                    self._refresh_collection_status()
                    last_status_refresh = now
            except Exception as exc:
                self._last_error = f"status loop failed: {exc}"
                self._log_loop_error("status", "Teleop status loop failed", exc)
            self._stop_event.wait(max(0.0, state_interval - (time.monotonic() - started)))

    def _is_recording_cached(self) -> bool:
        with self._record_status_lock:
            return bool(self._latest_record_status.get("recording"))

    def _image_loop(self) -> None:
        interval = 1.0 / max(1.0, self._image_hz)
        while not self._stop_event.is_set():
            if not self._should_send_preview_images():
                self._preview_demand_event.clear()
                self._preview_demand_event.wait(timeout=1.0)
                continue
            started = time.monotonic()
            try:
                target_client = self._target_client
                if target_client is not None:
                    keys = self._get_preview_image_keys()
                    if keys:
                        rpc_start = time.perf_counter()
                        obs = target_client.get_observations(keys, timeout_ms=self._request_timeout_ms())
                        self._record_image_rpc((time.perf_counter() - rpc_start) * 1000.0)
                        state = _observation_values(obs)
                        self._handle_target_images(state)
            except Exception as exc:
                self._logger.debug("Teleop target image read failed: %s", exc)
            self._stop_event.wait(max(0.0, interval - (time.monotonic() - started)))

    def _connection_monitor_loop(self) -> None:
        next_reconnect_scan = 0.0
        while not self._stop_event.is_set():
            for side in ("source", "target"):
                self._check_bound_server(side)
            now = time.monotonic()
            if now >= next_reconnect_scan:
                self._scan_reconnect_candidates()
                next_reconnect_scan = now + max(0.5, self._connection_reconnect_scan_s)
            self._stop_event.wait(max(0.1, self._connection_monitor_interval_s))

    def _check_bound_server(self, side: str) -> None:
        with self._lock:
            binding = self._binding
            client = self._source_client if side == "source" else self._target_client
            manifest = None if binding is None else (binding.source if side == "source" else binding.target)
        if binding is None or manifest is None:
            self._set_connection_status(side, "unbound")
            return
        if client is None:
            return
        try:
            response = client.ping(timeout_ms=self._connection_monitor_timeout_ms)
            if not response.ok:
                raise RuntimeError(response.message or "ping failed")
            if self.connection_status.get(side, {}).get("status") != "online":
                self._set_connection_status(side, "online", manifest, message="")
        except Exception as exc:
            self._handle_server_offline(side, manifest, str(exc))

    def _scan_reconnect_candidates(self) -> None:
        with self._lock:
            binding = self._binding
        if binding is None:
            return
        statuses = self.connection_status
        wanted = {
            "source": binding.source.robot_id,
            "target": binding.target.robot_id,
        }
        if all(statuses.get(side, {}).get("status") == "online" for side in wanted):
            return
        try:
            manifests = self.discover()
        except Exception as exc:
            self._logger.debug("Teleop reconnect discovery failed: %s", exc)
            return
        by_robot: dict[str, list[ServerManifest]] = {}
        for manifest in manifests:
            by_robot.setdefault(manifest.robot_id, []).append(manifest)
        for side, robot_id in wanted.items():
            current = statuses.get(side, {}).get("status")
            if current == "online":
                continue
            matches = by_robot.get(robot_id) or []
            if not matches:
                if current == "reconnectable":
                    manifest = binding.source if side == "source" else binding.target
                    self._reconnect_candidates[side] = None
                    self._set_connection_status(side, "offline", manifest, message="server is no longer discoverable")
                continue
            candidate = _preferred_manifest(matches)
            current_endpoint = statuses.get(side, {}).get("endpoint")
            candidate_endpoint = candidate.endpoint.address if candidate.endpoint is not None else ""
            if current == "reconnectable" and current_endpoint == candidate_endpoint:
                continue
            self._reconnect_candidates[side] = candidate
            self._set_connection_status(side, "reconnectable", candidate, message="server is available; confirm reconnect")

    def _handle_server_offline(self, side: str, manifest: ServerManifest, message: str) -> None:
        current = self.connection_status.get(side, {}).get("status")
        if current in ("offline", "reconnectable", "reconnecting"):
            return
        self._logger.warning("Teleop %s server offline: %s", side, message)
        self._close_side_client(side)
        self.stop_teleop()
        if side == "target" and self.playback.in_progress:
            self.playback.set_remote_status({
                "status": "error",
                "message": f"{side} server offline: {message}",
            })
        if side == "target":
            with self._record_status_lock:
                if self._latest_record_status.get("recording"):
                    self._latest_record_status["recording"] = False
                    self._latest_record_status["last_error"] = f"target server offline: {message}"
        self._set_connection_status(side, "offline", manifest, message=message)

    def _close_side_client(self, side: str) -> None:
        with self._lock:
            client = self._source_client if side == "source" else self._target_client
            if side == "source":
                self._source_client = None
            else:
                self._target_client = None
        if client is not None:
            client.close()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _start_web_ui(self) -> None:
        if self._web_ui is not None:
            return
        self._web_ui = TeleopWebUI(
            plugin=self,
            host=self._host,
            port=self._port,
            open_browser=self._open_browser,
            quit_callback=self.stop,
        )
        self._web_ui.start()

    def _client(self) -> ClientInterface:
        discovery = self.config.get("discovery") if isinstance(self.config.get("discovery"), dict) else {}
        static = discovery.get("static_endpoints") or []
        if isinstance(static, str):
            static = [static]
        return ClientInterface.with_defaults(
            static_addresses=[str(item) for item in static],
            local_registry=bool(discovery.get("local", True)),
            mdns=bool(discovery.get("mdns", True)),
        )

    def _set_connection_status(
        self,
        side: str,
        status: str,
        manifest: ServerManifest | None = None,
        *,
        message: str = "",
    ) -> None:
        now = time.time()
        with self._connection_status_lock:
            previous = self._connection_status.get(side, {})
            payload = dict(previous)
            payload.update({
                "side": side,
                "status": status,
                "message": message,
                "updated_at": now,
                "robot_id": manifest.robot_id if manifest is not None else previous.get("robot_id", ""),
                "robot_name": manifest.robot_name if manifest is not None else previous.get("robot_name", ""),
                "endpoint": manifest.endpoint.address if manifest is not None and manifest.endpoint is not None else previous.get("endpoint", ""),
                "endpoint_source": manifest.endpoint.source if manifest is not None and manifest.endpoint is not None else previous.get("endpoint_source", ""),
            })
            if status == "online":
                payload["last_online_at"] = now
            if status == "offline":
                payload["last_offline_at"] = now
            self._connection_status[side] = payload

    def _discovery_timeout_s(self) -> float:
        discovery = self.config.get("discovery") if isinstance(self.config.get("discovery"), dict) else {}
        return float(discovery.get("timeout_s") or 1.5)

    def _request_timeout_ms(self) -> int:
        app_cfg = self.config.get("app") if isinstance(self.config.get("app"), dict) else {}
        return int(app_cfg.get("request_timeout_ms") or self.config.get("request_timeout_ms") or 1000)

    def _save_profile(self) -> None:
        if self._profile_path is None or self._binding is None:
            return
        self._profile_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "source_robot_id": self._binding.source.robot_id,
            "target_robot_id": self._binding.target.robot_id,
            "embodiment_type": self._binding.source.embodiment_type,
            "source_observation": self._teleop_observation_name,
            "target_observation": self._target_observation_name,
            "action": self._teleop_action_name,
            "control_flows": self.control_flows,
        }
        self._profile_path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")

    def _close_clients(self) -> None:
        self._cancel_source_observation_stream()
        for client in (self._source_client, self._target_client):
            if client is not None:
                client.close()
        self._source_client = None
        self._target_client = None

    def _cancel_source_observation_stream(self) -> None:
        stream = self._source_observation_stream
        self._source_observation_stream = None
        cancel = getattr(stream, "cancel", None)
        if cancel is not None:
            cancel()

    def _require_target_client(self) -> RcpProtocolClient:
        if self._target_client is None:
            raise RuntimeError("target server is not bound")
        return self._target_client

    def _merge_local_observation(self, state: dict[str, Any]) -> None:
        with self._state_lock:
            merged = dict(self._latest_local_state or {})
            merged.update(state)
            self._latest_local_state = merged
            self._latest_local_state_seq += 1
        self._update_available_keys(state)

    def _handle_target_state(self, state: dict[str, Any]) -> None:
        with self._state_lock:
            merged = dict(self._latest_peer_state or {})
            merged.update(state)
            self._latest_peer_state = merged
            self._latest_peer_state_seq += 1
            self._latest_peer_state_at = time.time()
        self._last_heartbeat_recv = time.time()
        self._mark_metric("command_rx", payload_size(state))
        self._update_available_keys(state)

    def _handle_target_images(self, state: dict[str, Any]) -> None:
        images: dict[str, bytes] = {}
        max_long_edge = self._preview_resize_long_edge()
        image_names = set(self._get_preview_image_keys())
        for key, value in state.items():
            if str(key) not in image_names or not isinstance(value, dict):
                continue
            raw = value.get("image")
            raw_bytes = _as_bytes(raw)
            if raw_bytes is None:
                continue
            images[str(key)] = (
                resize_jpeg_long_edge(raw_bytes, max_long_edge)
                if max_long_edge > 0
                else raw_bytes
            )
        if not images:
            return
        with self._images_lock:
            self._latest_peer_images = images
            self._latest_peer_images_seq += 1
            self._latest_peer_images_at = time.time()
        self._last_heartbeat_recv = time.time()
        self._mark_metric("image_rx", payload_size(images))
        self._update_available_keys(state)

    def _update_available_keys(self, state: dict[str, Any]) -> None:
        with self._available_buffer_keys_lock:
            keys = set(self._available_buffer_keys)
            keys.update(str(key) for key in state.keys() if not str(key).startswith("_"))
            self._available_buffer_keys = sorted(keys)

    def _set_available_protocol_names(self) -> None:
        if self._binding is None:
            return
        names = [
            str(item["name"])
            for item in [*self._binding.target.observations, *self._binding.target.actions]
            if item.get("name")
        ]
        with self._available_buffer_keys_lock:
            self._available_buffer_keys = sorted(set(names))

    def _reset_protocol_key_cache(self) -> None:
        with self._preview_keys_lock:
            self._preview_image_keys = None
            self._source_state_keys = None
            self._target_state_keys = None

    def _refresh_collection_status(self) -> None:
        target_client = self._target_client
        if target_client is None:
            return
        try:
            response = target_client.get_collection_status(timeout_ms=self._request_timeout_ms())
            if response.ok and isinstance(response.payload, dict):
                with self._record_status_lock:
                    self._latest_record_status.update(response.payload)
                    if "running" in response.payload:
                        self._latest_record_status["recording"] = bool(response.payload.get("running"))
            self._latest_record_status.setdefault("episode_number", self._current_episode)
        except Exception:
            return

    def _download_collection_from_client(
        self,
        client: RcpProtocolClient,
        collection: dict[str, Any],
        destination_root: str,
    ) -> str:
        return download_collection_resource(
            client,
            collection,
            destination_root,
            timeout_ms=max(self._request_timeout_ms(), 30000),
        )

    def _prepare_playback_episode_path(self, episode_ref: str) -> tuple[str, str | None]:
        value = str(episode_ref or "")
        if not value.startswith(_RESOURCE_EPISODE_PREFIX):
            return value, None
        side, resource_id = _parse_resource_episode_ref(value)
        client = self._require_server_client(side)
        collection = self._collection_from_resource(client, resource_id)
        action_name = self._teleop_action_name
        if not action_name:
            raise RuntimeError("target action name is not selected")
        temp_root = self._make_temp_record_root("playback_")
        try:
            episode_dir = download_collection_entries(
                client,
                collection,
                temp_root,
                [
                    "collection_meta.json",
                    f"streams/{safe_name(action_name)}/samples.msgpack",
                ],
                timeout_ms=max(self._request_timeout_ms(), 30000),
            )
            self.records.add_collection_dir(episode_dir)
            return episode_dir, temp_root
        except Exception:
            shutil.rmtree(temp_root, ignore_errors=True)
            raise

    def _ensure_export_episode_path(
        self,
        episode_ref: str,
        *,
        temp_dirs: list[str],
        progress_callback: Optional[Callable[[float, float, str], None]] = None,
        index: int = 1,
        total: int = 1,
    ) -> str:
        value = str(episode_ref or "")
        if not value.startswith(_RESOURCE_EPISODE_PREFIX):
            return value
        side, resource_id = _parse_resource_episode_ref(value)
        client = self._require_server_client(side)
        collection = self._collection_from_resource(client, resource_id)
        local_path = _local_collection_path(collection) if self._server_side_is_local(side) else ""
        if local_path:
            if progress_callback:
                progress_callback(
                    min(14.0, (float(index - 1) / max(float(total), 1.0)) * 15.0),
                    100.0,
                    f"本机 server episode {index}/{total} 直接编码，无需复制原始数据...",
                )
            self.records.add_collection_dir(local_path)
            return local_path
        temp_root = self._make_temp_record_root("export_")
        temp_dirs.append(temp_root)
        if progress_callback:
            progress_callback(
                min(14.0, (float(index - 1) / max(float(total), 1.0)) * 15.0),
                100.0,
                f"正在从 {side} server 复制 episode {index}/{total} 到 app 临时目录...",
            )
        episode_dir = self._download_collection_from_client(client, collection, temp_root)
        self.records.add_collection_dir(episode_dir)
        return episode_dir

    def _server_side_is_local(self, side: str) -> bool:
        status = self.connection_status.get(side, {})
        return _server_location_is_local(
            {
                "online": status.get("status") == "online",
                "endpoint": status.get("endpoint") or "",
                "endpoint_source": status.get("endpoint_source") or "",
            }
        )

    def _collection_from_resource(self, client: RcpProtocolClient, resource_id: str) -> dict[str, Any]:
        info = client.get_resource_info(resource_id, timeout_ms=self._request_timeout_ms())
        if not info.ok:
            raise RuntimeError(info.message or "get_resource_info failed")
        resource = (info.payload or {}).get("resource") if isinstance(info.payload, dict) else None
        if not isinstance(resource, dict):
            raise RuntimeError("resource not found")
        metadata = resource.get("metadata") if isinstance(resource.get("metadata"), dict) else {}
        if metadata.get("kind") != "collection":
            raise RuntimeError("resource is not a collection")
        return {
            "collection_id": str(metadata.get("collection_id") or ""),
            "episode_id": str(metadata.get("episode_id") or ""),
            "collection_resource": resource,
            "local_path": str(metadata.get("local_path") or ""),
        }

    def _make_temp_record_root(self, prefix: str) -> str:
        root = self._app_temp_dir() / "record_transfers"
        root.mkdir(parents=True, exist_ok=True)
        return tempfile.mkdtemp(prefix=prefix, dir=str(root))

    def _app_temp_dir(self) -> Path:
        return local_root_from_config(self.config) / "tmp"

    def _get_preview_image_keys(self) -> list[str]:
        with self._preview_keys_lock:
            if self._preview_image_keys is None:
                self._preview_image_keys = _default_preview_image_names(self._binding)
            return list(self._preview_image_keys)

    def _get_source_state_keys(self) -> list[str]:
        with self._preview_keys_lock:
            if self._source_state_keys is None:
                self._source_state_keys = sorted({flow.source_observation for flow in self._control_flows})
            return list(self._source_state_keys)

    def _get_target_state_keys(self) -> list[str]:
        with self._preview_keys_lock:
            if self._target_state_keys is None:
                self._target_state_keys = [self._target_observation_name] if self._target_observation_name else []
            return list(self._target_state_keys)

    def _should_send_preview_images(self) -> bool:
        with self._preview_demand_lock:
            return self._peer_preview_requested

    def _preview_resize_long_edge(self) -> int:
        try:
            return max(0, int(self.config.get("image_max_long_edge", 320)))
        except (TypeError, ValueError):
            return 320

    def _set_playback_status(self, payload: dict[str, Any]) -> None:
        self.playback.status = payload

    def _log_video_encoder_self_check(self) -> None:
        try:
            from rynnrcp_app_common.hardware_codec import log_video_encoder_self_check

            log_video_encoder_self_check(
                self._logger,
                preferred=self.config.get("video_encoder"),
                smoke_test=bool(self.config.get("video_encoder_smoke_test", True)),
                prefix="TeleopEncoder",
            )
        except Exception as exc:
            self._logger.warning("Video encoder self-check failed: %s", exc, exc_info=True)

    def _mark_metric(self, key: str, size: int = 0) -> None:
        now = time.monotonic()
        with self._metrics_lock:
            events = self._metric_events.setdefault(key, deque())
            events.append(now)
            self._trim_metric_events_unlocked(key, now)
            byte_key = f"{key}_bytes"
            if byte_key in self._metric_bytes:
                self._metric_bytes[byte_key] += int(size)

    def _reset_realtime_metrics(self) -> None:
        with self._metrics_lock:
            for events in self._metric_events.values():
                events.clear()
            for key in self._metric_bytes:
                self._metric_bytes[key] = 0
            self._action_latency_window.clear()
            self._action_rpc_window.clear()
            self._observation_rpc_window.clear()
            self._source_state_rpc_window.clear()
            self._target_state_rpc_window.clear()
            self._image_rpc_window.clear()

    def _trim_metric_events_unlocked(self, key: str, now: float) -> None:
        events = self._metric_events[key]
        while events and now - events[0] > 2.0:
            events.popleft()

    def _fps_unlocked(self, key: str, now: float) -> float:
        events = self._metric_events[key]
        self._trim_metric_events_unlocked(key, now)
        return len(events) / 2.0

    def _record_action_latency(self, value_ms: float) -> None:
        with self._metrics_lock:
            self._action_latency_window.append(float(value_ms))

    def _record_action_rpc(self, value_ms: float) -> None:
        with self._metrics_lock:
            self._action_rpc_window.append(float(value_ms))

    def _record_observation_rpc(self, value_ms: float) -> None:
        with self._metrics_lock:
            self._observation_rpc_window.append(float(value_ms))

    def _record_source_state_rpc(self, value_ms: float) -> None:
        with self._metrics_lock:
            self._source_state_rpc_window.append(float(value_ms))

    def _record_target_state_rpc(self, value_ms: float) -> None:
        with self._metrics_lock:
            self._target_state_rpc_window.append(float(value_ms))

    def _record_image_rpc(self, value_ms: float) -> None:
        with self._metrics_lock:
            self._image_rpc_window.append(float(value_ms))

    def _log_loop_error(self, scope: str, message: str, exc: Exception) -> None:
        now = time.monotonic()
        error = str(exc)
        key = f"{type(exc).__name__}:{error}"
        with self._error_log_lock:
            state = self._loop_error_logs.get(scope)
            if state is None or state.get("key") != key:
                self._loop_error_logs[scope] = {"key": key, "last_at": now, "suppressed": 0}
                self._logger.warning("%s: %s", message, error)
                return

            suppressed = int(state.get("suppressed") or 0) + 1
            if now - float(state.get("last_at") or 0.0) >= self._error_log_interval_s:
                self._logger.warning("%s: %s (suppressed %d repeated errors)", message, error, suppressed)
                state["last_at"] = now
                state["suppressed"] = 0
            else:
                state["suppressed"] = suppressed

    def _clear_loop_error(self, scope: str) -> None:
        with self._error_log_lock:
            self._loop_error_logs.pop(scope, None)


def _load_yaml(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as fh:
        value = yaml.safe_load(fh) or {}
    if not isinstance(value, dict):
        raise ValueError("config must be a YAML mapping")
    return value


def _merge_default_config(config: Optional[dict[str, Any]]) -> dict[str, Any]:
    defaults_path = Path(__file__).with_name("default_config.yaml")
    raw = yaml.safe_load(defaults_path.read_text(encoding="utf-8")) or {}
    defaults = raw.get("defaults") if isinstance(raw, dict) else {}
    merged = dict(defaults or {})
    if isinstance(raw, dict):
        for key in ("config_type", "version"):
            if key in raw:
                merged[key] = raw[key]
    if isinstance(raw, dict) and isinstance(raw.get("app"), dict):
        merged["app"] = dict(raw["app"])
    if isinstance(raw, dict) and isinstance(raw.get("discovery"), dict):
        merged["discovery"] = dict(raw["discovery"])
    if config:
        merged.update(config)
    return merged


def _optional_path(value: Any, robot_root: Path) -> Path | None:
    if not value:
        return None
    return resolve_robot_path(str(value), robot_root)


def _config_value(app_cfg: dict[str, Any], root_cfg: dict[str, Any], key: str, default: Any) -> Any:
    if key in app_cfg:
        return app_cfg[key]
    if key in root_cfg:
        return root_cfg[key]
    return default


def _connection_status(status: str) -> dict[str, Any]:
    return {
        "side": "",
        "status": status,
        "message": "",
        "robot_id": "",
        "robot_name": "",
        "endpoint": "",
        "endpoint_source": "",
        "updated_at": time.time(),
        "last_online_at": None,
        "last_offline_at": None,
    }


def _record_item_descriptor(name: str, descriptor: dict[str, Any] | None = None) -> dict[str, Any]:
    descriptor = dict(descriptor or {})
    parts = str(name).split(".")
    category = str(descriptor.get("category") or (parts[0] if parts else ""))
    component = str(descriptor.get("component_name") or (parts[1] if len(parts) >= 3 else ""))
    object_name = str(descriptor.get("object_name") or (parts[2] if len(parts) >= 3 else parts[-1] if parts else name))
    item_type = str(descriptor.get("type") or object_name)
    return {
        "protocol_name": str(name),
        "category": category,
        "component_name": component,
        "name": object_name,
        "type": item_type,
        "description": str(descriptor.get("description") or ""),
    }


def _select_unique(manifests: list[ServerManifest], robot_id: str) -> ServerManifest:
    matches = [item for item in manifests if item.robot_id == robot_id]
    if not matches:
        raise ValueError(f"robot_id {robot_id!r} was not found")
    return _preferred_manifest(matches)


def _dedupe_robot_manifests(manifests: list[ServerManifest]) -> list[ServerManifest]:
    selected: dict[str, ServerManifest] = {}
    for item in manifests:
        current = selected.get(item.robot_id)
        if current is None or _endpoint_preference(item) < _endpoint_preference(current):
            selected[item.robot_id] = item
    return sorted(selected.values(), key=lambda item: (item.robot_name or item.robot_id, item.robot_id))


def _preferred_manifest(manifests: list[ServerManifest]) -> ServerManifest:
    return min(manifests, key=_endpoint_preference)


def _endpoint_preference(manifest: ServerManifest) -> tuple[int, str]:
    endpoint = manifest.endpoint
    if endpoint is None:
        return (99, "")
    address = str(endpoint.address)
    if endpoint.source == "static":
        rank = 0
    elif endpoint.source == "local_registry" or address.startswith(("127.", "localhost")):
        rank = 1
    elif endpoint.source == "mdns":
        rank = 2
    else:
        rank = 50
    return (rank, address)


def _connect_manifest(manifest: ServerManifest, client: ClientInterface) -> RcpProtocolClient:
    if manifest.endpoint is None:
        raise ValueError(f"robot_id {manifest.robot_id!r} has no endpoint")
    return RcpProtocolClient(client.connect(manifest.endpoint))


def _build_control_flows(
    source: ServerManifest,
    target: ServerManifest,
    raw_flows: list[dict[str, Any]] | None,
) -> list[ControlFlow]:
    if not raw_flows:
        raise ValueError("control_flows is required")
    if not isinstance(raw_flows, list):
        raise ValueError("control_flows must be a list")
    source_by_name = {
        str(item["name"]): dict(item)
        for item in source.observations
        if item.get("name")
    }
    target_by_name = {
        str(item["name"]): dict(item)
        for item in target.actions
        if item.get("name")
    }
    flows: list[ControlFlow] = []
    seen: set[tuple[str, str]] = set()
    for index, raw in enumerate(raw_flows):
        if not isinstance(raw, dict):
            raise ValueError(f"control_flows[{index}] must be an object")
        source_name = str(raw.get("source_observation") or "").strip()
        target_name = str(raw.get("target_action") or "").strip()
        if not source_name:
            raise ValueError(f"control_flows[{index}].source_observation is required")
        if not target_name:
            raise ValueError(f"control_flows[{index}].target_action is required")
        pair = (source_name, target_name)
        if pair in seen:
            raise ValueError(f"duplicate control flow: {source_name} -> {target_name}")
        seen.add(pair)
        source_item = source_by_name.get(source_name)
        if source_item is None:
            raise ValueError(f"source observation not found: {source_name}")
        if source_item.get("type") == "image":
            raise ValueError(f"image observation cannot be used as control source: {source_name}")
        target_item = target_by_name.get(target_name)
        if target_item is None:
            raise ValueError(f"target action not found: {target_name}")
        source_type = str(source_item.get("type") or "")
        target_type = str(target_item.get("type") or "")
        if not _can_pass_observation_to_action(source_type, target_type):
            raise ValueError(
                f"control flow value schema mismatch: {source_name}({source_type}) "
                f"cannot drive {target_name}({target_type}) directly"
            )
        flows.append(
            ControlFlow(
                source_observation=source_name,
                source_type=source_type,
                target_action=target_name,
                target_type=target_type,
            )
        )
    return flows


def _can_pass_observation_to_action(observation_type: str, action_type: str) -> bool:
    return (observation_type, action_type) in {
        ("joint_state", "joint_position"),
        ("ee_pose", "ee_pose"),
        ("gripper_state", "gripper"),
    }


def _tag_server_resource(resource: dict[str, Any], side: str) -> dict[str, Any]:
    item = dict(resource)
    item["location_type"] = "server"
    item["side"] = side
    item["location"] = "server"
    return item


def _resource_episode_ref(side: str, resource_id: str) -> str:
    return f"{_RESOURCE_EPISODE_PREFIX}{side}/{resource_id}"


def _parse_resource_episode_ref(value: str) -> tuple[str, str]:
    raw = value[len(_RESOURCE_EPISODE_PREFIX):]
    side, sep, resource_id = raw.partition("/")
    if sep != "/" or side not in ("source", "target") or not resource_id:
        raise ValueError("invalid resource episode reference")
    return side, resource_id


def _local_collection_path(collection: dict[str, Any]) -> str:
    raw_path = str(collection.get("local_path") or "").strip()
    if not raw_path:
        return ""
    path = Path(raw_path).expanduser().resolve()
    if not (path / "collection_meta.json").is_file():
        return ""
    return str(path)


def _server_location_is_local(location: dict[str, Any]) -> bool:
    if not location.get("online"):
        return False
    if location.get("endpoint_source") == "local_registry":
        return True
    endpoint = str(location.get("endpoint") or "").lower()
    return _endpoint_host(endpoint) in _local_endpoint_hosts()


def _endpoint_host(endpoint: str) -> str:
    value = str(endpoint or "").strip().lower()
    if not value:
        return ""
    if "://" in value:
        value = value.split("://", 1)[1]
    value = value.split("/", 1)[0]
    if value.startswith("["):
        return value[1:].split("]", 1)[0]
    if value.count(":") == 1:
        return value.rsplit(":", 1)[0]
    return value


_LOCAL_ENDPOINT_HOSTS: set[str] | None = None


def _local_endpoint_hosts() -> set[str]:
    global _LOCAL_ENDPOINT_HOSTS
    if _LOCAL_ENDPOINT_HOSTS is not None:
        return _LOCAL_ENDPOINT_HOSTS

    hosts = {"localhost", "127.0.0.1", "::1", "0.0.0.0"}
    names = {socket.gethostname(), socket.getfqdn(), "localhost"}
    for name in {item.lower() for item in names if item}:
        hosts.add(name)
        try:
            hosts.update(str(info[4][0]).lower() for info in socket.getaddrinfo(name, None))
        except OSError:
            pass
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            hosts.add(str(sock.getsockname()[0]).lower())
    except OSError:
        pass

    _LOCAL_ENDPOINT_HOSTS = hosts
    return hosts


def _merge_data_collections(
    base_collections: list[dict[str, Any]],
    incoming_collections: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    merged: dict[str, dict[str, Any]] = {}
    for collection in [*base_collections, *incoming_collections]:
        collection_id = str(collection.get("collection_id") or "")
        if not collection_id:
            continue
        target = merged.setdefault(
            collection_id,
            {"collection_id": collection_id, "task_prompts": {}, "size": 0},
        )
        for prompt in collection.get("task_prompts") or []:
            if not isinstance(prompt, dict):
                continue
            task_prompt = str(prompt.get("task_prompt") or "")
            target_prompt = target["task_prompts"].setdefault(
                task_prompt,
                {"task_prompt": task_prompt, "episodes": {}},
            )
            for episode in prompt.get("episodes") or []:
                if not isinstance(episode, dict):
                    continue
                episode_key = str(episode.get("episode_id") or episode.get("episode_name") or episode.get("path") or episode.get("resource_id") or "")
                if not episode_key:
                    continue
                existing = target_prompt["episodes"].get(episode_key)
                target_prompt["episodes"][episode_key] = _preferred_episode(existing, dict(episode))

    result: list[dict[str, Any]] = []
    for collection in merged.values():
        prompts: list[dict[str, Any]] = []
        collection_size = 0
        for prompt in collection["task_prompts"].values():
            episodes = list(prompt["episodes"].values())
            episodes.sort(key=lambda ep: float(ep.get("created_at_unix") or 0.0), reverse=True)
            collection_size += sum(int(ep.get("size") or 0) for ep in episodes)
            latest = max((float(ep.get("created_at_unix") or 0.0) for ep in episodes), default=0.0)
            prompts.append({
                "task_prompt": prompt["task_prompt"],
                "episodes": episodes,
                "latest_created_at_unix": latest,
            })
        prompts.sort(key=lambda item: item.get("latest_created_at_unix", 0.0), reverse=True)
        result.append({
            "collection_id": collection["collection_id"],
            "task_prompts": prompts,
            "size": collection_size,
            "size_formatted": format_bytes(collection_size),
            "latest_created_at_unix": max(
                (float(item.get("latest_created_at_unix") or 0.0) for item in prompts),
                default=0.0,
            ),
        })
    result.sort(key=lambda item: item.get("latest_created_at_unix", 0.0), reverse=True)
    return result


def _preferred_episode(existing: dict[str, Any] | None, incoming: dict[str, Any]) -> dict[str, Any]:
    if existing is None:
        return incoming
    existing_remote = existing.get("location") in {"server", "remote"}
    incoming_remote = incoming.get("location") in {"server", "remote"}
    if existing_remote and not incoming_remote:
        return incoming
    if incoming_remote and not existing_remote:
        return existing
    existing_time = float(existing.get("created_at_unix") or 0.0)
    incoming_time = float(incoming.get("created_at_unix") or 0.0)
    return incoming if incoming_time > existing_time else existing


def _server_collections_from_resources(resources: list[dict[str, Any]], *, side: str) -> list[dict[str, Any]]:
    collections: dict[str, dict[str, Any]] = {}
    for resource in resources:
        metadata = resource.get("metadata") if isinstance(resource.get("metadata"), dict) else {}
        collection_id = str(metadata.get("collection_id") or "server_collection")
        episode_id = str(metadata.get("episode_id") or resource.get("name") or resource.get("resource_id") or "")
        task_prompt = str(metadata.get("task_prompt") or "server")
        size = int(resource.get("size_bytes") or 0)
        collection = collections.setdefault(
            collection_id,
            {"collection_id": collection_id, "task_prompts": {}, "size": 0},
        )
        collection["size"] += size
        prompt = collection["task_prompts"].setdefault(
            task_prompt,
            {"task_prompt": task_prompt, "episodes": []},
        )
        prompt["episodes"].append({
            "episode_name": episode_id,
            "episode_id": episode_id,
            "path": _resource_episode_ref(side, str(resource.get("resource_id") or "")),
            "resource_id": str(resource.get("resource_id") or ""),
            "status": "complete",
            "frames": 0,
            "size": size,
            "size_formatted": format_bytes(size),
            "collection_id": collection_id,
            "task_prompt": task_prompt,
            "created_at_unix": float(metadata.get("started_at_unix") or resource.get("created_at") or 0.0),
            "side": side,
            "location_type": "server",
            "location": "server",
        })
    data_collections: list[dict[str, Any]] = []
    for collection in collections.values():
        prompts = list(collection["task_prompts"].values())
        for prompt in prompts:
            prompt["episodes"].sort(key=lambda ep: ep.get("created_at_unix", 0.0), reverse=True)
            prompt["latest_created_at_unix"] = max(
                (float(ep.get("created_at_unix") or 0.0) for ep in prompt["episodes"]),
                default=0.0,
            )
        prompts.sort(key=lambda item: item.get("latest_created_at_unix", 0.0), reverse=True)
        data_collections.append({
            "collection_id": collection["collection_id"],
            "task_prompts": prompts,
            "size": collection["size"],
            "size_formatted": format_bytes(collection["size"]),
            "latest_created_at_unix": max(
                (float(item.get("latest_created_at_unix") or 0.0) for item in prompts),
                default=0.0,
            ),
            "side": side,
            "location_type": "server",
            "location": "server",
        })
    data_collections.sort(key=lambda item: item.get("latest_created_at_unix", 0.0), reverse=True)
    return data_collections


def _select_target_state_observation(target: ServerManifest, preferred_type: str) -> dict[str, Any]:
    observations = [dict(item) for item in target.observations if item.get("name") and item.get("type") != "image"]
    if not observations:
        raise ValueError("target robot does not provide state observations")
    for item in observations:
        if item.get("type") == preferred_type:
            return item
    return observations[0]


def _state_observation_names(observations: list[dict[str, Any]]) -> list[str]:
    return [
        str(item["name"])
        for item in observations
        if item.get("name") and item.get("type") != "image"
    ]


def _image_observation_names(observations: list[dict[str, Any]]) -> list[str]:
    return [
        str(item["name"])
        for item in observations
        if item.get("name") and item.get("type") == "image"
    ]


def _filter_names(names: list[str], allowed: list[str]) -> list[str]:
    allowed_set = set(allowed)
    result: list[str] = []
    for name in names:
        value = str(name or "")
        if value in allowed_set and value not in result:
            result.append(value)
    return result


def _as_name_list(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return [str(item) for item in value]


def _default_preview_image_names(binding: TeleopBinding | None) -> list[str]:
    if binding is None:
        return []
    return _image_observation_names(binding.target.observations)


def _observation_values(response: Any) -> dict[str, Any]:
    return {
        name: item.get("value")
        for name, item in _observation_items(response).items()
    }


def _observation_items(response: Any) -> dict[str, dict[str, Any]]:
    if not getattr(response, "ok", False):
        raise InterfaceError(str(getattr(response, "message", "get_observations failed")))
    payload = getattr(response, "payload", None)
    if not isinstance(payload, dict):
        raise RuntimeError("get_observations returned invalid payload")
    observations = payload.get("observations")
    if not isinstance(observations, list):
        raise RuntimeError("get_observations payload missing observations")
    items: dict[str, dict[str, Any]] = {}
    for item in observations:
        if isinstance(item, dict) and item.get("name"):
            items[str(item["name"])] = item
    return items


def _as_bytes(value: Any) -> bytes | None:
    if isinstance(value, bytes):
        return value
    if isinstance(value, bytearray):
        return bytes(value)
    if isinstance(value, memoryview):
        return value.tobytes()
    return None


def _percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(float(v) for v in values)
    index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * q))))
    return ordered[index]


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Run the RynnRCP Teleop app.")
    parser.add_argument("--config", help="Optional Teleop app YAML config path.")
    args = parser.parse_args(argv)
    config = _merge_default_config(_load_yaml(args.config) if args.config else {})
    configure_logging(
        level=logging.INFO,
        sinks=["stderr", "file"],
        file_path=str(log_file_from_config(config, "teleop.log")),
    )
    app_config = dict(config)
    if args.config:
        app_config["config_file"] = args.config
    app = TeleopApp(config=app_config)
    try:
        app.run_web()
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        logging.getLogger(__name__).exception("Teleop app failed: %s", exc)
        print(f"Teleop app failed: {exc}", file=sys.stderr)
        return 1
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
