"""ActionService: expose RCP Action tools."""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Mapping
from typing import Any, Dict, Iterable, List

from rynnrcp.ipc.channel import ChannelManager
from rynnrcp.ipc.transport import parse_transport_level

from .base_service import BaseService
from rynnrcp.config.runner_config import RunnerOutputSpec
from rynnrcp.protocol.methods import LIST_ACTIONS, RUN_ACTION_CHUNK, STOP_ACTION
from rynnrcp.runtime.tool_bus import ToolBus
from rynnrcp.protocol.action_codecs import action_channel_type, action_input_schema, encode_action_value
from rynnrcp.utils.payload import (
    INLINE_JSON_PAYLOAD_MAX_BYTES,
    json_dumps_bytes,
    json_safe,
    pack_channel_message,
    shared_payload_ref_payload,
)
from rynnrcp.utils.shared_data_store import SharedDataStore

logger = logging.getLogger(__name__)


def _build_action_descriptors(outputs: Iterable[RunnerOutputSpec]) -> List[Dict[str, Any]]:
    descriptors: Dict[str, Dict[str, Any]] = {}
    for output in outputs:
        descriptor = _action_descriptor(output)
        existing = descriptors.get(str(descriptor["name"]))
        if existing is None:
            descriptors[str(descriptor["name"])] = descriptor
            continue
        existing_channels = existing.setdefault("_channels", [])
        for channel in descriptor.get("_channels", []):
            if channel not in existing_channels:
                existing_channels.append(channel)
    return list(descriptors.values())


def _action_descriptor(output: RunnerOutputSpec) -> Dict[str, Any]:
    params = output.params if isinstance(output.params, Mapping) else {}
    action_type = str(params["rcp_action_type"])
    name = str(params["rcp_action_name"])
    descriptor: Dict[str, Any] = {
        "name": name,
        "type": action_type,
        "input_schema": params.get("input_schema") or action_input_schema(action_type, name),
        "_channels": [output.channel],
    }
    description = params.get("description")
    if description:
        descriptor["description"] = str(description)
    frame_rate = params.get("frame_rate")
    if frame_rate is not None:
        descriptor["frame_rate"] = float(frame_rate)
    component_name = params.get("component_name")
    if component_name:
        descriptor["component_name"] = str(component_name)
    return descriptor

def _public_action_descriptor(action: Mapping[str, Any]) -> Dict[str, Any]:
    return {str(key): value for key, value in action.items() if not str(key).startswith("_")}


class ActionService(BaseService):
    """Publish actions to configured runner output channels."""

    def __init__(
        self,
        bus: ToolBus,
        outputs: Iterable[RunnerOutputSpec],
    ) -> None:
        super().__init__(bus, "action_service")
        self._outputs = list(outputs)
        self._actions = _build_action_descriptors(self._outputs)
        self._publishers: Dict[str, Dict[str, Any]] = {}
        self._action_stores: Dict[str, SharedDataStore] = {}
        self._action_stop_event = threading.Event()
        self._latest_actions: Dict[str, Dict[str, Any]] = {}
        self._latest_actions_lock = threading.RLock()
        self._latest_action_capture_enabled = False
        for output in self._outputs:
            self._add_action_publisher(
                output.channel,
                output.msg_size,
                output.channel_transport,
                shared_data_buffer_size=output.params.get("shared_data_buffer_size"),
                shared_data_slot_count=output.params.get("shared_data_slot_count"),
            )
        self._action_by_name = {action["name"]: action for action in self._actions}

    @property
    def latest_actions(self) -> Dict[str, Dict[str, Any]]:
        """Return the last successfully published frame for each Action."""
        with self._latest_actions_lock:
            return {name: dict(value) for name, value in self._latest_actions.items()}

    def set_latest_action_capture(self, enabled: bool) -> None:
        """Enable the optional latest-Action snapshot used by the debug UI."""
        enabled = bool(enabled)
        with self._latest_actions_lock:
            if self._latest_action_capture_enabled == enabled:
                return
            self._latest_action_capture_enabled = enabled
            if not enabled:
                self._latest_actions.clear()

    def bind(self) -> None:
        self._register_tool(
            LIST_ACTIONS.name,
            self.list_actions,
            input_schema=LIST_ACTIONS.input_schema,
            output_schema=LIST_ACTIONS.output_schema,
            description=LIST_ACTIONS.description,
        )
        self._register_tool(
            RUN_ACTION_CHUNK.name,
            self.run_action_chunk,
            input_schema=RUN_ACTION_CHUNK.input_schema,
            output_schema=RUN_ACTION_CHUNK.output_schema,
            description=RUN_ACTION_CHUNK.description,
        )
        self._register_tool(
            STOP_ACTION.name,
            self.stop_action,
            input_schema=STOP_ACTION.input_schema,
            output_schema=STOP_ACTION.output_schema,
            description=STOP_ACTION.description,
        )

    def unbind(self) -> None:
        for store in self._action_stores.values():
            store.close()
        self._action_stores.clear()
        super().unbind()

    def list_actions(self) -> Dict[str, Any]:
        return ToolBus.make_result(
            True,
            result={"actions": [_public_action_descriptor(action) for action in self._actions]},
            message="OK",
        )

    def run_action_chunk(
        self,
        name: str | None = None,
        frames: Any = None,
        frame_rate: float = 30,
        **kwargs: Any,
    ) -> Dict[str, Any]:
        """Publish finite RCP Action frames."""
        action_name = str(name or "")
        action = self._action_by_name.get(action_name)
        if action is None:
            return _action_chunk_result(False, message=f"Unknown action: {action_name}")
        if bool(kwargs.get("async", False)):
            # TODO: Implement async action_step callbacks after the Interface transport
            # supports server push/streaming. Current gRPC transport is unary-unary.
            return _action_chunk_result(False, message="async action execution is not implemented")
        try:
            values = _normalize_action_frames(frames)
        except (KeyError, TypeError) as exc:
            return _action_chunk_result(False, message=str(exc))
        if not values:
            return _action_chunk_result(False, message="No action frames provided")

        try:
            frame_rate_value = float(frame_rate)
        except (TypeError, ValueError):
            return _action_chunk_result(False, message="frame_rate must be a number")
        if frame_rate_value <= 0:
            return _action_chunk_result(False, message="frame_rate must be greater than 0")

        frames_sent = 0
        expect_frames = len(values)
        interval = 1.0 / frame_rate_value
        start_time = time.time()
        action_type = str(action["type"])
        try:
            payload_action_type = action_channel_type(action_type)
        except ValueError as exc:
            return _action_chunk_result(False, accepted_frames=frames_sent, message=str(exc))
        output_channels = set(action.get("_channels") or [])
        if not output_channels:
            return _action_chunk_result(False, message=f"No output channel configured for action: {action_name}")
        self._action_stop_event.clear()

        for index, value in enumerate(values):
            if self._action_stop_event.is_set():
                return _action_chunk_result(
                    False,
                    accepted_frames=frames_sent,
                    message="Action stopped",
                )
            try:
                frame = encode_action_value(action_type, value, action_name)
                self._publish_action(
                    action_name,
                    frame,
                    frame_rate_value,
                    payload_action_type,
                    output_channels=output_channels,
                )
                frames_sent += 1
            except Exception as exc:
                logger.error("Failed to run action frame %d: %s", index, exc, exc_info=True)
                return _action_chunk_result(
                    False,
                    accepted_frames=frames_sent,
                    message=f"Error at frame {index}: {exc}",
                )

            if index < expect_frames - 1:
                sleep_time = (index + 1) * interval - (time.time() - start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        return _action_chunk_result(frames_sent == expect_frames, accepted_frames=frames_sent)

    def stop_action(self, reason: str | None = None) -> Dict[str, Any]:
        self._action_stop_event.set()
        return ToolBus.make_result(True, result={"stopped": True}, message="OK")

    def _add_action_publisher(
        self,
        channel: str,
        msg_size: int,
        channel_transport: str,
        *,
        shared_data_buffer_size: Any = None,
        shared_data_slot_count: Any = None,
    ) -> None:
        if channel in self._publishers:
            return
        self._publishers[channel] = {
            "publisher": ChannelManager.instance().create_publisher(
                channel,
                int(msg_size),
                transport=parse_transport_level(channel_transport),
            ),
            "msg_size": int(msg_size),
            "transport": channel_transport,
            "shared_data_buffer_size": _optional_int(shared_data_buffer_size),
            "shared_data_slot_count": _optional_int(shared_data_slot_count),
        }

    def _publish_action(
        self,
        action_name: str,
        frame: Any,
        fps: float,
        action_type: str,
        *,
        output_channels: set[str] | None = None,
    ) -> None:
        timestamp = time.time()
        payload_bytes = _action_payload_bytes(
            action_name=action_name,
            frame=frame,
            fps=fps,
            action_type=action_type,
            timestamp=timestamp,
        )
        for channel, info in self._publishers.items():
            if output_channels and channel not in output_channels:
                continue
            data = self._pack_action_message(channel, info, timestamp, payload_bytes, action_name)
            msg_size = int(info["msg_size"])
            if len(data) > msg_size:
                raise ValueError(
                    f"Action frame exceeds action channel msg_size for {channel}: {len(data)} > {msg_size}"
                )
            info["publisher"].publish(data)
        if self._latest_action_capture_enabled:
            with self._latest_actions_lock:
                self._latest_actions[action_name] = {
                    "timestamp": timestamp,
                    "value": frame,
                    "frame_rate": float(fps),
                    "type": action_type,
                }

    def _pack_action_message(
        self,
        channel: str,
        info: Dict[str, Any],
        timestamp: float,
        payload_bytes: bytes,
        action_name: str,
    ) -> bytes:
        msg_size = int(info["msg_size"])
        if len(payload_bytes) <= min(INLINE_JSON_PAYLOAD_MAX_BYTES, max(0, msg_size - 8)):
            return pack_channel_message(timestamp, payload_bytes)

        store = self._action_stores.get(channel)
        if store is None:
            store = SharedDataStore(
                f"action:{channel}",
                create=True,
                buffer_size=info.get("shared_data_buffer_size"),
                slot_count=info.get("shared_data_slot_count"),
            )
            self._action_stores[channel] = store
        ref_payload = shared_payload_ref_payload(
            store=store,
            object_name=action_name,
            payload_type="json",
            timestamp=timestamp,
            data=payload_bytes,
            codec="json_action",
        )
        return pack_channel_message(timestamp, ref_payload)


def _optional_int(value: Any) -> int | None:
    if value is None:
        return None
    return int(value)


def _action_chunk_result(
    success: bool,
    *,
    accepted_frames: int = 0,
    message: str = "OK",
) -> Dict[str, Any]:
    return ToolBus.make_result(
        success,
        result={"accepted_frames": accepted_frames},
        message=message,
    )


def _normalize_action_frames(frames: Any) -> List[Any]:
    if frames is None:
        raise KeyError("frames is required")
    if not isinstance(frames, list):
        raise TypeError("frames must be a list")

    values: List[Any] = []
    for index, frame in enumerate(frames):
        if not isinstance(frame, Mapping):
            raise TypeError(f"frames[{index}] must be an object")
        values.append(frame)
    return values


def _action_payload_bytes(
    *,
    action_name: str,
    frame: Any,
    fps: float,
    action_type: str,
    timestamp: float,
) -> bytes:
    extra: Dict[str, Any] = {
        "action_name": str(action_name),
        str(action_name): json_safe(frame),
        "action_type": action_type,
        "fps": float(fps),
    }
    return json_dumps_bytes({"timestamp": timestamp, "extra": extra})
