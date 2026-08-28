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
from rynnrcp.protocol.action_codecs import (
    action_channel_type,
    action_input_schema,
    encode_action_value,
)
from rynnrcp.utils.log_gate import LogGate
from rynnrcp.utils.payload import (
    INLINE_JSON_PAYLOAD_MAX_BYTES,
    json_dumps_bytes,
    json_safe,
    pack_channel_message,
    shared_payload_ref_payload,
)
from rynnrcp.utils.redaction import describe_payload
from rynnrcp.utils.shared_data_store import SharedDataStore

logger = logging.getLogger(__name__)


def _build_action_descriptors(
    outputs: Iterable[RunnerOutputSpec],
) -> List[Dict[str, Any]]:
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
        "input_schema": params.get("input_schema")
        or action_input_schema(action_type, name),
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
    return {
        str(key): value for key, value in action.items() if not str(key).startswith("_")
    }


def _joint_position_diagnostics(value: Any) -> Dict[str, Any]:
    positions = value.get("joint_positions") if isinstance(value, Mapping) else None
    if not isinstance(positions, list):
        return {"dimension": None, "all_zero": None, "values": describe_payload(value)}
    try:
        values = [float(item) for item in positions]
    except (TypeError, ValueError):
        return {
            "dimension": len(positions),
            "all_zero": None,
            "values": describe_payload(positions),
        }
    displayed = ", ".join(f"{item:.4f}" for item in values[:12])
    remaining = len(values) - 12
    suffix = f", ... (+{remaining})" if remaining > 0 else ""
    return {
        "dimension": len(values),
        "all_zero": all(item == 0.0 for item in values),
        "values": f"[{displayed}{suffix}]",
    }


def _configured_joint_dimension(action: Mapping[str, Any]) -> int | None:
    schema = action.get("input_schema")
    fields = schema.get("fields") if isinstance(schema, Mapping) else None
    joint_schema = (
        fields.get("joint_positions") if isinstance(fields, Mapping) else None
    )
    shape = joint_schema.get("shape") if isinstance(joint_schema, Mapping) else None
    if isinstance(shape, list) and len(shape) == 1 and isinstance(shape[0], int):
        return int(shape[0])
    return None


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
        self._action_log_stats: Dict[str, Dict[str, Any]] = {}
        self._action_log_lock = threading.Lock()
        self._action_rejection_log: LogGate | None = None
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
            result={
                "actions": [
                    _public_action_descriptor(action) for action in self._actions
                ]
            },
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
            return self._reject_action(action_name, f"Unknown action: {action_name}")
        if bool(kwargs.get("async", False)):
            # TODO: Implement async action_step callbacks after the Interface transport
            # supports server push/streaming. Current gRPC transport is unary-unary.
            return self._reject_action(
                action_name, "async action execution is not implemented"
            )
        try:
            values = _normalize_action_frames(frames)
        except (KeyError, TypeError) as exc:
            return self._reject_action(action_name, str(exc))
        if not values:
            return self._reject_action(action_name, "No action frames provided")

        try:
            frame_rate_value = float(frame_rate)
        except (TypeError, ValueError):
            return self._reject_action(action_name, "frame_rate must be a number")
        if frame_rate_value <= 0:
            return self._reject_action(action_name, "frame_rate must be greater than 0")

        frames_sent = 0
        expect_frames = len(values)
        interval = 1.0 / frame_rate_value
        start_time = time.time()
        action_type = str(action["type"])
        try:
            payload_action_type = action_channel_type(action_type)
        except ValueError as exc:
            return self._reject_action(
                action_name, str(exc), accepted_frames=frames_sent
            )
        output_channels = set(action.get("_channels") or [])
        if not output_channels:
            return self._reject_action(
                action_name,
                f"No output channel configured for action: {action_name}",
            )
        rejection_log = self._action_rejection_log
        if rejection_log is not None:
            self._action_rejection_log = None
            rejection_log.success()
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
                self._log_action_publish(
                    action,
                    frame,
                    frame_rate_value,
                    output_channels,
                )
            except Exception as exc:
                logger.error(
                    "[ActionService][PUBLISH_ERROR] action=%s frame=%d/%d error=%s",
                    action_name,
                    index + 1,
                    expect_frames,
                    exc,
                    exc_info=True,
                )
                return _action_chunk_result(
                    False,
                    accepted_frames=frames_sent,
                    message=f"Error at frame {index}: {exc}",
                )

            if index < expect_frames - 1:
                sleep_time = (index + 1) * interval - (time.time() - start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        return _action_chunk_result(
            frames_sent == expect_frames, accepted_frames=frames_sent
        )

    def _reject_action(
        self,
        action_name: str,
        message: str,
        *,
        accepted_frames: int = 0,
    ) -> Dict[str, Any]:
        if self._action_rejection_log is None:
            self._action_rejection_log = LogGate(
                logger,
                "ActionService/REQUEST_REJECTED",
                interval_s=5.0,
                level=logging.WARNING,
            )
        self._action_rejection_log.failure(
            "action=%s reason=%s; verify the action name, frame schema, and frame rate",
            action_name or "<missing>",
            message,
        )
        return _action_chunk_result(
            False,
            accepted_frames=accepted_frames,
            message=message,
        )

    def _log_action_publish(
        self,
        action: Mapping[str, Any],
        frame: Any,
        frame_rate: float,
        output_channels: set[str],
    ) -> None:
        action_name = str(action["name"])
        now = time.monotonic()
        diagnostics = _joint_position_diagnostics(frame)
        with self._action_log_lock:
            stats = self._action_log_stats.setdefault(
                action_name,
                {
                    "total": 0,
                    "window_count": 0,
                    "window_start": now,
                    "last_dimension": None,
                    "last_dimension_warning": 0.0,
                },
            )
            stats["total"] += 1
            stats["window_count"] += 1
            total = int(stats["total"])
            previous_dimension = stats["last_dimension"]
            dimension = diagnostics["dimension"]
            stats["last_dimension"] = dimension
            elapsed = now - float(stats["window_start"])
            should_log_progress = elapsed >= 5.0
            if should_log_progress:
                observed_rate = float(stats["window_count"]) / elapsed
                stats["window_count"] = 0
                stats["window_start"] = now
            else:
                observed_rate = 0.0

            expected_dimension = _configured_joint_dimension(action)
            dimension_mismatch = (
                expected_dimension is not None
                and dimension is not None
                and expected_dimension != dimension
                and now - float(stats["last_dimension_warning"]) >= 10.0
            )
            if dimension_mismatch:
                stats["last_dimension_warning"] = now

        if total == 1:
            logger.info(
                "[ActionService][FIRST_FRAME] action=%s requested_rate=%.2fHz "
                "channels=%s dim=%s all_zero=%s",
                action_name,
                frame_rate,
                sorted(output_channels),
                dimension,
                diagnostics["all_zero"],
            )
            logger.debug(
                "[ActionService][FIRST_FRAME_VALUES] action=%s values=%s",
                action_name,
                diagnostics["values"],
            )
        elif previous_dimension is not None and previous_dimension != dimension:
            logger.warning(
                "[ActionService][DIM_CHANGED] action=%s dim=%s->%s; "
                "compare the producer output with the configured action schema",
                action_name,
                previous_dimension,
                dimension,
            )
            logger.debug(
                "[ActionService][DIM_CHANGED_VALUES] action=%s values=%s",
                action_name,
                diagnostics["values"],
            )

        if dimension_mismatch:
            logger.warning(
                "[ActionService][DIM_MISMATCH] action=%s configured_dim=%d received_dim=%d; "
                "align the action bridge output with the robot configuration",
                action_name,
                expected_dimension,
                dimension,
            )

        if should_log_progress:
            logger.info(
                "[ActionService][PUBLISH] action=%s observed_rate=%.2fHz total=%d "
                "dim=%s all_zero=%s",
                action_name,
                observed_rate,
                total,
                dimension,
                diagnostics["all_zero"],
            )
            logger.debug(
                "[ActionService][PUBLISH_VALUES] action=%s values=%s",
                action_name,
                diagnostics["values"],
            )

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
            data = self._pack_action_message(
                channel, info, timestamp, payload_bytes, action_name
            )
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
        if len(payload_bytes) <= min(
            INLINE_JSON_PAYLOAD_MAX_BYTES, max(0, msg_size - 8)
        ):
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
