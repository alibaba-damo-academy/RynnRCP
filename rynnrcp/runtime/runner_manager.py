"""Runtime manager that owns configured runners.

Runners are the device-facing layer in RynnRCP. They poll inputs
such as robots and cameras, publish observations to Channels, and subscribe
to action Channels before writing commands to the real device transport.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from functools import lru_cache
from typing import Any, Dict, List

from rynnrcp.connectors.connector_factory import ConnectorFactory
from rynnrcp.config.runtime_config import RuntimeConfig
from rynnrcp.config.runner_config import (
    RunnerInputSpec,
    RunnerHealthSpec,
    RunnerOutputSpec,
    build_runner_config,
    clone_output_spec_with_target,
)
from rynnrcp.protocol.action_codecs import is_supported_channel_action_type
from rynnrcp.runtime.scheduler import Component, Scheduler
from rynnrcp.utils.imports import import_object
from rynnrcp.utils.payload import (
    INLINE_JSON_PAYLOAD_MAX_BYTES,
    bytes_shared_payload,
    close_shared_reader_cache,
    decode_action_payload,
    image_adapter_shared_payload,
    json_dumps_bytes,
    json_observation_payload,
    pack_channel_message,
    shared_payload_ref_payload,
)
from rynnrcp.utils.shared_data_store import SharedDataStore
from rynnrcp.utils import camel_to_snake

logger = logging.getLogger(__name__)

class ConnectorInputRunner:
    """Subscribe through a connector and publish normalized data to a Channel."""

    def __init__(
        self,
        connector_factory: ConnectorFactory,
        connector: str,
        params: Dict[str, Any],
        input_adapter: Any,
        channel_name: str,
        msg_size: int,
        object_name: str,
        payload_type: str,
        channel_transport: str = "shm",
        shared_data_buffer_size: int | None = None,
        shared_data_slot_count: int | None = None,
        scheduler: Scheduler | None = None,
        component_name: str | None = None,
    ) -> None:
        self._connector_factory = connector_factory
        self._connector = connector
        self._params = params
        self._input_adapter = input_adapter
        self._channel_name = channel_name
        self._msg_size = msg_size
        self._object_name = object_name
        self._payload_type = payload_type
        self._channel_transport = channel_transport
        self._shared_data_buffer_size = shared_data_buffer_size
        self._shared_data_slot_count = shared_data_slot_count
        self._scheduler = scheduler
        self._component_name = component_name
        self._publisher: Any = None
        self._shared_data_store: SharedDataStore | None = None
        self._last_drop_warning_at = 0.0

    def start(self) -> None:
        from rynnrcp.ipc.channel import ChannelManager
        from rynnrcp.ipc.transport import parse_transport_level

        self._publisher = ChannelManager.instance().create_publisher(
            self._channel_name,
            self._msg_size,
            transport=parse_transport_level(self._channel_transport),
        )
        params = dict(self._params)
        if self._scheduler is not None:
            params["_scheduler"] = self._scheduler
            params["_component_name"] = self._component_name or f"input:{self._channel_name}"
        self._connector_factory.sub(self._connector, params, self._on_message)

    def stop(self) -> None:
        if self._shared_data_store is not None:
            self._shared_data_store.close()
            self._shared_data_store = None

    def _on_message(self, msg: Any) -> None:
        if self._publisher is None:
            return
        timestamp, payload = self._to_channel_payload(msg)
        if not payload:
            return
        data = pack_channel_message(timestamp, payload)
        if len(data) <= self._msg_size:
            self._publisher.publish(data)
        else:
            self._record_payload_drop(len(data))

    def _to_channel_payload(self, msg: Any) -> tuple[float, bytes]:
        if self._payload_type == "bytes":
            store = self._get_shared_data_store()
            return bytes_shared_payload(msg, self._object_name, store)
        if self._payload_type == "image":
            store = self._get_shared_data_store()
            return image_adapter_shared_payload(
                msg,
                self._input_adapter,
                self._object_name,
                store=store,
            )
        timestamp, data = self._input_adapter.parse(msg)
        payload = json_observation_payload(data, self._object_name, timestamp)
        if len(payload) <= min(INLINE_JSON_PAYLOAD_MAX_BYTES, max(0, self._msg_size - 8)):
            return timestamp, payload
        store = self._get_shared_data_store()
        return timestamp, shared_payload_ref_payload(
            store=store,
            object_name=self._object_name,
            payload_type="json",
            timestamp=timestamp,
            data=payload,
            codec="json_observation",
        )

    def _get_shared_data_store(self) -> SharedDataStore:
        if self._shared_data_store is None:
            self._shared_data_store = SharedDataStore(
                self._shared_data_key(),
                create=True,
                buffer_size=self._shared_data_buffer_size,
                slot_count=self._shared_data_slot_count,
            )
        return self._shared_data_store

    def _shared_data_key(self) -> str:
        return f"input:{self._channel_name}:{self._object_name}"

    def _record_payload_drop(self, data_size: int) -> None:
        reason = (
            f"input payload too large for channel {self._channel_name}: "
            f"{data_size} > {self._msg_size}"
        )
        now = time.monotonic()
        if now - self._last_drop_warning_at >= 2.0:
            logger.warning(
                "%s (connector=%s, object_name=%s, component=%s)",
                reason,
                self._connector,
                self._object_name,
                self._component_name or "",
            )
            self._last_drop_warning_at = now


class ConnectorOutputTarget:
    """Subscribe to action channel and publish frames through a connector."""

    def __init__(
        self,
        connector_factory: ConnectorFactory,
        connector: str,
        params: Dict[str, Any],
        output_adapter: Any,
        output_config: Dict[str, Any],
        channel_name: str,
        msg_size: int,
        channel_transport: str,
        component_name: str | None = None,
        consume_mode: str = "queue",
    ) -> None:
        self._connector_factory = connector_factory
        self._connector = connector
        self._params = params
        self._output_adapter = output_adapter
        self._output_config = output_config
        self._channel_name = channel_name
        self._msg_size = int(msg_size)
        self._channel_transport = channel_transport
        self._component_name = component_name
        self._consume_mode = str(consume_mode or "queue").lower()
        self._subscriber: Any = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._last_drop_warning_at = 0.0
        self._shared_reader_cache: Dict[str, Any] = {}
        if self._consume_mode not in ("queue", "latest"):
            raise ValueError("action_consume_mode must be 'queue' or 'latest'")

    def start(self) -> None:
        from rynnrcp.ipc.channel import ChannelManager
        from rynnrcp.ipc.transport import parse_transport_level

        self._subscriber = ChannelManager.instance().create_subscriber(
            self._channel_name,
            self._msg_size,
            transport=parse_transport_level(self._channel_transport),
        )
        self._running = True
        self._thread = threading.Thread(
            target=self._loop,
            name=f"output_{self._component_name or self._channel_name}",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        close_shared_reader_cache(self._shared_reader_cache)

    def _loop(self) -> None:
        while self._running:
            self._poll_once(timeout_ms=500)

    def _poll_once(self, timeout_ms: int = 0) -> None:
        if not self._running or self._subscriber is None:
            return
        if self._consume_mode == "latest":
            raw = self._subscriber.read_latest()
            if raw is None and timeout_ms > 0:
                raw = self._subscriber.poll(timeout_ms=timeout_ms)
        else:
            raw = self._subscriber.poll(timeout_ms=timeout_ms)
        if raw is None or len(raw) < 8:
            return
        try:
            self._handle_payload(raw[8:])
        except Exception as exc:
            logger.warning(
                "Output target failed to handle action channel %s "
                "(connector=%s, component=%s): %s",
                self._channel_name,
                self._connector,
                self._component_name or "",
                exc,
                exc_info=True,
            )
            if self._scheduler is not None:
                raise

    def _handle_payload(self, payload: bytes) -> None:
        _data, extra = decode_action_payload(payload, self._channel_name, reader_cache=self._shared_reader_cache)
        action_name = str(extra["action_name"])
        action = extra[action_name]
        action_type = extra["action_type"]
        if not is_supported_channel_action_type(str(action_type)):
            raise ValueError(f"Unsupported action_type: {action_type}")
        fps = float(extra["fps"])
        step_output = {action_name: action, "action_type": action_type}
        frame_groups = self._output_adapter.build_step_output(
            step_output,
            [self._output_config],
            fps,
        )
        for frame_group in frame_groups:
            for connector, params, msg, _interval, _step in frame_group:
                self._connector_factory.pub(connector, params, msg)

class RunnerManager:
    """Build and lifecycle-manage runners declared by robot integration config."""

    def __init__(
        self,
        *,
        config: RuntimeConfig,
        runner_filter: set[str] | None = None,
    ) -> None:
        self._config = config
        self._robot_id = config.robot_id
        self._runner_filter = set(runner_filter or set())
        self._runner_config = build_runner_config(config, self._runner_filter)
        self._runners: List[Any] = []
        self._runner_names = list(self._runner_config.runner_names)
        self.input_specs: List[RunnerInputSpec] = list(self._runner_config.input_specs)
        self.health_specs: List[RunnerHealthSpec] = list(self._runner_config.health_specs)
        self.output_specs: List[RunnerOutputSpec] = []
        self.capabilities = dict(self._runner_config.capabilities)
        self._scheduler = Scheduler()
        connector_names = set(self._runner_config.connector_names)
        self._connector_factory = ConnectorFactory(
            connector_names=connector_names,
            connectors_config=config.connectors_config,
            node_name=_connector_node_name(config.robot_id, connector_names),
        )
        self._build()

    @property
    def runner_names(self) -> List[str]:
        return list(self._runner_names)

    @property
    def is_running(self) -> bool:
        return self._scheduler.is_running

    def start(self) -> None:
        for runner in self._runners:
            runner.start()
        self._scheduler.start()

    def stop(self) -> None:
        self._scheduler.stop()
        for runner in reversed(self._runners):
            try:
                runner.stop()
            except Exception as exc:
                logger.warning("Failed to stop runner %s: %s", runner, exc, exc_info=True)
        self._connector_factory.stop()

    def health_check(self) -> Dict[str, bool]:
        return {name: self.is_running for name in self.runner_names}

    def _build(self) -> None:
        for input_config in self._runner_config.inputs:
            self._build_input(
                runner_name=input_config.runner_name,
                input_name=input_config.name,
                protocol=input_config.protocol,
                adapter=input_config.adapter,
                params=input_config.params,
            )
        for output_config in self._runner_config.outputs:
            self._build_output(
                runner_name=output_config.runner_name,
                output_name=output_config.name,
                protocol=output_config.protocol,
                adapter=output_config.adapter,
                params=output_config.params,
                spec=output_config.spec,
            )
        for health_config in self._runner_config.healths:
            self._build_health(
                runner_name=health_config.runner_name,
                health_name=health_config.name,
                protocol=health_config.protocol,
                adapter=health_config.adapter,
                params=health_config.params,
            )

    def _build_input(
        self,
        *,
        runner_name: str,
        input_name: str,
        protocol: str,
        adapter: str,
        params: Dict[str, Any],
    ) -> None:
        connector = protocol
        params = dict(params)
        object_name = params["object_name"]
        channel = params["channel"]
        msg_size = int(params["msg_size"])
        channel_transport = str(params["channel_transport"])
        shared_data_buffer_size = _optional_int(params.get("shared_data_buffer_size"))
        shared_data_slot_count = _optional_int(params.get("shared_data_slot_count"))

        if connector not in ("module", "port", "lcm", "ros2"):
            raise NotImplementedError(f"Input connector is not implemented yet: {connector}")

        input_adapter = _resolve_adapter_class(adapter)(params)
        is_image = _is_protocol_image_input_adapter(adapter)
        payload_type = "image" if is_image else ("bytes" if connector == "port" else "json")
        runner = ConnectorInputRunner(
            self._connector_factory,
            connector,
            params,
            input_adapter=input_adapter,
            channel_name=channel,
            msg_size=msg_size,
            object_name=object_name,
            payload_type=payload_type,
            channel_transport=channel_transport,
            shared_data_buffer_size=shared_data_buffer_size,
            shared_data_slot_count=shared_data_slot_count,
            scheduler=self._scheduler if connector in ("module", "port") else None,
            component_name=f"{runner_name}.{input_name}",
        )

        self._runners.append(runner)

    def _build_output(
        self,
        *,
        runner_name: str,
        output_name: str,
        protocol: str,
        adapter: str,
        params: Dict[str, Any],
        spec: RunnerOutputSpec,
    ) -> None:
        connector = protocol
        params = dict(params)
        action_channel = params["action_channel"]
        action_msg_size = int(params["action_msg_size"])
        action_channel_transport = str(params["channel_transport"])
        consume_mode = str(params.get("action_consume_mode", "queue"))
        if connector not in ("module", "lcm", "ros2"):
            raise NotImplementedError(f"Output connector is not implemented yet: {connector}")
        target = ConnectorOutputTarget(
            self._connector_factory,
            connector,
            params,
            output_adapter=_resolve_adapter_class(adapter)(),
            output_config=params,
            channel_name=action_channel,
            msg_size=action_msg_size,
            channel_transport=action_channel_transport,
            component_name=f"{runner_name}.{output_name}",
            consume_mode=consume_mode,
        )
        self._runners.append(target)
        self.output_specs.append(clone_output_spec_with_target(spec, target))

    def _build_health(
        self,
        *,
        runner_name: str,
        health_name: str,
        protocol: str,
        adapter: str,
        params: Dict[str, Any],
    ) -> None:
        connector = protocol
        params = dict(params)
        object_name = params["object_name"]
        channel = params["channel"]
        msg_size = int(params["msg_size"])
        channel_transport = str(params["channel_transport"])

        if connector not in ("module", "port", "lcm", "ros2"):
            raise NotImplementedError(f"Health connector is not implemented yet: {connector}")

        runner = ConnectorInputRunner(
            self._connector_factory,
            connector,
            params,
            input_adapter=_resolve_adapter_class(adapter)(params),
            channel_name=channel,
            msg_size=msg_size,
            object_name=object_name,
            payload_type="json",
            channel_transport=channel_transport,
            shared_data_buffer_size=_optional_int(params.get("shared_data_buffer_size")),
            shared_data_slot_count=_optional_int(params.get("shared_data_slot_count")),
            scheduler=self._scheduler if connector in ("module", "port") else None,
            component_name=f"{runner_name}.{health_name}",
        )
        self._runners.append(runner)


def _optional_int(value: Any) -> int | None:
    if value is None:
        return None
    return int(value)


def _is_protocol_image_input_adapter(name: str) -> bool:
    from rynnrcp.adapters.protocol_image_input_adapter import ProtocolImageInputAdapter

    adapter_cls = _resolve_adapter_class(name)
    return isinstance(adapter_cls, type) and issubclass(adapter_cls, ProtocolImageInputAdapter)


@lru_cache(maxsize=64)
def _resolve_adapter_class(name: str) -> Any:
    if "." in name:
        return import_object(name)
    return import_object(f"rynnrcp.adapters.{camel_to_snake(name)}.{name}")

def _connector_node_name(robot_id: str, connector_names: set[str]) -> str:
    runner_part = "_".join(sorted(str(name) for name in connector_names)) or "runner"
    raw = f"rynnrcp_{robot_id}_{runner_part}_{os.getpid()}"
    return "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in raw)


def setup_runner_manager_process(channel_manager: Any, config: Dict[str, Any]):
    """ProcessNode component that runs one RunnerManager in a child process."""
    runtime_config = config["runtime_config"]
    logger.info(
        "Starting runner manager process for robot=%s runner_filter=%s",
        runtime_config.robot_id,
        config.get("runner_filter") or [],
    )
    runner_manager = RunnerManager(
        config=runtime_config,
        runner_filter=set(config.get("runner_filter") or []),
    )
    runner_manager.start()
    logger.info(
        "Runner manager process started for robot=%s runners=%s",
        runtime_config.robot_id,
        runner_manager.runner_names,
    )

    def cleanup() -> None:
        runner_manager.stop()

    return cleanup
