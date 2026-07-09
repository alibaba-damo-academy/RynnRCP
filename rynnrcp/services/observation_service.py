"""ObservationService: expose runner inputs as RCP Observation objects."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Dict, Iterable

from .base_service import BaseService
from rynnrcp.config.runner_config import RunnerInputSpec
from rynnrcp.protocol.methods import GET_OBSERVATIONS, LIST_OBSERVATIONS
from rynnrcp.protocol.observation_codecs import observation_value_schema, protocol_observation_value
from rynnrcp.runtime.tool_bus import ToolBus
from rynnrcp.utils.payload import close_shared_reader_cache, parse_channel_payload
from rynnrcp.utils.shared_data_store import SharedDataExpired


class ObservationService(BaseService):
    """Read latest runner Channels and expose them through the RCP Observation API."""

    def __init__(self, bus: ToolBus, inputs: Iterable[RunnerInputSpec]) -> None:
        super().__init__(bus, "observation_service")
        self._inputs = list(inputs)
        self._observations = _build_observation_descriptors(self._inputs)
        self._input_by_key = {
            str(descriptor["name"]): spec
            for spec, descriptor in zip(self._inputs, self._observations)
        }
        self._type_by_key = {
            str(descriptor["name"]): str(descriptor["type"])
            for descriptor in self._observations
        }
        self._available_keys = sorted(self._input_by_key)
        self._latest_value_cache: Dict[str, tuple[float, Any]] = {}
        self._latest_payload_cache: Dict[str, tuple[float, bytes, Any]] = {}
        self._shared_reader_cache: Dict[str, Any] = {}
        for spec in self._inputs:
            self.subscribe_channel(spec.channel, spec.msg_size, spec.channel_transport)

    def bind(self) -> None:
        self._register_tool(
            LIST_OBSERVATIONS.name,
            self.list_observations,
            input_schema=LIST_OBSERVATIONS.input_schema,
            output_schema=LIST_OBSERVATIONS.output_schema,
            description=LIST_OBSERVATIONS.description,
        )
        self._register_tool(
            GET_OBSERVATIONS.name,
            self.get_observations,
            input_schema=GET_OBSERVATIONS.input_schema,
            output_schema=GET_OBSERVATIONS.output_schema,
            description=GET_OBSERVATIONS.description,
        )

    def unbind(self) -> None:
        close_shared_reader_cache(self._shared_reader_cache)
        super().unbind()

    def list_observations(self) -> Dict[str, Any]:
        return ToolBus.make_result(True, result={"observations": list(self._observations)}, message="OK")

    def get_observations(self, names: list[Any] | None = None, sync: bool = False) -> Dict[str, Any]:
        if sync:
            # TODO: Implement time-aligned multi-observation reads after the runner
            # exposes synchronized sample snapshots. Current channels store latest values.
            return ToolBus.make_result(False, result={"observations": []}, message="sync observations are not implemented")
        try:
            requested_keys, available_keys, unknown_keys = self._resolve_request(names)
        except ValueError as exc:
            return ToolBus.make_result(False, result={"observations": []}, message=str(exc))

        if unknown_keys:
            return ToolBus.make_result(
                False,
                result={
                    "unknown_keys": unknown_keys,
                    "available_observations": available_keys,
                },
                message=f"Unknown observation(s): {', '.join(unknown_keys)}",
            )

        observations: list[Dict[str, Any]] = []

        for key in requested_keys:
            try:
                ts, value = self._read_observation(key)
            except (KeyError, TypeError, ValueError) as exc:
                return ToolBus.make_result(False, result={"observations": []}, message=str(exc))
            if value is not None:
                observation = {
                    "name": key,
                    "timestamp": float(ts) if ts is not None else 0.0,
                    "value": value,
                }
                observations.append(observation)

        if not observations:
            return ToolBus.make_result(
                False,
                result={
                    "observations": [],
                    "requested_observations": requested_keys,
                    "available_observations": available_keys,
                },
                message="No observation data available",
            )

        return ToolBus.make_result(True, result={"observations": observations}, message="OK")

    def _resolve_request(
        self,
        keys: list[Any] | None,
    ) -> tuple[list[str], list[str], list[str]]:
        if keys is None:
            raise ValueError("names is required")
        if not isinstance(keys, list) or not all(isinstance(key, str) for key in keys):
            raise ValueError("names must be a list of Observation names")
        requested_keys = [str(key) for key in keys]
        unknown_keys = sorted(key for key in requested_keys if key not in self._input_by_key)
        return requested_keys, list(self._available_keys), unknown_keys

    def _parse_payload(self, spec: RunnerInputSpec, ts: float, payload: bytes) -> Any:
        cached = self._latest_payload_cache.get(spec.object_name)
        if cached is not None and cached[0] == ts and cached[1] == payload:
            return cached[2]
        value = parse_channel_payload(spec, payload, reader_cache=self._shared_reader_cache)
        self._latest_payload_cache[spec.object_name] = (ts, payload, value)
        return value

    def _read_observation(self, key: str) -> tuple[float | None, Any]:
        spec = self._input_by_key[key]
        ts, payload = self.latest_parsed(spec.channel)
        if ts is None or payload is None:
            return self._latest_value_cache.get(key, (None, None))
        try:
            value = self._parse_payload(spec, float(ts), payload)
        except SharedDataExpired:
            return self._latest_value_cache.get(key, (None, None))
        value = protocol_observation_value(self._type_by_key[key], value)
        self._latest_value_cache[key] = (float(ts), value)
        return self._latest_value_cache[key]


def _build_observation_descriptors(inputs: Iterable[RunnerInputSpec]) -> list[Dict[str, Any]]:
    return [_observation_descriptor(spec) for spec in inputs]


def _observation_descriptor(spec: RunnerInputSpec) -> Dict[str, Any]:
    info = spec.info if isinstance(spec.info, Mapping) else {}
    observation_type = _observation_type(spec)
    descriptor: Dict[str, Any] = {
        "name": _protocol_observation_name(spec, observation_type),
        "type": observation_type,
        "value_schema": info.get("value_schema") or observation_value_schema(observation_type, info),
    }
    description = info.get("description")
    if description:
        descriptor["description"] = str(description)
    component_name = info.get("component_name")
    if component_name:
        descriptor["component_name"] = str(component_name)
    frame_rate = info.get("frame_rate")
    if frame_rate is not None:
        descriptor["frame_rate"] = float(frame_rate)
    return descriptor


def _observation_type(spec: RunnerInputSpec) -> str:
    info = spec.info if isinstance(spec.info, Mapping) else {}
    return str(info["rcp_observation_type"])


def _protocol_observation_name(spec: RunnerInputSpec, observation_type: str) -> str:
    info = spec.info if isinstance(spec.info, Mapping) else {}
    return str(info["rcp_observation_name"])
