"""HealthService: expose RCP health issues."""

from __future__ import annotations

import time
from typing import Any, Dict, Iterable, Mapping

from .base_service import BaseService
from rynnrcp.config.runner_config import RunnerHealthSpec, RunnerInputSpec
from rynnrcp.protocol.methods import GET_HEALTH
from rynnrcp.runtime.tool_bus import ToolBus
from rynnrcp.utils.payload import close_shared_reader_cache, parse_channel_payload


class HealthService(BaseService):
    """Report current robot/runtime health issues in protocol shape."""

    def __init__(
        self,
        bus: ToolBus,
        *,
        inputs: Iterable[RunnerInputSpec],
        healths: Iterable[RunnerHealthSpec] = (),
        stale_after_s: float = 2.0,
    ) -> None:
        super().__init__(bus, "health_service")
        self._inputs = list(inputs)
        self._healths = list(healths)
        self._stale_after_s = float(stale_after_s)
        self._shared_reader_cache: Dict[str, Any] = {}
        for spec in self._inputs:
            self.subscribe_channel(spec.channel, spec.msg_size, spec.channel_transport)
        for spec in self._healths:
            self.subscribe_channel(spec.channel, spec.msg_size, spec.channel_transport)

    def bind(self) -> None:
        self._register_tool(
            GET_HEALTH.name,
            self.get_health,
            input_schema=GET_HEALTH.input_schema,
            output_schema=GET_HEALTH.output_schema,
            description=GET_HEALTH.description,
        )

    def unbind(self) -> None:
        close_shared_reader_cache(self._shared_reader_cache)
        super().unbind()

    def get_health(self) -> Dict[str, Any]:
        now = time.time()
        messages: list[Dict[str, Any]] = []

        for spec in self._healths:
            ts, payload = self.latest_parsed(spec.channel)
            if ts is None or payload is None:
                continue
            try:
                value = parse_channel_payload(spec, payload, reader_cache=self._shared_reader_cache)
            except Exception as exc:
                messages.append(
                    _health_message(
                        "warning",
                        "health.invalid_payload",
                        f"Health source {spec.name} returned invalid payload: {exc}",
                        source=_diagnostic_component_name(spec),
                        timestamp=float(ts),
                    )
                )
                continue
            messages.extend(_normalize_health_payload(
                value,
                source=_diagnostic_component_name(spec),
                timestamp=float(ts),
            ))

        for spec in self._inputs:
            observation_name = _diagnostic_observation_name(spec)
            ts, _payload = self.latest_parsed(spec.channel)
            if ts is None:
                messages.append(
                    _health_message(
                        "warning",
                        "observation.no_data",
                        f"No data has been received for {observation_name}",
                        source=spec.runner_name,
                        timestamp=now,
                    )
                )
                continue
            age_s = max(0.0, now - float(ts))
            if age_s > self._stale_after_s:
                messages.append(
                    _health_message(
                        "warning",
                        "observation.stale",
                        f"Observation {observation_name} is stale ({age_s:.3f}s)",
                        source=spec.runner_name,
                        timestamp=float(ts),
                    )
                )

        return ToolBus.make_result(True, result={"msg": messages}, message="OK")


def _health_message(level: str, code: str, message: str, *, source: str, timestamp: float, details: Any = None) -> Dict[str, Any]:
    item = {
        "level": str(level),
        "code": code,
        "message": message,
        "source": source,
        "timestamp": float(timestamp),
    }
    if details is not None:
        item["details"] = details
    return item


def _diagnostic_observation_name(spec: RunnerInputSpec) -> str:
    info = spec.info if isinstance(spec.info, dict) else {}
    return str(info.get("rcp_observation_name") or spec.name)


def _diagnostic_component_name(spec: RunnerHealthSpec) -> str:
    info = spec.info if isinstance(spec.info, dict) else {}
    return str(info.get("component_name") or spec.runner_name)


def _normalize_health_payload(
    value: Any,
    *,
    source: str,
    timestamp: float,
) -> list[Dict[str, Any]]:
    if value is None:
        return []
    if not isinstance(value, Mapping):
        return [
            _health_message(
                "error",
                "health.invalid_payload",
                f"Health source {source} must return a dict with errors/warnings",
                source=source,
                timestamp=timestamp,
            )
        ]
    messages: list[Dict[str, Any]] = []
    for item in _issue_list(value, "errors"):
        messages.append(_normalize_issue(item, default_level="error", default_source=source, default_timestamp=timestamp))
    for item in _issue_list(value, "warnings"):
        messages.append(_normalize_issue(item, default_level="warning", default_source=source, default_timestamp=timestamp))
    for item in _issue_list(value, "msg"):
        messages.append(_normalize_issue(item, default_level="info", default_source=source, default_timestamp=timestamp))
    return messages


def _issue_list(value: Mapping[str, Any], key: str) -> list[Any]:
    raw = value.get(key) or []
    if isinstance(raw, list):
        return raw
    return [
        {
            "code": "health.invalid_payload",
            "message": f"health.{key} must be a list",
        }
    ]


def _normalize_issue(
    value: Any,
    *,
    default_level: str,
    default_source: str,
    default_timestamp: float,
) -> Dict[str, Any]:
    if isinstance(value, Mapping):
        issue = dict(value)
    else:
        issue = {
            "code": "health.issue",
            "message": str(value),
        }
    issue["level"] = str(issue.get("level") or default_level)
    issue["code"] = str(issue.get("code") or "health.issue")
    issue["message"] = str(issue.get("message") or issue["code"])
    issue["source"] = str(issue.get("source") or default_source)
    issue["timestamp"] = float(issue.get("timestamp") or default_timestamp)
    return issue
