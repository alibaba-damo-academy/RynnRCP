"""PolicyService: run local policies through Observation and Action tools."""

from __future__ import annotations

import importlib.util
import logging
import sys
import threading
import time
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .base_service import BaseService
from rynnrcp.protocol.methods import LIST_POLICIES, START_POLICY, STOP_POLICY, UPDATE_POLICY_INPUTS
from rynnrcp.runtime.tool_bus import ToolBus

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class PolicySpec:
    policy_id: str
    path: Path
    entrypoint: str
    descriptor: dict[str, Any]
    observation_refs: list[dict[str, str]]
    observation_map: dict[str, str]
    runtime_inputs: dict[str, dict[str, Any]]
    action_refs: list[dict[str, str]]
    actions: list[str]


class PolicyService(BaseService):
    """Run one active policy loop against existing RCP protocol tools."""

    def __init__(
        self,
        bus: ToolBus,
        *,
        config: Mapping[str, Any],
        robot_root_dir: Path,
    ) -> None:
        super().__init__(bus, "policy_service")
        self._config = config
        self._robot_root_dir = robot_root_dir
        self._policies = _load_policies(config, robot_root_dir=robot_root_dir)
        self._lock = threading.RLock()
        self._active_policy_id: str | None = None
        self._active_inputs: dict[str, Any] = {}
        self._input_updated_at: dict[str, float] = {}
        self._stop_event: threading.Event | None = None
        self._thread: threading.Thread | None = None
        self._last_error = ""

    def bind(self) -> None:
        self._register_tool(
            LIST_POLICIES.name,
            self.list_policies,
            input_schema=LIST_POLICIES.input_schema,
            output_schema=LIST_POLICIES.output_schema,
            description=LIST_POLICIES.description,
        )
        self._register_tool(
            START_POLICY.name,
            self.start_policy,
            input_schema=START_POLICY.input_schema,
            output_schema=START_POLICY.output_schema,
            description=START_POLICY.description,
        )
        self._register_tool(
            UPDATE_POLICY_INPUTS.name,
            self.update_policy_inputs,
            input_schema=UPDATE_POLICY_INPUTS.input_schema,
            output_schema=UPDATE_POLICY_INPUTS.output_schema,
            description=UPDATE_POLICY_INPUTS.description,
        )
        self._register_tool(
            STOP_POLICY.name,
            self.stop_policy,
            input_schema=STOP_POLICY.input_schema,
            output_schema=STOP_POLICY.output_schema,
            description=STOP_POLICY.description,
        )

    def unbind(self) -> None:
        self.stop_policy(reason="service unbind")
        super().unbind()

    def list_policies(self) -> dict[str, Any]:
        with self._lock:
            active_policy_id = self._active_policy_id
            last_error = self._last_error
        return ToolBus.make_result(
            True,
            result={
                "policies": [_public_policy(policy) for policy in self._policies.values()],
                "active_policy_id": active_policy_id,
                "last_error": last_error,
            },
            message="OK",
        )

    def start_policy(
        self,
        policy_id: str | None = None,
        runtime_inputs: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        requested_id = str(policy_id or "").strip()
        if requested_id not in self._policies:
            return ToolBus.make_result(False, result={}, message=f"Unknown policy: {requested_id}")
        try:
            self._validate_protocol_objects(self._policies[requested_id])
        except Exception as exc:
            return ToolBus.make_result(False, result={}, message=str(exc))

        policy = self._policies[requested_id]
        active_inputs = _default_runtime_inputs(policy)
        now = time.time()
        if runtime_inputs is not None:
            if not isinstance(runtime_inputs, Mapping):
                return ToolBus.make_result(False, result={}, message="runtime_inputs must be a dict")
            unknown = _unknown_runtime_inputs(policy, runtime_inputs)
            if unknown:
                return ToolBus.make_result(False, result={}, message=f"Unknown runtime input(s): {', '.join(unknown)}")
            active_inputs.update(dict(runtime_inputs))
        try:
            policy_obj = _load_policy_object(policy)
            if hasattr(policy_obj, "load"):
                policy_obj.load(policy.path)
            if hasattr(policy_obj, "reset"):
                policy_obj.reset(dict(active_inputs))
        except Exception as exc:
            return ToolBus.make_result(False, result={}, message=str(exc))

        self.stop_policy(reason="switch policy")
        stop_event = threading.Event()
        thread = threading.Thread(
            target=self._policy_loop,
            args=(policy, policy_obj, stop_event),
            name=f"policy-{requested_id}",
            daemon=True,
        )
        with self._lock:
            self._active_policy_id = requested_id
            self._active_inputs = active_inputs
            self._input_updated_at = {key: now for key in active_inputs}
            self._stop_event = stop_event
            self._thread = thread
            self._last_error = ""
        thread.start()
        return ToolBus.make_result(True, result={"active_policy_id": requested_id}, message="OK")

    def update_policy_inputs(
        self,
        policy_id: str | None = None,
        runtime_inputs: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        if not isinstance(runtime_inputs, Mapping):
            return ToolBus.make_result(False, result={}, message="runtime_inputs must be a dict")
        with self._lock:
            active_policy_id = self._active_policy_id
            if active_policy_id is None:
                return ToolBus.make_result(False, result={}, message="No active policy")
            requested_id = str(policy_id or active_policy_id)
            if requested_id != active_policy_id:
                return ToolBus.make_result(False, result={}, message=f"Policy is not active: {requested_id}")
            unknown = _unknown_runtime_inputs(self._policies[active_policy_id], runtime_inputs)
            if unknown:
                return ToolBus.make_result(False, result={}, message=f"Unknown runtime input(s): {', '.join(unknown)}")
            now = time.time()
            self._active_inputs.update(dict(runtime_inputs))
            for key in runtime_inputs:
                self._input_updated_at[str(key)] = now
            result = {"active_policy_id": active_policy_id, "runtime_inputs": dict(self._active_inputs)}
        return ToolBus.make_result(True, result=result, message="OK")

    def stop_policy(self, policy_id: str | None = None, reason: str | None = None) -> dict[str, Any]:
        with self._lock:
            active_policy_id = self._active_policy_id
            if active_policy_id is None:
                return ToolBus.make_result(True, result={"stopped_policy_id": None}, message="OK")
            requested_id = str(policy_id or active_policy_id)
            if requested_id != active_policy_id:
                return ToolBus.make_result(False, result={}, message=f"Policy is not active: {requested_id}")
            stop_event = self._stop_event
            thread = self._thread
            self._active_policy_id = None
            self._stop_event = None
            self._thread = None
        if stop_event is not None:
            stop_event.set()
        if self._bus.has_tool("stop_action"):
            self._bus.call_tool("stop_action", reason=reason or "policy stopped")
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        return ToolBus.make_result(True, result={"stopped_policy_id": active_policy_id}, message="OK")

    def _policy_loop(self, policy: PolicySpec, policy_obj: Any, stop_event: threading.Event) -> None:
        while not stop_event.is_set():
            started = time.monotonic()
            try:
                obs = self._read_policy_observations(policy)
                obs.update(self._runtime_inputs(policy))
                chunk = _validate_policy_output(policy, policy_obj.step(obs))
                result = self._bus.call_tool(
                    "run_action_chunk",
                    name=chunk["name"],
                    frames=chunk["frames"],
                    frame_rate=chunk["frame_rate"],
                )
                if not bool(result.get("success")):
                    raise RuntimeError(str(result.get("message") or "run_action_chunk failed"))
                sleep_s = max(0.0, len(chunk["frames"]) / float(chunk["frame_rate"]) - (time.monotonic() - started))
                if sleep_s:
                    stop_event.wait(sleep_s)
            except Exception as exc:
                logger.warning("Policy %s tick failed: %s", policy.policy_id, exc, exc_info=True)
                with self._lock:
                    self._last_error = str(exc)
                stop_event.wait(0.1)

    def _read_policy_observations(self, policy: PolicySpec) -> dict[str, Any]:
        response = self._bus.call_tool("get_observations", names=list(policy.observation_map.values()))
        if not bool(response.get("success")):
            raise RuntimeError(str(response.get("message") or "get_observations failed"))
        observations = (response.get("result") or {}).get("observations")
        if not isinstance(observations, list):
            raise RuntimeError("get_observations returned invalid payload")
        by_name = {str(item.get("name")): item.get("value") for item in observations if isinstance(item, Mapping)}
        missing = sorted(name for name in policy.observation_map.values() if name not in by_name)
        if missing:
            raise RuntimeError(f"Missing observation(s): {', '.join(missing)}")
        return {local_key: by_name[remote_key] for local_key, remote_key in policy.observation_map.items()}

    def _runtime_inputs(self, policy: PolicySpec) -> dict[str, Any]:
        now = time.time()
        with self._lock:
            values = dict(self._active_inputs)
            updated_at = dict(self._input_updated_at)
        for key, spec in policy.runtime_inputs.items():
            stale_after_ms = spec.get("stale_after_ms")
            if stale_after_ms is None:
                continue
            if now - float(updated_at.get(key, 0.0)) > float(stale_after_ms) / 1000.0:
                values[key] = spec.get("default")
        return values

    def _validate_protocol_objects(self, policy: PolicySpec) -> None:
        observations = self._bus.call_tool("list_observations")
        actions = self._bus.call_tool("list_actions")
        observation_names = {
            str(item.get("name"))
            for item in (observations.get("result") or {}).get("observations", [])
            if isinstance(item, Mapping)
        }
        action_names = {
            str(item.get("name"))
            for item in (actions.get("result") or {}).get("actions", [])
            if isinstance(item, Mapping)
        }
        missing_observations = sorted(name for name in policy.observation_map.values() if name not in observation_names)
        missing_actions = sorted(name for name in policy.actions if name not in action_names)
        if missing_observations:
            raise ValueError(f"Policy {policy.policy_id} references unknown observation(s): {', '.join(missing_observations)}")
        if missing_actions:
            raise ValueError(f"Policy {policy.policy_id} references unknown action(s): {', '.join(missing_actions)}")


def _load_policies(config: Mapping[str, Any], *, robot_root_dir: Path) -> dict[str, PolicySpec]:
    policy_config = config.get("policies")
    if not isinstance(policy_config, Mapping):
        return {}
    paths = policy_config.get("paths") or []
    if not isinstance(paths, list):
        raise ValueError("policies.paths must be a list")
    policies: dict[str, PolicySpec] = {}
    for raw_path in paths:
        path = _resolve_policy_path(str(raw_path), config=config, robot_root_dir=robot_root_dir)
        for yaml_path in sorted(path.glob("*/policy.yaml")):
            policy = _load_policy_yaml(yaml_path)
            if policy.policy_id in policies:
                raise ValueError(f"Duplicate policy_id: {policy.policy_id}")
            policies[policy.policy_id] = policy
    return policies


def _resolve_policy_path(raw_path: str, *, config: Mapping[str, Any], robot_root_dir: Path) -> Path:
    path = Path(raw_path).expanduser()
    if path.is_absolute():
        return path
    config_dir = config.get("_config_dir")
    if isinstance(config_dir, str) and config_dir:
        return Path(config_dir).expanduser() / path
    return robot_root_dir / path


def _load_policy_yaml(path: Path) -> PolicySpec:
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, Mapping):
        raise ValueError(f"{path} must contain a YAML mapping")
    policy_id = str(raw.get("policy_id") or path.parent.name).strip()
    if not policy_id:
        raise ValueError(f"{path} missing policy_id")
    inputs = raw.get("inputs") if isinstance(raw.get("inputs"), Mapping) else {}
    outputs = raw.get("outputs") if isinstance(raw.get("outputs"), Mapping) else {}
    observations = inputs.get("observations") if isinstance(inputs, Mapping) else {}
    runtime_inputs = inputs.get("runtime_inputs") if isinstance(inputs, Mapping) else {}
    if runtime_inputs is None:
        runtime_inputs = {}
    actions = outputs.get("actions") if isinstance(outputs, Mapping) else []
    observation_refs = _resolve_refs(observations, "observation", f"{path} inputs.observations")
    action_refs = _resolve_refs(actions, "action", f"{path} outputs.actions")
    observation_map = {item["key"]: item["key"] for item in observation_refs}
    if not isinstance(runtime_inputs, Mapping):
        raise ValueError(f"{path} inputs.runtime_inputs must be a dict")
    if not action_refs:
        raise ValueError(f"{path} outputs.actions is required")
    return PolicySpec(
        policy_id=policy_id,
        path=path.parent,
        entrypoint=str(raw.get("entrypoint") or "policy.py:Policy"),
        descriptor=dict(raw),
        observation_refs=observation_refs,
        observation_map=observation_map,
        runtime_inputs=_normalize_runtime_input_specs(runtime_inputs, f"{path} inputs.runtime_inputs"),
        action_refs=action_refs,
        actions=[item["key"] for item in action_refs],
    )


def _public_policy(policy: PolicySpec) -> dict[str, Any]:
    return {
        "policy_id": policy.policy_id,
        "name": str(policy.descriptor.get("name") or policy.policy_id),
        "description": str(policy.descriptor.get("description") or ""),
        "inputs": {
            "observations": list(policy.observation_refs),
            "runtime_inputs": dict(policy.runtime_inputs),
        },
        "outputs": {"actions": list(policy.action_refs)},
    }


def _resolve_refs(value: Any, prefix: str, context: str) -> list[dict[str, str]]:
    if not isinstance(value, list):
        raise ValueError(f"{context} must be a list")
    refs: list[dict[str, str]] = []
    for index, item in enumerate(value):
        item_context = f"{context}[{index}]"
        if not isinstance(item, Mapping):
            raise ValueError(f"{item_context} must be a dict")
        component = str(item.get("component") or "").strip()
        name = str(item.get("name") or "").strip()
        if not component or not name:
            raise ValueError(f"{item_context} requires component and name")
        refs.append({"component": component, "name": name, "key": f"{prefix}.{component}.{name}"})
    return refs


def _default_runtime_inputs(policy: PolicySpec) -> dict[str, Any]:
    return {key: spec.get("default") for key, spec in policy.runtime_inputs.items()}


def _unknown_runtime_inputs(policy: PolicySpec, runtime_inputs: Mapping[str, Any]) -> list[str]:
    return sorted(str(key) for key in runtime_inputs if str(key) not in policy.runtime_inputs)


def _normalize_runtime_input_specs(value: Mapping[str, Any], context: str) -> dict[str, dict[str, Any]]:
    specs: dict[str, dict[str, Any]] = {}
    for key, spec in value.items():
        if not isinstance(spec, Mapping):
            raise ValueError(f"{context}.{key} must be a dict")
        specs[str(key)] = dict(spec)
    return specs


def _load_policy_object(policy: PolicySpec) -> Any:
    module_name, sep, symbol_name = policy.entrypoint.partition(":")
    if not sep:
        raise ValueError(f"Policy {policy.policy_id} entrypoint must look like policy.py:Policy")
    module_path = policy.path / module_name
    if not module_path.exists():
        raise FileNotFoundError(f"Policy entrypoint not found: {module_path}")
    unique_name = f"_rynnrcp_policy_{policy.policy_id}_{abs(hash(str(module_path)))}"
    spec = importlib.util.spec_from_file_location(unique_name, module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load policy module: {module_path}")
    module = importlib.util.module_from_spec(spec)
    old_path = list(sys.path)
    sys.path.insert(0, str(policy.path))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path[:] = old_path
    symbol = getattr(module, symbol_name)
    return symbol() if isinstance(symbol, type) else symbol


def _validate_policy_output(policy: PolicySpec, output: Any) -> dict[str, Any]:
    if not isinstance(output, Mapping):
        raise TypeError("policy.step must return a dict")
    name = str(output.get("name") or "")
    if name not in policy.actions:
        raise ValueError(f"Policy output action {name!r} is not declared in outputs.actions")
    frames = output.get("frames")
    if not isinstance(frames, list) or not frames:
        raise ValueError("Policy output frames must be a non-empty list")
    for index, frame in enumerate(frames):
        if not isinstance(frame, Mapping):
            raise ValueError(f"Policy output frames[{index}] must be an object")
    try:
        frame_rate = float(output.get("frame_rate"))
    except (TypeError, ValueError) as exc:
        raise ValueError("Policy output frame_rate must be int or float") from exc
    if frame_rate <= 0:
        raise ValueError("Policy output frame_rate must be greater than 0")
    return {"name": name, "frames": list(frames), "frame_rate": frame_rate}
