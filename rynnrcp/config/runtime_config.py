"""Runtime configuration context for RynnRCP server startup."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import os
from pathlib import Path
import re
from typing import Any, Dict, Mapping

from rynnrcp.config.loader import load_config
from rynnrcp.config.resolver import (
    load_core_config,
    load_integration_config,
    require_mapping,
    require_str,
    resolve_refs,
)
from rynnrcp.config.validator import ConfigValidator
from rynnrcp.utils.user_paths import (
    collections_dir,
    new_log_session_id,
    robot_root_from_config,
    resolve_robot_path,
)

_BOOL_REF_PATTERN = re.compile(r"\$\{([^}]+)\}")


@dataclass(frozen=True)
class RuntimeConfig:
    """Loaded and validated config set used to build Runtime modules."""

    server_config: Dict[str, Any]
    core_config: Dict[str, Any]
    integration_config: Dict[str, Any]
    runtime_options: Dict[str, Any]
    runtime_context: Dict[str, Any]
    robot_id: str
    runner_mode: str
    channel_transport: str
    channel_registry_name: str
    startup_timeout_s: float
    collection_root_dir: str
    log_session_id: str

    @classmethod
    def load(cls, path: str, *, log_session_id: str | None = None) -> "RuntimeConfig":
        return cls.from_mapping(
            load_config(path),
            log_session_id=log_session_id,
            config_dir=str(Path(path).expanduser().resolve().parent),
        )

    @classmethod
    def from_mapping(
        cls,
        config: Mapping[str, Any],
        *,
        log_session_id: str | None = None,
        config_dir: str | None = None,
    ) -> "RuntimeConfig":
        server_config = deepcopy(dict(config))
        if server_config.get("config_type") != "rynnrcp_server_config":
            raise ValueError("Runtime requires a rynnrcp_server_config")
        ConfigValidator.validate_source(server_config)

        core_config = load_core_config()
        integration_config = load_integration_config(server_config)
        robot_id = require_str(require_mapping(server_config, "manifest"), "robot_id")
        runtime_options = _runtime_options(core_config, server_config)
        runner_mode = require_str(runtime_options, "runner_mode")
        runtime_context = _runtime_context(
            server_config,
            integration_config=integration_config,
            config_dir=config_dir,
        )

        return cls(
            server_config=server_config,
            core_config=core_config,
            integration_config=integration_config,
            runtime_options=runtime_options,
            runtime_context=runtime_context,
            robot_id=robot_id,
            runner_mode=runner_mode,
            channel_transport=_channel_transport(core_config, runner_mode),
            channel_registry_name=_channel_registry_name(runtime_options, robot_id),
            startup_timeout_s=_startup_timeout(runtime_options),
            collection_root_dir=str(_collection_root_dir(runtime_context)),
            log_session_id=str(log_session_id or new_log_session_id()),
        )

    @property
    def connectors_config(self) -> Dict[str, Any]:
        return dict(self.server_config.get("connectors") or {})


def _runtime_context(
    server_config: Mapping[str, Any],
    *,
    integration_config: Mapping[str, Any],
    config_dir: str | None = None,
) -> Dict[str, Any]:
    context = deepcopy(dict(server_config))
    server = dict(context.get("server") or {})
    manifest = _runtime_manifest(context, integration_config)
    server_id = str(manifest.get("robot_id") or "").strip()
    server["config_name"] = str(server.get("config_name") or server_id).strip()
    context["manifest"] = manifest
    context["server"] = server
    if config_dir:
        context["_config_dir"] = str(config_dir)
    return context


def _runtime_manifest(
    server_config: Mapping[str, Any],
    integration_config: Mapping[str, Any],
) -> Dict[str, Any]:
    integration_manifest = require_mapping(integration_config, "manifest")
    server_manifest = require_mapping(server_config, "manifest")
    active_components = _active_components(server_config, integration_config)
    manifest = deepcopy(dict(integration_manifest))
    if "embodiment_type" in manifest:
        manifest["embodiment_type"] = resolve_refs(manifest["embodiment_type"], server_config)
    if "metadata" in manifest:
        manifest["metadata"] = resolve_refs(manifest["metadata"], server_config)
    manifest.update(deepcopy(dict(server_manifest)))
    manifest["components"] = _select_components(
        integration_manifest,
        integration_config=integration_config,
        active_components=active_components,
        server_config=server_config,
    )
    return manifest


def _active_components(
    server_config: Mapping[str, Any],
    integration_config: Mapping[str, Any],
) -> set[str]:
    components = _iter_component_definitions(require_mapping_or_list(integration_config, "components"))
    active: set[str] = set()
    for component_name, component in components:
        if _component_enabled(server_config, str(component_name), component):
            active.add(str(component_name))
    if not active:
        raise ValueError("at least one component must be enabled")
    return active


def _component_enabled(
    server_config: Mapping[str, Any],
    component_name: str,
    component: Mapping[str, Any],
) -> bool:
    try:
        return resolve_config_bool(component["enabled"], server_config)
    except KeyError as exc:
        raise ValueError(f"components.{component_name}.enabled cannot be resolved") from exc
    except TypeError as exc:
        raise ValueError(f"components.{component_name}.enabled must resolve to a boolean") from exc


def resolve_config_bool(value: Any, context: Mapping[str, Any]) -> bool:
    if isinstance(value, bool):
        return value
    if not isinstance(value, str):
        raise TypeError("expected a boolean or ${...} reference")
    match = _BOOL_REF_PATTERN.fullmatch(value)
    if not match:
        raise TypeError("expected a boolean or ${...} reference")
    resolved = _lookup_config_path(context, match.group(1))
    if not isinstance(resolved, bool):
        raise TypeError(f"${{{match.group(1)}}} must resolve to a boolean")
    return resolved


def _lookup_config_path(context: Mapping[str, Any], path: str) -> Any:
    cur: Any = context
    for part in path.split("."):
        if isinstance(cur, Mapping) and part in cur:
            cur = cur[part]
        else:
            raise KeyError(f"Missing config value for ${{{path}}}")
    return cur


def _select_components(
    integration_manifest: Mapping[str, Any],
    *,
    integration_config: Mapping[str, Any],
    active_components: set[str],
    server_config: Mapping[str, Any],
) -> list[Dict[str, Any]]:
    raw_components = integration_manifest.get("components")
    if raw_components is None:
        raw_components = _derive_components(integration_config)
    if not isinstance(raw_components, list):
        raise ValueError("manifest.components must be defined in robot integration config")
    by_name = {
        str(component.get("name")): dict(component)
        for component in raw_components
        if isinstance(component, Mapping) and component.get("name")
    }
    missing = sorted(name for name in active_components if name not in by_name)
    if missing:
        raise ValueError(f"enabled interface references unknown component(s): {', '.join(missing)}")
    selected: list[Dict[str, Any]] = []
    for component in raw_components:
        if (
            not isinstance(component, Mapping)
            or component.get("name") not in active_components
        ):
            continue
        resolved = dict(resolve_refs(component, server_config))
        dof = resolved.get("dof")
        if dof is not None and (
            not isinstance(dof, int) or isinstance(dof, bool) or dof <= 0
        ):
            raise ValueError(
                f"components.{resolved.get('name', 'unknown')}.dof "
                "must resolve to a positive integer"
            )
        selected.append(resolved)
    return selected


def _iter_component_definitions(
    raw_components: Mapping[str, Any] | list[Any]
) -> list[tuple[str, Dict[str, Any]]]:
    if isinstance(raw_components, Mapping):
        components: list[tuple[str, Dict[str, Any]]] = []
        names: set[str] = set()
        for name, component in raw_components.items():
            if not isinstance(name, str) or not name.strip():
                raise ValueError("components keys must be non-empty strings")
            if not isinstance(component, Mapping):
                raise ValueError(f"components.{name} must be a dict")
            component_name = str(name)
            if component_name in names:
                raise ValueError(f"components has duplicate name {component_name!r}")
            names.add(component_name)
            components.append((component_name, dict(component)))
        return components
    if not isinstance(raw_components, list):
        raise ValueError("components must be a dict or list")
    components: list[tuple[str, Dict[str, Any]]] = []
    seen: set[str] = set()
    for index, component in enumerate(raw_components):
        if not isinstance(component, Mapping):
            raise ValueError(f"components[{index}] must be a dict")
        name = component.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ValueError(f"components[{index}].name is required")
        component_name = str(name)
        if component_name in seen:
            raise ValueError(f"components has duplicate name {component_name!r}")
        seen.add(component_name)
        components.append((component_name, dict(component)))
    return components


def require_mapping_or_list(config: Mapping[str, Any], key: str) -> Mapping[str, Any] | list[Any]:
    value = config.get(key)
    if not isinstance(value, (Mapping, list)):
        raise ValueError(f"{key} must be a dict or list")
    return value


def _derive_components(integration_config: Mapping[str, Any]) -> list[Dict[str, Any]]:
    component_defs = _iter_component_definitions(require_mapping_or_list(integration_config, "components"))
    components = []
    for _, component in component_defs:
        manifest_component = {str(key): value for key, value in component.items() if key in {"name", "type", "parent_component", "dof", "frame", "description"}}
        if "name" in manifest_component:
            components.append(manifest_component)
    return components


def _runtime_options(core_config: Mapping[str, Any], server_config: Mapping[str, Any]) -> Dict[str, Any]:
    runtime = dict(require_mapping(core_config, "runtime"))
    runtime.update(dict(server_config.get("runtime") or {}))
    runner_mode = str(runtime.get("runner_mode") or "")
    if runner_mode not in ("thread", "process"):
        raise ValueError("runtime.runner_mode must be 'thread' or 'process'")
    return runtime


def _channel_transport(core_config: Mapping[str, Any], runner_mode: str) -> str:
    ipc = require_mapping(core_config, "ipc")
    if runner_mode == "process":
        return str(ipc.get("process_transport") or "shm")
    return str(ipc.get("thread_transport") or "memory")


def _channel_registry_name(runtime_options: Mapping[str, Any], robot_id: str) -> str:
    explicit = runtime_options.get("channel_registry_name")
    if explicit:
        return str(explicit)
    return f"rynnrcp_channel_registry_{robot_id}_{os.getpid()}"


def _startup_timeout(runtime_options: Mapping[str, Any]) -> float:
    value = runtime_options.get("startup_timeout_s")
    if value is None:
        return 30.0
    try:
        timeout = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("runtime.startup_timeout_s must be a number") from exc
    if timeout <= 0:
        raise ValueError("runtime.startup_timeout_s must be greater than 0")
    return timeout


def _collection_root_dir(config: Mapping[str, Any]) -> Any:
    root = robot_root_from_config(config)
    server = config.get("server") or {}
    if isinstance(server, dict):
        server_storage = server.get("storage") or {}
        if isinstance(server_storage, dict) and server_storage.get("collection_dir"):
            return resolve_robot_path(str(server_storage["collection_dir"]), root)
    storage = config.get("storage") or {}
    if isinstance(storage, dict) and storage.get("collection_dir"):
        return resolve_robot_path(str(storage["collection_dir"]), root)
    return collections_dir(root, "manual")
