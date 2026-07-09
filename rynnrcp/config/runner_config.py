"""Build runner-ready config objects from validated RynnRCP configs."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping

from rynnrcp.config.resolver import require_mapping, require_number, require_str, resolve_refs
from rynnrcp.config.runtime_config import RuntimeConfig, resolve_config_bool


@dataclass(frozen=True)
class RunnerInputSpec:
    """Channel metadata for one configured input runner."""

    name: str
    runner_name: str
    protocol: str
    adapter: str
    channel: str
    msg_size: int
    object_name: str
    channel_transport: str = "shm"
    payload_type: str = "json"
    info: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RunnerOutputSpec:
    """Action channel metadata for one configured output."""

    name: str
    runner_name: str
    protocol: str
    adapter: str
    params: Dict[str, Any]
    channel: str
    msg_size: int
    channel_transport: str = "shm"
    target: Any = None


@dataclass(frozen=True)
class RunnerHealthSpec:
    """Channel metadata for one configured component health source."""

    name: str
    runner_name: str
    protocol: str
    channel: str
    msg_size: int
    object_name: str
    channel_transport: str = "shm"
    info: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RunnerInputConfig:
    runner_name: str
    name: str
    protocol: str
    adapter: str
    params: Dict[str, Any]
    specs: list[RunnerInputSpec]


@dataclass(frozen=True)
class RunnerOutputConfig:
    runner_name: str
    name: str
    protocol: str
    adapter: str
    params: Dict[str, Any]
    spec: RunnerOutputSpec


@dataclass(frozen=True)
class RunnerHealthConfig:
    runner_name: str
    name: str
    protocol: str
    adapter: str
    params: Dict[str, Any]
    spec: RunnerHealthSpec


@dataclass(frozen=True)
class RunnerConfig:
    runner_names: list[str]
    connector_names: set[str]
    capabilities: dict[str, bool]
    inputs: list[RunnerInputConfig]
    outputs: list[RunnerOutputConfig]
    healths: list[RunnerHealthConfig]
    input_specs: list[RunnerInputSpec]
    output_specs: list[RunnerOutputSpec]
    health_specs: list[RunnerHealthSpec]


def build_runner_config(config: RuntimeConfig, runner_filter: set[str] | None = None) -> RunnerConfig:
    runner_filter = set(runner_filter or set())
    inputs: list[RunnerInputConfig] = []
    outputs: list[RunnerOutputConfig] = []
    healths: list[RunnerHealthConfig] = []
    input_specs: list[RunnerInputSpec] = []
    output_specs: list[RunnerOutputSpec] = []
    health_specs: list[RunnerHealthSpec] = []
    runner_names: list[str] = []
    connector_names: set[str] = set()

    for interface_name, raw_component, runner_name in _configured_components(
        config.runtime_context,
        config.integration_config,
        runner_filter,
    ):
        if runner_name not in runner_names:
            runner_names.append(runner_name)
        for input_cfg in _io_items(raw_component.get("observations") or [], "observations"):
            name = _protocol_object_name("observation", str(interface_name), input_cfg)
            source = require_mapping(input_cfg, "source")
            codec = require_mapping(input_cfg, "codec")
            protocol = require_str(source, "connector")
            connector_names.add(protocol)
            adapter = require_str(codec, "adapter")
            params = _input_params(
                server_config=config.runtime_context,
                core_config=config.core_config,
                channel_transport=config.channel_transport,
                component_name=str(interface_name),
                input_cfg=input_cfg,
            )
            specs = _input_specs(
                runner_name=runner_name,
                input_name=name,
                protocol=protocol,
                adapter=adapter,
                params=params,
            )
            inputs.append(
                RunnerInputConfig(
                    runner_name=runner_name,
                    name=name,
                    protocol=protocol,
                    adapter=adapter,
                    params=params,
                    specs=specs,
                )
            )
            input_specs.extend(specs)

        for output_cfg in _io_items(raw_component.get("actions") or [], "actions"):
            if not _capability_enabled(config.runtime_context, "actions"):
                continue
            name = _protocol_object_name("action", str(interface_name), output_cfg)
            source = require_mapping(output_cfg, "source")
            codec = require_mapping(output_cfg, "codec")
            protocol = require_str(source, "connector")
            connector_names.add(protocol)
            adapter = require_str(codec, "adapter")
            params = _output_params(
                server_config=config.runtime_context,
                core_config=config.core_config,
                channel_transport=config.channel_transport,
                component_name=str(interface_name),
                output_cfg=output_cfg,
            )
            spec = RunnerOutputSpec(
                name=name,
                runner_name=runner_name,
                protocol=protocol,
                adapter=adapter,
                params=params,
                channel=params["action_channel"],
                msg_size=int(params["action_msg_size"]),
                channel_transport=str(params["channel_transport"]),
            )
            outputs.append(
                RunnerOutputConfig(
                    runner_name=runner_name,
                    name=name,
                    protocol=protocol,
                    adapter=adapter,
                    params=params,
                    spec=spec,
                )
            )
            output_specs.append(spec)

        if _capability_enabled(config.runtime_context, "health") and raw_component.get("health"):
            health_cfg = require_mapping(raw_component, "health")
            source = require_mapping(health_cfg, "source")
            protocol = require_str(source, "connector")
            connector_names.add(protocol)
            adapter = "ProtocolInputAdapter"
            params = _health_params(
                server_config=config.runtime_context,
                core_config=config.core_config,
                channel_transport=config.channel_transport,
                component_name=str(interface_name),
                health_cfg=health_cfg,
            )
            spec = RunnerHealthSpec(
                name=str(params["health_name"]),
                runner_name=runner_name,
                protocol=protocol,
                channel=str(params["channel"]),
                msg_size=int(params["msg_size"]),
                object_name=str(params["object_name"]),
                channel_transport=str(params["channel_transport"]),
                info=_health_info(params),
            )
            healths.append(
                RunnerHealthConfig(
                    runner_name=runner_name,
                    name=spec.name,
                    protocol=protocol,
                    adapter=adapter,
                    params=params,
                    spec=spec,
                )
            )
            health_specs.append(spec)

    return RunnerConfig(
        runner_names=runner_names,
        connector_names=connector_names,
        capabilities=_configured_capabilities(config.runtime_context),
        inputs=inputs,
        outputs=outputs,
        healths=healths,
        input_specs=input_specs,
        output_specs=output_specs,
        health_specs=health_specs,
    )


def _configured_components(
    server_config: Mapping[str, Any],
    integration_config: Mapping[str, Any],
    runner_filter: set[str],
):
    components = _iter_component_definitions(_components_from_runtime(integration_config, "components"))
    for component_name, component in components:
        if not isinstance(component, Mapping):
            continue
        component_name = str(component_name)
        if not _component_enabled(server_config, component_name, component):
            continue
        runner_name = component_name
        if runner_filter and runner_name not in runner_filter:
            continue
        yield component_name, component, runner_name


def _component_enabled(server_config: Mapping[str, Any], component_name: str, component: Mapping[str, Any]) -> bool:
    return resolve_config_bool(component["enabled"], server_config)


def _capability_enabled(config: Mapping[str, Any], name: str) -> bool:
    manifest = config.get("manifest") if isinstance(config, Mapping) else {}
    capabilities = manifest.get("capabilities") if isinstance(manifest, Mapping) else {}
    return bool(capabilities.get(name)) if isinstance(capabilities, Mapping) else False


def _configured_capabilities(config: Mapping[str, Any]) -> dict[str, bool]:
    return {str(key): bool(value) for key, value in require_mapping(require_mapping(config, "manifest"), "capabilities").items()}


def _input_params(
    *,
    server_config: Mapping[str, Any],
    core_config: Mapping[str, Any],
    channel_transport: str,
    component_name: str,
    input_cfg: Mapping[str, Any],
) -> Dict[str, Any]:
    source = require_mapping(input_cfg, "source")
    codec = require_mapping(input_cfg, "codec")
    protocol = require_str(source, "connector")
    adapter = require_str(codec, "adapter")
    observation_name = _protocol_object_name("observation", component_name, input_cfg)
    observation_type = require_str(input_cfg, "type")
    object_name = str(observation_name)
    params: Dict[str, Any] = {
        "object_name": object_name,
        "rcp_observation_name": observation_name,
        "rcp_observation_type": observation_type,
        "component_name": component_name,
        "channel": object_name,
        "channel_transport": channel_transport,
        "msg_size": _input_msg_size(core_config, adapter, observation_type),
        **_shared_data_params(core_config, image=_is_protocol_image_adapter_name(adapter)),
    }
    _copy_optional_fields(
        params,
        input_cfg,
        ("description", "frame_rate", "value_schema", "width", "height", "encoding"),
    )
    if protocol == "module":
        frame_rate = float(require_number(input_cfg, "frame_rate"))
        params.update(
            {
                "module_name": require_str(source, "module_name"),
                "sys_path": list(resolve_refs(source.get("sys_path") or [], server_config)),
                "config_class": None,
                "init_args": resolve_refs(dict(source.get("init") or {}), server_config),
                "start_call": _lifecycle_calls(source, "start"),
                "destroy_call": _lifecycle_calls(source, "stop"),
                "method_name": require_str(source, "method_name"),
                "method_kwargs": dict(source.get("method_kwargs") or {}),
                "interval": 1.0 / frame_rate,
            }
        )
    elif protocol == "port":
        frame_rate = float(require_number(input_cfg, "frame_rate"))
        params.update(
            {
                "port_type": require_str(source, "port_type"),
                "sys_path": list(resolve_refs(source.get("sys_path") or [], server_config)),
                "init_args": resolve_refs(dict(source.get("init") or {}), server_config),
                "method_name": require_str(source, "method_name"),
                "interval": 1.0 / frame_rate,
            }
        )
    elif protocol in ("ros2", "lcm"):
        params.update(
            {
                "topic": require_str(source, "topic"),
                "msg_type": require_str(source, "msg_type"),
            }
        )
        if protocol == "lcm":
            params["payload_mode"] = str(source["payload_mode"])
        elif "payload_mode" in source:
            params["payload_mode"] = str(source["payload_mode"])
        if "qos" in source:
            params["qos"] = dict(source["qos"])
    else:
        raise ValueError(f"Unsupported connector: {protocol}")
    return params


def _output_params(
    *,
    server_config: Mapping[str, Any],
    core_config: Mapping[str, Any],
    channel_transport: str,
    component_name: str,
    output_cfg: Mapping[str, Any],
) -> Dict[str, Any]:
    source = require_mapping(output_cfg, "source")
    codec = require_mapping(output_cfg, "codec")
    protocol = require_str(source, "connector")
    action_name = _protocol_object_name("action", component_name, output_cfg)
    action_type = require_str(output_cfg, "type")
    params: Dict[str, Any] = {
        "connector": protocol,
        "rcp_action_name": action_name,
        "rcp_action_type": action_type,
        "component_name": component_name,
        "action_channel": action_name,
        "action_msg_size": int(require_mapping(core_config, "buffer")["action_msg_size"]),
        "channel_transport": channel_transport,
        "action_consume_mode": str(output_cfg.get("consume_mode") or "queue"),
        **_shared_data_params(core_config),
    }
    _copy_optional_fields(
        params,
        output_cfg,
        ("description", "frame_rate", "input_schema"),
    )
    if protocol == "module":
        params.update(
            {
                "module_name": require_str(source, "module_name"),
                "sys_path": list(resolve_refs(source.get("sys_path") or [], server_config)),
                "config_class": None,
                "init_args": resolve_refs(dict(source.get("init") or {}), server_config),
                "start_call": _lifecycle_calls(source, "start"),
                "destroy_call": _lifecycle_calls(source, "stop"),
                "method_name": require_str(source, "method_name"),
            }
        )
    elif protocol in ("ros2", "lcm"):
        params["topic"] = require_str(source, "topic")
        if protocol == "ros2":
            params["msg_type"] = require_str(source, "msg_type")
        if protocol == "lcm":
            params["payload_mode"] = str(source["payload_mode"])
        elif "payload_mode" in source:
            params["payload_mode"] = str(source["payload_mode"])
        if "qos" in source:
            params["qos"] = dict(source["qos"])
    else:
        raise ValueError(f"Unsupported output connector: {protocol}")
    return params


def _health_params(
    *,
    server_config: Mapping[str, Any],
    core_config: Mapping[str, Any],
    channel_transport: str,
    component_name: str,
    health_cfg: Mapping[str, Any],
) -> Dict[str, Any]:
    source = require_mapping(health_cfg, "source")
    protocol = require_str(source, "connector")
    health_name = f"health.{component_name}"
    params: Dict[str, Any] = {
        "object_name": health_name,
        "health_name": health_name,
        "component_name": component_name,
        "channel": health_name,
        "channel_transport": channel_transport,
        "msg_size": int(require_mapping(core_config, "buffer")["action_msg_size"]),
        **_shared_data_params(core_config),
    }
    if protocol == "module":
        frame_rate = float(require_number(health_cfg, "frame_rate"))
        params.update(
            {
                "module_name": require_str(source, "module_name"),
                "sys_path": list(resolve_refs(source.get("sys_path") or [], server_config)),
                "config_class": None,
                "init_args": resolve_refs(dict(source.get("init") or {}), server_config),
                "start_call": _lifecycle_calls(source, "start"),
                "destroy_call": _lifecycle_calls(source, "stop"),
                "method_name": require_str(source, "method_name"),
                "method_kwargs": dict(source.get("method_kwargs") or {}),
                "interval": 1.0 / frame_rate,
            }
        )
    elif protocol == "port":
        frame_rate = float(require_number(health_cfg, "frame_rate"))
        params.update(
            {
                "port_type": require_str(source, "port_type"),
                "sys_path": list(resolve_refs(source.get("sys_path") or [], server_config)),
                "init_args": resolve_refs(dict(source.get("init") or {}), server_config),
                "method_name": require_str(source, "method_name"),
                "interval": 1.0 / frame_rate,
            }
        )
    elif protocol in ("ros2", "lcm"):
        params.update(
            {
                "topic": require_str(source, "topic"),
                "msg_type": require_str(source, "msg_type"),
            }
        )
        if protocol == "lcm":
            params["payload_mode"] = str(source["payload_mode"])
        elif "payload_mode" in source:
            params["payload_mode"] = str(source["payload_mode"])
        if "qos" in source:
            params["qos"] = dict(source["qos"])
    else:
        raise ValueError(f"Unsupported health connector: {protocol}")
    return params


def _input_specs(
    *,
    runner_name: str,
    input_name: str,
    protocol: str,
    adapter: str,
    params: Dict[str, Any],
) -> list[RunnerInputSpec]:
    specs = []
    for spec_object_name in _input_object_names(params, str(params["object_name"])):
        spec_params = dict(params)
        spec_params["object_name"] = spec_object_name
        payload_type = "image" if _is_protocol_image_adapter_name(adapter) else ("bytes" if protocol == "port" else "json")
        specs.append(
            RunnerInputSpec(
                name=input_name,
                runner_name=runner_name,
                protocol=protocol,
                adapter=adapter,
                channel=params["channel"],
                msg_size=int(params["msg_size"]),
                object_name=spec_object_name,
                channel_transport=str(params["channel_transport"]),
                payload_type=payload_type,
                info=_input_info(protocol, adapter, spec_params),
            )
        )
    return specs


def _input_msg_size(core_config: Mapping[str, Any], adapter: str, observation_type: str) -> int:
    buffer_cfg = require_mapping(core_config, "buffer")
    if _is_protocol_image_adapter_name(adapter):
        return int(buffer_cfg["ref_msg_size"])
    if observation_type == "joint_state":
        return int(buffer_cfg["state_msg_size"])
    return int(buffer_cfg["action_msg_size"])


def _shared_data_params(core_config: Mapping[str, Any], *, image: bool = False) -> Dict[str, int]:
    buffer_cfg = require_mapping(core_config, "buffer")
    if image:
        return {
            "shared_data_buffer_size": int(buffer_cfg["shared_image_data_buffer_size"]),
            "shared_data_slot_count": int(buffer_cfg["shared_image_data_slot_count"]),
        }
    return {
        "shared_data_buffer_size": int(buffer_cfg["shared_data_buffer_size"]),
        "shared_data_slot_count": int(buffer_cfg["shared_data_slot_count"]),
    }


def _copy_optional_fields(target: Dict[str, Any], source: Mapping[str, Any], keys: tuple[str, ...]) -> None:
    for key in keys:
        if key in source:
            target[key] = source[key]


def _protocol_object_name(category: str, component_name: str, cfg: Mapping[str, Any]) -> str:
    return f"{category}.{component_name}.{require_str(cfg, 'name')}"


def _io_items(raw_items: Any, key: str) -> list[Mapping[str, Any]]:
    if not isinstance(raw_items, list):
        raise ValueError(f"{key} must be a list")
    for item in raw_items:
        if not isinstance(item, Mapping):
            raise ValueError(f"{key} entries must be objects")
    return list(raw_items)


def _lifecycle_calls(interface: Mapping[str, Any], key: str) -> list[Dict[str, Any]]:
    return [{"method_name": require_str(interface, key), "method_kwargs": {}}]


def _components_from_runtime(config: Mapping[str, Any], key: str) -> Mapping[str, Any] | list[Any]:
    value = config.get(key)
    if not isinstance(value, (Mapping, list)):
        raise ValueError(f"{key} must be a dict or list")
    return value


def _iter_component_definitions(
    raw_components: Mapping[str, Any] | list[Any],
) -> list[tuple[str, Dict[str, Any]]]:
    if isinstance(raw_components, Mapping):
        components: list[tuple[str, Dict[str, Any]]] = []
        seen: set[str] = set()
        for name, component in raw_components.items():
            if not isinstance(name, str):
                raise ValueError("components keys must be non-empty strings")
            if not isinstance(component, Mapping):
                raise ValueError(f"components.{name} must be a dict")
            component_name = str(name)
            if component_name in seen:
                raise ValueError(f"components has duplicate name {component_name!r}")
            seen.add(component_name)
            components.append((component_name, dict(component)))
        return components
    components: list[tuple[str, Dict[str, Any]]] = []
    seen: set[str] = set()
    for index, component in enumerate(raw_components):
        if not isinstance(component, Mapping):
            raise ValueError(f"components[{index}] must be a dict")
        name = component.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ValueError(f"components[{index}].name is required")
        name = str(name)
        if name in seen:
            raise ValueError(f"components has duplicate name {name!r}")
        seen.add(name)
        components.append((name, dict(component)))
    return components


def _input_object_names(params: Dict[str, Any], primary_name: str) -> list[str]:
    return [str(primary_name)]


def _input_info(protocol: str, adapter: str, params: Dict[str, Any]) -> Dict[str, Any]:
    base = _rcp_observation_info(params)
    if protocol == "port":
        init_args = dict(params["init_args"])
        return {
            **base,
            "type": params["port_type"],
            "device_id": init_args.get("device_id"),
            "width": init_args.get("width"),
            "height": init_args.get("height"),
            "encoding": init_args.get("encoding"),
            "fps": init_args.get("fps"),
            "brand": init_args.get("brand"),
            "rotate": init_args.get("rotate"),
            "object_name": params["object_name"],
            "channel": params["channel"],
            "channel_transport": params["channel_transport"],
            "msg_size": params["msg_size"],
        }
    if _is_protocol_image_adapter_name(adapter):
        if protocol == "module":
            return {
                **base,
                "type": params.get("module_name"),
                "method_name": params.get("method_name"),
                "width": params.get("width"),
                "height": params.get("height"),
                "encoding": params.get("encoding"),
                "object_name": params["object_name"],
                "channel": params["channel"],
                "channel_transport": params["channel_transport"],
                "msg_size": params["msg_size"],
            }
        return {
            **base,
            "type": params.get("type") or params.get("msg_type"),
            "topic": params["topic"],
            "width": params.get("width"),
            "height": params.get("height"),
            "encoding": params.get("encoding"),
            "object_name": params["object_name"],
            "channel": params["channel"],
            "channel_transport": params["channel_transport"],
            "msg_size": params["msg_size"],
        }
    return base


def _is_protocol_image_adapter_name(adapter: str) -> bool:
    return str(adapter).rsplit(".", 1)[-1] == "ProtocolImageInputAdapter"


def _rcp_observation_info(params: Mapping[str, Any]) -> Dict[str, Any]:
    info = {
        "rcp_observation_name": str(params["rcp_observation_name"]),
        "rcp_observation_type": str(params["rcp_observation_type"]),
    }
    _copy_optional_fields(
        info,
        params,
        ("description", "component_name", "frame_rate", "value_schema"),
    )
    return info


def _health_info(params: Mapping[str, Any]) -> Dict[str, Any]:
    return {
        "health_name": str(params["health_name"]),
        "component_name": str(params["component_name"]),
        "channel": str(params["channel"]),
    }


def clone_output_spec_with_target(spec: RunnerOutputSpec, target: Any) -> RunnerOutputSpec:
    return RunnerOutputSpec(
        name=spec.name,
        runner_name=spec.runner_name,
        protocol=spec.protocol,
        adapter=spec.adapter,
        params=deepcopy(spec.params),
        channel=spec.channel,
        msg_size=spec.msg_size,
        channel_transport=spec.channel_transport,
        target=target,
    )
