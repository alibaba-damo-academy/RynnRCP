"""Configuration validators for RynnRCP source configs."""

from __future__ import annotations

from typing import Any, Mapping

from rynnrcp.protocol.action_codecs import action_input_schema
from rynnrcp.protocol.observation_codecs import observation_value_schema
from rynnrcp.utils import safe_name


VALID_CONNECTORS = {"module", "port", "lcm", "ros2"}
VALID_RUNNER_MODES = {"thread", "process"}
VALID_CONFIG_TRANSPORTS = {"memory", "shm"}
ROS2_STANDARD_OBSERVATIONS = {
    ("sensor_msgs.msg.JointState", "joint_state"): "Ros2StandardInputAdapter",
    ("sensor_msgs.msg.Image", "image"): "ProtocolImageInputAdapter",
    ("sensor_msgs.msg.CompressedImage", "image"): "ProtocolImageInputAdapter",
    ("geometry_msgs.msg.PoseStamped", "ee_pose"): "Ros2StandardInputAdapter",
}
ROS2_STANDARD_ACTIONS = {
    ("sensor_msgs.msg.JointState", "joint_position"): "Ros2StandardActionOutputAdapter",
    ("sensor_msgs.msg.JointState", "joint_velocity"): "Ros2StandardActionOutputAdapter",
    ("geometry_msgs.msg.PoseStamped", "ee_pose"): "Ros2StandardActionOutputAdapter",
    ("geometry_msgs.msg.Twist", "base_velocity"): "Ros2StandardActionOutputAdapter",
}


class ConfigValidationError(ValueError):
    """Raised when a configuration is invalid."""


def validate(config: Mapping[str, Any]) -> None:
    validate_source(config)


def validate_source(config: Mapping[str, Any]) -> None:
    if not isinstance(config, Mapping):
        raise ConfigValidationError(f"config must be a dict, got {type(config).__name__}")
    config_type = _require(config, "config_type", str, "config")
    validators = {
        "rynnrcp_core_config": _validate_core_config,
        "rynnrcp_app_config": _validate_app_config,
        "rynnrcp_robot_integration": _validate_robot_integration,
        "rynnrcp_server_config": _validate_server_config,
    }
    try:
        validators[config_type](config)
    except KeyError as exc:
        raise ConfigValidationError(f"unsupported config_type: {config_type!r}") from exc


class ConfigValidator:
    """Class-style entry point for callers that prefer validator objects."""

    validate = staticmethod(validate)
    validate_source = staticmethod(validate_source)


def _validate_core_config(config: Mapping[str, Any]) -> None:
    _require(config, "version", int, "config")
    runtime = _require(config, "runtime", Mapping, "config")
    runner_mode = _require(runtime, "runner_mode", str, "runtime")
    if runner_mode not in VALID_RUNNER_MODES:
        raise ConfigValidationError("runtime.runner_mode must be 'thread' or 'process'")
    _validate_runtime_startup_timeout(runtime)

    ipc = _require(config, "ipc", Mapping, "config")
    for key in ("thread_transport", "process_transport"):
        value = _require(ipc, key, str, "ipc")
        if value not in VALID_CONFIG_TRANSPORTS:
            raise ConfigValidationError(f"ipc.{key} must be 'memory' or 'shm'")

    runner = config.get("runner")
    if runner is not None and not isinstance(runner, Mapping):
        raise ConfigValidationError("config.runner must be a dict")

    buffer = _require(config, "buffer", Mapping, "config")
    for key in (
        "state_msg_size",
        "action_msg_size",
        "ref_msg_size",
        "image_padding_bytes",
        "shared_data_buffer_size",
        "shared_data_slot_count",
        "shared_image_data_buffer_size",
        "shared_image_data_slot_count",
    ):
        _require(buffer, key, int, "buffer")


def _validate_app_config(config: Mapping[str, Any]) -> None:
    _require(config, "version", int, "config")
    if "app" in config:
        app = _require(config, "app", Mapping, "config")
        if "name" in app and not isinstance(app["name"], str):
            raise ConfigValidationError("app.name must be a string when provided")
    if "defaults" in config and not isinstance(config["defaults"], Mapping):
        raise ConfigValidationError("defaults must be a dict when provided")


def _validate_server_config(config: Mapping[str, Any]) -> None:
    _reject_unknown_fields(
        "config",
        config,
        {"config_type", "version", "manifest", "integration", "components", "server", "connectors", "runtime", "storage", "policies"},
    )
    _require(config, "version", int, "config")
    manifest = _require(config, "manifest", Mapping, "config")
    _reject_unknown_fields("manifest", manifest, {"robot_id", "robot_name", "capabilities"})
    _validate_robot_id(_require(manifest, "robot_id", str, "manifest"), "manifest.robot_id")
    _require(manifest, "robot_name", str, "manifest")
    _validate_capabilities(_require(manifest, "capabilities", Mapping, "manifest"), "manifest.capabilities")

    server = config.get("server") or {}
    if not isinstance(server, Mapping):
        raise ConfigValidationError("server must be a dict when provided")
    _optional_mapping(server, "interface", "server")
    _optional_mapping(server, "metadata", "server")
    _validate_storage_keys("server.storage", server.get("storage"))

    integration = _require(config, "integration", Mapping, "config")
    _require(integration, "config", str, "integration")

    _require(config, "components", Mapping, "config")
    _optional_mapping(config, "connectors", "config")
    _optional_mapping(config, "runtime", "config")
    _optional_mapping(config, "policies", "config")
    _validate_storage_keys("storage", config.get("storage"))


def _validate_robot_integration(config: Mapping[str, Any]) -> None:
    _require(config, "version", int, "config")
    manifest = _require(config, "manifest", Mapping, "config")
    _require(manifest, "embodiment_type", str, "manifest")
    raw_components = _require(config, "components", (Mapping, list), "config")
    component_entries = _iter_component_entries(raw_components, "components")
    manifest_components = manifest.get("components")
    if manifest_components is None:
        manifest["components"] = _manifest_components_from_definitions(component_entries)
    else:
        _validate_components(_require(manifest, "components", list, "manifest"), "manifest.components")
    _validate_model_refs(manifest.get("model_refs"), "manifest.model_refs")
    _optional_mapping(manifest, "metadata", "manifest")
    _validate_capabilities(manifest.get("capabilities"), "manifest.capabilities", allow_refs=True)
    components = raw_components
    observation_names: set[str] = set()
    action_names: set[str] = set()
    for name, component in component_entries:
        if not isinstance(component, Mapping):
            raise ConfigValidationError(f"components.{name} must be a dict")
        _validate_component_config(str(name), component)
        _collect_protocol_names(str(name), component, observation_names, action_names)


def _validate_component_config(name: str, component: Mapping[str, Any]) -> None:
    context = f"components.{name}"
    _reject_unknown_fields(
        context,
        component,
        {"name", "type", "parent_component", "dof", "frame", "description", "enabled", "observations", "actions", "health"},
    )
    if "enabled" not in component:
        raise ConfigValidationError(f"{context}.enabled is required")

    _validate_io_group(name, component, "observations", component.get("observations") or [], _validate_input)
    _validate_io_group(name, component, "actions", component.get("actions") or [], _validate_output)
    if component.get("health") is not None:
        _validate_health(name, _require(component, "health", Mapping, context))


def _iter_component_entries(
    raw_components: Mapping[str, Any] | list[Any], context: str
) -> list[tuple[str, Mapping[str, Any]]]:
    entries: list[tuple[str, Mapping[str, Any]]] = []
    if isinstance(raw_components, Mapping):
        for key, value in raw_components.items():
            if not isinstance(key, str) or not key.strip():
                raise ConfigValidationError(f"{context} keys must be non-empty strings")
            if not isinstance(value, Mapping):
                raise ConfigValidationError(f"{context}.{key} must be a dict")
            entries.append((key, value))
        return entries

    if not isinstance(raw_components, list):
        raise ConfigValidationError(f"{context} must be a dict or list")
    for index, value in enumerate(raw_components):
        if not isinstance(value, Mapping):
            raise ConfigValidationError(f"{context}[{index}] must be a dict")
        name = value.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ConfigValidationError(f"{context}[{index}].name is required")
        if any(existing == str(name) for existing, _ in entries):
            raise ConfigValidationError(f"{context} duplicate component name {name!r}")
        entries.append((str(name), value))
    return entries


def _manifest_components_from_definitions(
    component_entries: list[tuple[str, Mapping[str, Any]]],
) -> list[Dict[str, Any]]:
    manifest_components: list[Dict[str, Any]] = []
    for name, raw_component in component_entries:
        component = {"name": name}
        for key in ("type", "parent_component", "dof", "frame", "description"):
            if key in raw_component:
                component[key] = raw_component[key]
        _validate_components([component], "components (auto-derived from component definitions)")
        manifest_components.append(component)
    return manifest_components


def _validate_io_group(
    interface_name: str,
    interface: Mapping[str, Any],
    group_name: str,
    group: Any,
    validator,
) -> None:
    if not isinstance(group, list):
        raise ConfigValidationError(f"components.{interface_name}.{group_name} must be a list when provided")
    for index, io_cfg in enumerate(group):
        if not isinstance(io_cfg, Mapping):
            raise ConfigValidationError(f"components.{interface_name}.{group_name}[{index}] must be a dict")
        validator(interface_name, interface, index, io_cfg)


def _validate_input(
    interface_name: str,
    interface: Mapping[str, Any],
    index: int,
    input_cfg: Mapping[str, Any],
) -> None:
    context = f"components.{interface_name}.observations[{index}]"
    _reject_unknown_fields(
        context,
        input_cfg,
        {
            "name",
            "type",
            "description",
            "frame_rate",
            "value_schema",
            "width",
            "height",
            "encoding",
            "source",
            "codec",
        },
    )
    source = _validate_source(input_cfg, context, action=False)
    codec = _validate_codec(input_cfg, context)
    protocol = str(source["connector"])
    if protocol in ("module", "port"):
        _require_number(input_cfg, "frame_rate", context)
        _validate_optional_protocol_descriptor(input_cfg, context)
        observation_name = _require(input_cfg, "name", str, context)
        _validate_local_object_name(observation_name, f"{context}.name")
        observation_type = _require(input_cfg, "type", str, context)
        _validate_observation_type(observation_type, context)
        return

    _validate_optional_protocol_descriptor(input_cfg, context)
    observation_name = _require(input_cfg, "name", str, context)
    _validate_local_object_name(observation_name, f"{context}.name")
    observation_type = _require(input_cfg, "type", str, context)
    _validate_observation_type(observation_type, context)
    if protocol == "ros2":
        _require(source, "qos", Mapping, f"{context}.source")
        payload_mode = _require(source, "payload_mode", str, f"{context}.source")
        if payload_mode == "protocol_json":
            if _require(codec, "adapter", str, f"{context}.codec") != "ProtocolInputAdapter":
                raise ConfigValidationError(f"{context}.codec.adapter must be ProtocolInputAdapter")
        elif payload_mode == "ros2_standard":
            _validate_ros2_standard_observation(
                str(source["msg_type"]),
                observation_type,
                str(codec["adapter"]),
                context,
            )
            if str(source["msg_type"]) == "sensor_msgs.msg.CompressedImage":
                _require(input_cfg, "width", int, context)
                _require(input_cfg, "height", int, context)
                _require(input_cfg, "encoding", str, context)
        else:
            raise ConfigValidationError(f"{context}.source.payload_mode must be protocol_json or ros2_standard")
    if protocol == "lcm":
        if _require(source, "msg_type", str, f"{context}.source") != "json":
            raise ConfigValidationError(f"{context}.source.msg_type must be json")
        if _require(source, "payload_mode", str, f"{context}.source") != "protocol_json":
            raise ConfigValidationError(f"{context}.source.payload_mode must be protocol_json")
        if _require(codec, "adapter", str, f"{context}.codec") != "ProtocolInputAdapter":
            raise ConfigValidationError(f"{context}.codec.adapter must be ProtocolInputAdapter")


def _validate_output(
    interface_name: str,
    interface: Mapping[str, Any],
    index: int,
    output_cfg: Mapping[str, Any],
) -> None:
    context = f"components.{interface_name}.actions[{index}]"
    _reject_unknown_fields(
        context,
        output_cfg,
        {
            "name",
            "type",
            "consume_mode",
            "action_poll_hz",
            "description",
            "frame_rate",
            "input_schema",
            "source",
            "codec",
        },
    )
    source = _validate_source(output_cfg, context, action=True)
    codec = _validate_codec(output_cfg, context)
    protocol = str(source["connector"])
    if protocol == "port":
        raise ConfigValidationError(f"{context}: port components do not support actions")
    _validate_optional_protocol_descriptor(output_cfg, context)
    action_name = _require(output_cfg, "name", str, context)
    _validate_local_object_name(action_name, f"{context}.name")
    action_type = _require(output_cfg, "type", str, context)
    _validate_action_type(action_type, context, _protocol_object_name("action", interface_name, output_cfg))
    if protocol == "module":
        return

    if protocol == "ros2":
        _require(source, "topic", str, f"{context}.source")
        _require(source, "msg_type", str, f"{context}.source")
        _require(source, "qos", Mapping, f"{context}.source")
        payload_mode = _require(source, "payload_mode", str, f"{context}.source")
        if payload_mode == "protocol_json":
            if _require(codec, "adapter", str, f"{context}.codec") != "ProtocolActionOutputAdapter":
                raise ConfigValidationError(f"{context}.codec.adapter must be ProtocolActionOutputAdapter")
        elif payload_mode == "ros2_standard":
            _validate_ros2_standard_action(
                str(source["msg_type"]),
                action_type,
                str(codec["adapter"]),
                context,
            )
        else:
            raise ConfigValidationError(f"{context}.source.payload_mode must be protocol_json or ros2_standard")
        return
    if protocol == "lcm":
        _require(source, "topic", str, f"{context}.source")
        if _require(codec, "adapter", str, f"{context}.codec") != "ProtocolActionOutputAdapter":
            raise ConfigValidationError(f"{context}.codec.adapter must be ProtocolActionOutputAdapter")
        if _require(source, "payload_mode", str, f"{context}.source") != "protocol_json":
            raise ConfigValidationError(f"{context}.source.payload_mode must be protocol_json")
        return


def _validate_health(
    component_name: str,
    health_cfg: Mapping[str, Any],
) -> None:
    context = f"components.{component_name}.health"
    _reject_unknown_fields(context, health_cfg, {"frame_rate", "source"})
    source = _validate_source(health_cfg, context, action=False)
    protocol = str(source["connector"])
    if protocol in ("module", "port"):
        _require_number(health_cfg, "frame_rate", context)
    if protocol == "ros2":
        _require(source, "qos", Mapping, f"{context}.source")
        if _require(source, "payload_mode", str, f"{context}.source") != "protocol_json":
            raise ConfigValidationError(f"{context}.source.payload_mode must be protocol_json")
    if protocol == "lcm":
        if _require(source, "msg_type", str, f"{context}.source") != "json":
            raise ConfigValidationError(f"{context}.source.msg_type must be json")
        if _require(source, "payload_mode", str, f"{context}.source") != "protocol_json":
            raise ConfigValidationError(f"{context}.source.payload_mode must be protocol_json")


def _validate_source(config: Mapping[str, Any], context: str, *, action: bool) -> Mapping[str, Any]:
    source = _require(config, "source", Mapping, context)
    connector = _require(source, "connector", str, f"{context}.source")
    if connector not in VALID_CONNECTORS:
        raise ConfigValidationError(f"{context}.source.connector is unsupported: {connector!r}")
    if connector == "module":
        _reject_unknown_fields(
            f"{context}.source",
            source,
            {"connector", "module_name", "sys_path", "init", "start", "stop", "method_name", "method_kwargs"},
        )
        _require(source, "module_name", str, f"{context}.source")
        _require(source, "init", Mapping, f"{context}.source")
        for key in ("start", "stop", "method_name"):
            _require(source, key, str, f"{context}.source")
    elif connector == "port":
        _reject_unknown_fields(
            f"{context}.source",
            source,
            {"connector", "port_type", "sys_path", "init", "method_name"},
        )
        _require(source, "port_type", str, f"{context}.source")
        _require(source, "init", Mapping, f"{context}.source")
        _require(source, "method_name", str, f"{context}.source")
    elif connector == "ros2":
        _reject_unknown_fields(
            f"{context}.source",
            source,
            {"connector", "topic", "msg_type", "payload_mode", "qos"},
        )
        _require(source, "topic", str, f"{context}.source")
        _require(source, "msg_type", str, f"{context}.source")
    elif connector == "lcm":
        allowed = {"connector", "topic", "payload_mode"} if action else {"connector", "topic", "msg_type", "payload_mode"}
        _reject_unknown_fields(
            f"{context}.source",
            source,
            allowed,
        )
        _require(source, "topic", str, f"{context}.source")
    if action and connector == "port":
        raise ConfigValidationError(f"{context}.source.connector port does not support actions")
    return source


def _validate_codec(config: Mapping[str, Any], context: str) -> Mapping[str, Any]:
    codec = _require(config, "codec", Mapping, context)
    _reject_unknown_fields(f"{context}.codec", codec, {"adapter"})
    _require(codec, "adapter", str, f"{context}.codec")
    return codec


def _collect_protocol_names(
    interface_name: str,
    interface: Mapping[str, Any],
    observation_names: set[str],
    action_names: set[str],
) -> None:
    for index, input_cfg in enumerate(interface.get("observations") or []):
        if not isinstance(input_cfg, Mapping):
            continue
        for name in _configured_observation_names(interface_name, input_cfg):
            _add_unique_name(observation_names, name, f"components.{interface_name}.observations[{index}].name")
    for index, output_cfg in enumerate(interface.get("actions") or []):
        if not isinstance(output_cfg, Mapping):
            continue
        for name in _configured_action_names(interface_name, output_cfg):
            _add_unique_name(action_names, name, f"components.{interface_name}.actions[{index}].name")


def _configured_observation_names(component_name: str, input_cfg: Mapping[str, Any]) -> list[str]:
    return [_protocol_object_name("observation", component_name, input_cfg)]


def _configured_action_names(component_name: str, output_cfg: Mapping[str, Any]) -> list[str]:
    return [_protocol_object_name("action", component_name, output_cfg)]


def _add_unique_name(seen: set[str], name: str, context: str) -> None:
    if name in seen:
        raise ConfigValidationError(f"{context} duplicates protocol object name {name!r}")
    seen.add(name)


def _validate_local_object_name(name: str, context: str) -> None:
    if "." in name or safe_name(name) != name:
        raise ConfigValidationError(f"{context} must be a local object name such as 'joint_state' or 'image'")


def _protocol_object_name(category: str, component_name: str, config: Mapping[str, Any]) -> str:
    return f"{category}.{component_name}.{str(config['name'])}"


def _validate_optional_protocol_descriptor(config: Mapping[str, Any], context: str) -> None:
    for key in ("description",):
        _optional_str(config, key, context)
    _optional_number(config, "frame_rate", context)
    for key in ("value_schema", "input_schema"):
        _optional_mapping(config, key, context)


def _validate_observation_type(observation_type: str, context: str) -> None:
    try:
        observation_value_schema(observation_type)
    except ValueError as exc:
        raise ConfigValidationError(f"{context}.type is not a supported protocol observation type: {observation_type}") from exc


def _validate_action_type(action_type: str, context: str, action_name: str) -> None:
    try:
        action_input_schema(action_type, action_name)
    except ValueError as exc:
        raise ConfigValidationError(f"{context}.type is not a supported protocol action type: {action_type}") from exc


def _validate_ros2_standard_observation(
    msg_type: str,
    observation_type: str,
    adapter: str,
    context: str,
) -> None:
    expected_adapter = ROS2_STANDARD_OBSERVATIONS.get((msg_type, observation_type))
    if expected_adapter is None:
        supported = ", ".join(f"{m}->{t}" for m, t in sorted(ROS2_STANDARD_OBSERVATIONS))
        raise ConfigValidationError(
            f"{context}.source ros2_standard does not support {msg_type!r} for "
            f"observation type {observation_type!r}; supported: {supported}"
        )
    if adapter != expected_adapter:
        raise ConfigValidationError(f"{context}.codec.adapter must be {expected_adapter}")


def _validate_ros2_standard_action(
    msg_type: str,
    action_type: str,
    adapter: str,
    context: str,
) -> None:
    expected_adapter = ROS2_STANDARD_ACTIONS.get((msg_type, action_type))
    if expected_adapter is None:
        supported = ", ".join(f"{m}->{t}" for m, t in sorted(ROS2_STANDARD_ACTIONS))
        raise ConfigValidationError(
            f"{context}.source ros2_standard does not support {msg_type!r} for "
            f"action type {action_type!r}; supported: {supported}"
        )
    if adapter != expected_adapter:
        raise ConfigValidationError(f"{context}.codec.adapter must be {expected_adapter}")


def _validate_storage_keys(context: str, storage: Any) -> None:
    if storage is None:
        return
    if not isinstance(storage, Mapping):
        raise ConfigValidationError(f"{context} must be a dict when provided")
    unknown = sorted(str(key) for key in storage if key != "collection_dir")
    if unknown:
        raise ConfigValidationError(f"{context} only supports collection_dir")


def _validate_robot_id(value: str, context: str) -> None:
    if not value.strip():
        raise ConfigValidationError(f"{context} is required")
    if safe_name(value) != value:
        raise ConfigValidationError(f"{context} may only contain letters, numbers, '.', '_' and '-'")


def _validate_capabilities(capabilities: Any, context: str, *, allow_refs: bool = False) -> None:
    if capabilities is None:
        return
    if not isinstance(capabilities, Mapping):
        raise ConfigValidationError(f"{context} must be a dict when provided")
    required = {"observations", "actions", "health", "resources", "data_collection", "policy_service"}
    allowed = required
    unknown = sorted(str(key) for key in capabilities if str(key) not in allowed)
    if unknown:
        raise ConfigValidationError(f"{context} has unsupported field(s): {', '.join(unknown)}")
    missing = sorted(key for key in required if key not in capabilities)
    if missing:
        raise ConfigValidationError(f"{context} missing required field(s): {', '.join(missing)}")
    for key, value in capabilities.items():
        if allow_refs and isinstance(value, str) and value.startswith("${") and value.endswith("}"):
            continue
        if not isinstance(value, bool):
            raise ConfigValidationError(f"{context}.{key} must be a boolean")


def _validate_components(components: Any, context: str = "manifest.components") -> None:
    if not isinstance(components, list):
        raise ConfigValidationError(f"{context} must be a list")
    names: set[str] = set()
    for index, component in enumerate(components):
        item_context = f"{context}[{index}]"
        if not isinstance(component, Mapping):
            raise ConfigValidationError(f"{item_context} must be a dict")
        _reject_unknown_fields(
            item_context,
            component,
            {"name", "type", "parent_component", "dof", "frame", "description"},
        )
        name = _require(component, "name", str, item_context)
        _add_unique_name(names, name, f"{item_context}.name")
        _require(component, "type", str, item_context)
        if "parent_component" in component and component["parent_component"] is not None:
            if not isinstance(component["parent_component"], str):
                raise ConfigValidationError(f"{item_context}.parent_component must be a string or null")
        if "dof" in component and component["dof"] is not None:
            if not isinstance(component["dof"], (int, float)) or isinstance(component["dof"], bool):
                raise ConfigValidationError(f"{item_context}.dof must be a number")
        _optional_str(component, "frame", item_context)
        _optional_str(component, "description", item_context)


def _validate_model_refs(model_refs: Any, context: str = "manifest.model_refs") -> None:
    if model_refs is None:
        return
    if not isinstance(model_refs, Mapping):
        raise ConfigValidationError(f"{context} must be a dict when provided")
    allowed = {"urdf", "mjcf", "calibration", "assets"}
    unknown = sorted(str(key) for key in model_refs if str(key) not in allowed)
    if unknown:
        raise ConfigValidationError(f"{context} has unsupported field(s): {', '.join(unknown)}")
    for key, value in model_refs.items():
        item_context = f"{context}.{key}"
        if key == "assets":
            if value is None:
                continue
            if not isinstance(value, list):
                raise ConfigValidationError(f"{item_context} must be a list")
            for index, item in enumerate(value):
                if isinstance(item, str) and item.strip():
                    continue
                _validate_resource_object(item, f"{item_context}[{index}]")
            continue
        if value in (None, ""):
            continue
        if isinstance(value, str):
            continue
        if not isinstance(value, Mapping):
            raise ConfigValidationError(f"{item_context} must be a path string or object when provided")
        _reject_unknown_fields(
            item_context,
            value,
            {"format", "content", "resource_id", "hash", "size_bytes"},
        )
        if not value.get("content") and not value.get("resource_id"):
            raise ConfigValidationError(f"{item_context} requires content or resource_id")
        _optional_str(value, "format", item_context)
        _optional_str(value, "content", item_context)
        _optional_str(value, "resource_id", item_context)
        _optional_str(value, "hash", item_context)
        if "size_bytes" in value and value["size_bytes"] is not None:
            if not isinstance(value["size_bytes"], int) or isinstance(value["size_bytes"], bool):
                raise ConfigValidationError(f"{item_context}.size_bytes must be an int")


def _validate_resource_object(value: Any, context: str) -> None:
    if not isinstance(value, Mapping):
        raise ConfigValidationError(f"{context} must be a dict")
    required = {"resource_id", "type", "mode"}
    missing = sorted(key for key in required if key not in value)
    if missing:
        raise ConfigValidationError(f"{context} missing required field(s): {', '.join(missing)}")
    _reject_unknown_fields(
        context,
        value,
        {
            "resource_id",
            "type",
            "domain",
            "name",
            "format",
            "mime_type",
            "mode",
            "size_bytes",
            "hash",
            "created_at",
            "updated_at",
            "metadata",
        },
    )
    for key in ("resource_id", "type", "mode"):
        _require(value, key, str, context)


def _reject_unknown_fields(context: str, config: Mapping[str, Any], allowed: set[str]) -> None:
    unknown = sorted(str(key) for key in config if str(key) not in allowed)
    if unknown:
        raise ConfigValidationError(f"{context} has unsupported field(s): {', '.join(unknown)}")


def _validate_runtime_startup_timeout(runtime: Mapping[str, Any]) -> None:
    if "startup_timeout_s" not in runtime:
        return
    value = runtime["startup_timeout_s"]
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ConfigValidationError("runtime.startup_timeout_s must be a number")
    if float(value) <= 0:
        raise ConfigValidationError("runtime.startup_timeout_s must be greater than 0")


def _optional_mapping(config: Mapping[str, Any], key: str, context: str) -> None:
    if key in config and config[key] is not None and not isinstance(config[key], Mapping):
        raise ConfigValidationError(f"{context}.{key} must be a dict when provided")


def _optional_str(config: Mapping[str, Any], key: str, context: str) -> None:
    if key in config and config[key] is not None and not isinstance(config[key], str):
        raise ConfigValidationError(f"{context}.{key} must be a string when provided")


def _optional_number(config: Mapping[str, Any], key: str, context: str) -> None:
    if key not in config or config[key] is None:
        return
    if not isinstance(config[key], (int, float)) or isinstance(config[key], bool):
        raise ConfigValidationError(f"{context}.{key} must be a number when provided")


def _require(config: Mapping[str, Any], key: str, expected_type: Any, context: str) -> Any:
    if key not in config:
        raise ConfigValidationError(f"{context}.{key} is required")
    value = config[key]
    if not isinstance(value, expected_type):
        raise ConfigValidationError(
            f"{context}.{key} must be {_type_name(expected_type)}, got {type(value).__name__}"
        )
    if expected_type is str and not value.strip():
        raise ConfigValidationError(f"{context}.{key} must be a non-empty string")
    return value


def _require_number(config: Mapping[str, Any], key: str, context: str) -> float:
    value = _require(config, key, (int, float), context)
    if isinstance(value, bool):
        raise ConfigValidationError(f"{context}.{key} must be number, got bool")
    return float(value)


def _type_name(expected_type: Any) -> str:
    if isinstance(expected_type, tuple):
        return " or ".join(item.__name__ for item in expected_type)
    return expected_type.__name__
