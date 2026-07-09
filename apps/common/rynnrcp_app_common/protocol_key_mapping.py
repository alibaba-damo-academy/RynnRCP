"""Key mapping between RCP protocol names and RynnBot dataset names."""

from __future__ import annotations

from typing import Any, Iterable, Mapping


def protocol_to_rynnbot_key(name: str, custom: Mapping[str, Any] | None = None) -> str:
    key = str(name or "")
    if isinstance(custom, Mapping) and key in custom and str(custom[key]).strip():
        return str(custom[key])
    if key == "action.robot.joint_position":
        return "action"
    if key == "observation.robot.joint_state":
        return "observation.state"
    parts = key.split(".")
    if len(parts) == 3 and parts[0] == "observation" and parts[2] == "image":
        return f"observation.images.{parts[1]}"
    return key


def rynnbot_to_protocol_key(
    name: str,
    protocol_names: Iterable[str],
    custom: Mapping[str, Any] | None = None,
) -> str:
    key = str(name or "")
    names = [str(item) for item in protocol_names]
    name_set = set(names)
    if key in name_set:
        return key
    if isinstance(custom, Mapping):
        reverse = {str(value): str(source) for source, value in custom.items() if str(value).strip()}
        if key in reverse and reverse[key] in name_set:
            return reverse[key]
    for protocol_name in names:
        if protocol_to_rynnbot_key(protocol_name, custom) == key:
            return protocol_name
    if key == "action":
        return "action.robot.joint_position" if "action.robot.joint_position" in name_set else ""
    if key == "observation.state":
        return "observation.robot.joint_state" if "observation.robot.joint_state" in name_set else ""
    prefix = "observation.images."
    if key.startswith(prefix):
        component = key.removeprefix(prefix)
        for protocol_name in names:
            parts = protocol_name.split(".")
            if len(parts) == 3 and parts[0] == "observation" and parts[2] == "image":
                if parts[1] == component or parts[1].removesuffix("_camera") == component:
                    return protocol_name
    return key


def protocol_to_rynnbot_mapping(
    protocol_names: Iterable[str],
    custom: Mapping[str, Any] | None = None,
) -> dict[str, str]:
    return {str(name): protocol_to_rynnbot_key(str(name), custom) for name in protocol_names}
