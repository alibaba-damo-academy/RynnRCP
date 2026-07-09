"""Protocol Action codecs used by ActionService and CollectionService."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any, Dict


@dataclass(frozen=True)
class ActionCodec:
    action_type: str
    channel_action_type: str
    input_schema: Dict[str, Any]
    encode: Callable[[Any], Any]
    to_protocol: Callable[[Any], Any]


def action_input_schema(action_type: str, action_name: str | None = None) -> Dict[str, Any]:
    return dict(_codec(action_type).input_schema)


def action_channel_type(action_type: str) -> str:
    return _codec(action_type).channel_action_type


def encode_action_value(action_type: str, value: Any, action_name: str | None = None) -> Any:
    if str(action_type) == "custom":
        return _custom_encode(value, action_name)
    return _codec(action_type).encode(value)


def protocol_action_value(action_type: str, value: Any, action_name: str | None = None) -> Any:
    if str(action_type) == "custom":
        return _custom_to_protocol(value, action_name)
    return _codec(action_type).to_protocol(value)


def is_supported_channel_action_type(channel_action_type: str) -> bool:
    return str(channel_action_type) in {codec.channel_action_type for codec in ACTION_CODECS.values()}


def _codec(action_type: str) -> ActionCodec:
    try:
        return ACTION_CODECS[str(action_type)]
    except KeyError as exc:
        raise ValueError(f"Unsupported action type: {action_type}") from exc


def _joint_position_encode(value: Any) -> Dict[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError("joint_position action value must be an object")
    return {"joint_positions": _required_number_list(value, "joint_positions")}


def _joint_position_to_protocol(value: Any) -> Dict[str, Any]:
    if isinstance(value, Mapping) and "joint_positions" in value:
        return {"joint_positions": _required_number_list(value, "joint_positions")}
    raise TypeError("joint_position action value requires joint_positions")


def _joint_velocity_encode(value: Any) -> Dict[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError("joint_velocity action value must be an object")
    return {"joint_velocities": _required_number_list(value, "joint_velocities")}


def _joint_velocity_to_protocol(value: Any) -> Dict[str, Any]:
    if isinstance(value, Mapping) and "joint_velocities" in value:
        return {"joint_velocities": _required_number_list(value, "joint_velocities")}
    raise TypeError("joint_velocity action value requires joint_velocities")


def _ee_pose_encode(value: Any) -> Dict[str, list[float]]:
    if not isinstance(value, Mapping):
        raise TypeError("ee_pose action value must be an object")
    return {
        "position": _required_number_list(value, "position", length=3),
        "orientation_quat_xyzw": _required_number_list(value, "orientation_quat_xyzw", length=4),
    }


def _ee_pose_to_protocol(value: Any) -> Dict[str, list[float]]:
    return _ee_pose_encode(value)


def _base_velocity_encode(value: Any) -> Dict[str, list[float]]:
    if not isinstance(value, Mapping):
        raise TypeError("base_velocity action value must be an object")
    result: Dict[str, list[float]] = {}
    if "linear_vel" in value:
        result["linear_vel"] = _required_number_list(value, "linear_vel", length=3)
    if "angular_vel" in value:
        result["angular_vel"] = _required_number_list(value, "angular_vel", length=3)
    if not result:
        raise KeyError("base_velocity action value requires linear_vel or angular_vel")
    return result


def _base_velocity_to_protocol(value: Any) -> Dict[str, list[float]]:
    return _base_velocity_encode(value)


def _gripper_encode(value: Any) -> Dict[str, float]:
    if not isinstance(value, Mapping):
        raise TypeError("gripper action value must be an object")
    result: Dict[str, float] = {}
    for key in ("position", "force"):
        if key in value:
            result[key] = _number(value[key], key)
    if not result:
        raise KeyError("gripper action value requires position or force")
    return result


def _gripper_to_protocol(value: Any) -> Dict[str, float]:
    return _gripper_encode(value)


def _head_pose_encode(value: Any) -> Dict[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError("head_pose action value must be an object")
    result: Dict[str, Any] = {}
    if "position" in value:
        result["position"] = _required_number_list(value, "position", length=3)
    if "orientation_quat_xyzw" in value:
        result["orientation_quat_xyzw"] = _required_number_list(value, "orientation_quat_xyzw", length=4)
    if "yaw_pitch" in value:
        result["yaw_pitch"] = _required_number_list(value, "yaw_pitch", length=2)
    if not result:
        raise KeyError("head_pose action value requires position, orientation_quat_xyzw, or yaw_pitch")
    return result


def _head_pose_to_protocol(value: Any) -> Dict[str, Any]:
    return _head_pose_encode(value)


def _prearranged_encode(value: Any) -> Dict[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError("prearranged action value must be an object")
    if value:
        raise ValueError("prearranged action value must be an empty object")
    return {}


def _prearranged_to_protocol(value: Any) -> Dict[str, Any]:
    return _prearranged_encode(value)


def _custom_encode(value: Any, action_name: str | None) -> Dict[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError("custom action value must be an object")
    return dict(value)


def _custom_to_protocol(value: Any, action_name: str | None) -> Dict[str, Any]:
    return _custom_encode(value, action_name)


def _required_number_list(value: Mapping[str, Any], key: str, *, length: int | None = None) -> list[float]:
    if key not in value:
        raise KeyError(f"action value requires {key}")
    return _number_list(value[key], key, length=length)


def _number(value: Any, key: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{key} must be a number")
    return float(value)


def _number_list(value: Any, key: str, *, length: int | None = None) -> list[float]:
    if not isinstance(value, list):
        raise TypeError(f"{key} must be a list")
    if length is not None and len(value) != length:
        raise ValueError(f"{key} must contain {length} values")
    result: list[float] = []
    for index, item in enumerate(value):
        result.append(_number(item, f"{key}[{index}]"))
    return result


ACTION_CODECS: Dict[str, ActionCodec] = {
    "joint_position": ActionCodec(
        action_type="joint_position",
        channel_action_type="joint_position",
        input_schema={
            "type": "object",
            "fields": {
                "joint_positions": {
                    "type": "array",
                    "items": "float",
                    "shape": ["dof"],
                    "unit": "rad",
                }
            },
        },
        encode=_joint_position_encode,
        to_protocol=_joint_position_to_protocol,
    ),
    "joint_velocity": ActionCodec(
        action_type="joint_velocity",
        channel_action_type="joint_velocity",
        input_schema={
            "type": "object",
            "fields": {
                "joint_velocities": {
                    "type": "array",
                    "items": "float",
                    "shape": ["dof"],
                    "unit": "rad/s",
                }
            },
        },
        encode=_joint_velocity_encode,
        to_protocol=_joint_velocity_to_protocol,
    ),
    "ee_pose": ActionCodec(
        action_type="ee_pose",
        channel_action_type="ee_pose",
        input_schema={
            "type": "object",
            "fields": {
                "position": {
                    "type": "array",
                    "items": "float",
                    "shape": [3],
                    "unit": "m",
                },
                "orientation_quat_xyzw": {
                    "type": "array",
                    "items": "float",
                    "shape": [4],
                },
            },
        },
        encode=_ee_pose_encode,
        to_protocol=_ee_pose_to_protocol,
    ),
    "base_velocity": ActionCodec(
        action_type="base_velocity",
        channel_action_type="base_velocity",
        input_schema={
            "type": "object",
            "fields": {
                "linear_vel": {"type": "array", "items": "float", "shape": [3], "optional": True},
                "angular_vel": {"type": "array", "items": "float", "shape": [3], "optional": True},
            },
        },
        encode=_base_velocity_encode,
        to_protocol=_base_velocity_to_protocol,
    ),
    "gripper": ActionCodec(
        action_type="gripper",
        channel_action_type="gripper",
        input_schema={
            "type": "object",
            "fields": {
                "position": {"type": "float", "optional": True},
                "force": {"type": "float", "optional": True},
            },
        },
        encode=_gripper_encode,
        to_protocol=_gripper_to_protocol,
    ),
    "head_pose": ActionCodec(
        action_type="head_pose",
        channel_action_type="head_pose",
        input_schema={
            "type": "object",
            "fields": {
                "position": {"type": "array", "items": "float", "shape": [3], "unit": "m", "optional": True},
                "orientation_quat_xyzw": {"type": "array", "items": "float", "shape": [4], "optional": True},
                "yaw_pitch": {"type": "array", "items": "float", "shape": [2], "unit": "rad", "optional": True},
            },
        },
        encode=_head_pose_encode,
        to_protocol=_head_pose_to_protocol,
    ),
    "prearranged": ActionCodec(
        action_type="prearranged",
        channel_action_type="prearranged",
        input_schema={"type": "object", "fields": {}},
        encode=_prearranged_encode,
        to_protocol=_prearranged_to_protocol,
    ),
    "custom": ActionCodec(
        action_type="custom",
        channel_action_type="custom",
        input_schema={"type": "object", "fields": {}},
        encode=lambda value: _custom_encode(value, None),
        to_protocol=lambda value: _custom_to_protocol(value, None),
    ),
}
