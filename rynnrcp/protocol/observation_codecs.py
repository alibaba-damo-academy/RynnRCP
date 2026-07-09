"""Protocol Observation codecs and schema helpers."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Dict


def observation_value_schema(observation_type: str, info: Mapping[str, Any] | None = None) -> Dict[str, Any]:
    info = info or {}
    if observation_type == "image":
        encoding = str(info.get("encoding") or "bytes")
        return {
            "type": "object",
            "fields": {
                "width": {"type": "int"},
                "height": {"type": "int"},
                "encoding": {"type": "string"},
                "image": {"type": "bytes", "encoding": encoding},
            },
        }
    if observation_type == "joint_state":
        return {
            "type": "object",
            "fields": {
                "joint_positions": {
                    "type": "array",
                    "items": "float",
                    "shape": ["dof"],
                    "unit": "rad",
                },
                "joint_velocities": {
                    "type": "array",
                    "items": "float",
                    "shape": ["dof"],
                    "unit": "rad/s",
                    "optional": True,
                },
            },
        }
    if observation_type == "ee_pose":
        return {
            "type": "object",
            "fields": {
                "position": {"type": "array", "items": "float", "shape": [3], "unit": "m"},
                "orientation_quat_xyzw": {"type": "array", "items": "float", "shape": [4]},
            },
        }
    if observation_type == "gripper_state":
        return {
            "type": "object",
            "fields": {
                "position": {"type": "float"},
                "force": {"type": "float", "optional": True},
            },
        }
    if observation_type == "imu":
        return {
            "type": "object",
            "fields": {
                "accel": {"type": "array", "items": "float", "shape": [3]},
                "gyro": {"type": "array", "items": "float", "shape": [3]},
                "mag": {"type": "array", "items": "float", "shape": [3], "optional": True},
                "orientation_quat_wxyz": {"type": "array", "items": "float", "shape": [4], "optional": True},
            },
        }
    if observation_type == "force_torque":
        return {
            "type": "object",
            "fields": {
                "force": {"type": "array", "items": "float", "shape": [3]},
                "torque": {"type": "array", "items": "float", "shape": [3], "optional": True},
            },
        }
    if observation_type == "tactile":
        return {
            "type": "object",
            "fields": {
                "contact_press_map": {
                    "type": "object",
                    "fields": {
                        "width": {"type": "int"},
                        "height": {"type": "int"},
                        "encoding": {"type": "string"},
                        "data": {"type": "bytes"},
                    },
                    "optional": True,
                },
                "pressure_map": {"type": "array", "items": "float", "optional": True},
                "contact": {"type": "bool", "optional": True},
                "slip": {"type": "bool", "optional": True},
            },
        }
    if observation_type in {"point_cloud", "robot_state"}:
        return {"type": "object", "fields": {}}
    raise ValueError(f"Unsupported observation type: {observation_type}")


def protocol_observation_value(observation_type: str, value: Any) -> Any:
    if observation_type == "joint_state":
        return _joint_state(value)
    if observation_type == "ee_pose":
        return {
            "position": _required_number_list(value, "position", length=3),
            "orientation_quat_xyzw": _required_number_list(value, "orientation_quat_xyzw", length=4),
        }
    if observation_type == "gripper_state":
        result = {"position": _required_number(value, "position")}
        if isinstance(value, Mapping) and "force" in value:
            result["force"] = _number(value["force"], "force")
        return result
    if observation_type == "imu":
        result = {
            "accel": _required_number_list(value, "accel", length=3),
            "gyro": _required_number_list(value, "gyro", length=3),
        }
        if isinstance(value, Mapping) and "mag" in value:
            result["mag"] = _number_list(value["mag"], "mag", length=3)
        if isinstance(value, Mapping) and "orientation_quat_wxyz" in value:
            result["orientation_quat_wxyz"] = _number_list(value["orientation_quat_wxyz"], "orientation_quat_wxyz", length=4)
        return result
    if observation_type == "force_torque":
        result = {"force": _required_number_list(value, "force", length=3)}
        if isinstance(value, Mapping) and "torque" in value:
            result["torque"] = _number_list(value["torque"], "torque", length=3)
        return result
    if observation_type == "image":
        return _image(value)
    if observation_type == "tactile":
        return _tactile(value)
    if observation_type in {"point_cloud", "robot_state"}:
        return _object_value(value, observation_type)
    raise ValueError(f"Unsupported observation type: {observation_type}")


def _joint_state(value: Any) -> Dict[str, Any]:
    if isinstance(value, Mapping):
        if "joint_positions" in value:
            result = {"joint_positions": _required_number_list(value, "joint_positions")}
            if "joint_velocities" in value:
                result["joint_velocities"] = _number_list(value["joint_velocities"], "joint_velocities")
            return result
    raise TypeError("joint_state observation value requires joint_positions")


def _image(value: Any) -> Dict[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError("image observation value must be an object")
    result: Dict[str, Any] = {
        "width": int(_required_number(value, "width")),
        "height": int(_required_number(value, "height")),
        "encoding": str(_required_value(value, "encoding")),
    }
    if "image" not in value:
        raise KeyError("image observation value requires image")
    result["image"] = value["image"]
    return result


def _tactile(value: Any) -> Dict[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError("tactile observation value must be an object")
    result: Dict[str, Any] = {}
    if "contact_press_map" in value:
        press_map = value["contact_press_map"]
        if not isinstance(press_map, Mapping):
            raise TypeError("contact_press_map must be an object")
        result["contact_press_map"] = {
            "width": int(_required_number(press_map, "width")),
            "height": int(_required_number(press_map, "height")),
            "encoding": str(_required_value(press_map, "encoding")),
            "data": _required_value(press_map, "data"),
        }
    if "pressure_map" in value:
        result["pressure_map"] = _number_list(value["pressure_map"], "pressure_map")
    if "contact" in value:
        result["contact"] = _bool(value["contact"], "contact")
    if "slip" in value:
        result["slip"] = _bool(value["slip"], "slip")
    return result


def _object_value(value: Any, observation_type: str) -> Dict[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError(f"{observation_type} observation value must be an object")
    return dict(value)


def _required_number(value: Any, key: str) -> float:
    if not isinstance(value, Mapping) or key not in value:
        raise KeyError(f"observation value requires {key}")
    return _number(value[key], key)


def _required_value(value: Any, key: str) -> Any:
    if not isinstance(value, Mapping) or key not in value:
        raise KeyError(f"observation value requires {key}")
    return value[key]


def _required_number_list(value: Any, key: str, *, length: int | None = None) -> list[float]:
    if not isinstance(value, Mapping) or key not in value:
        raise KeyError(f"observation value requires {key}")
    return _number_list(value[key], key, length=length)


def _number(value: Any, key: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{key} must be a number")
    return float(value)


def _bool(value: Any, key: str) -> bool:
    if not isinstance(value, bool):
        raise TypeError(f"{key} must be a bool")
    return value


def _number_list(value: Any, key: str, *, length: int | None = None) -> list[float]:
    if not isinstance(value, list):
        raise TypeError(f"{key} must be a list")
    if length is not None and len(value) != length:
        raise ValueError(f"{key} must contain {length} values")
    return [_number(item, f"{key}[{index}]") for index, item in enumerate(value)]
