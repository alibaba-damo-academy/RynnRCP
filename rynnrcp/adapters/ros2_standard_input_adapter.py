"""ROS2 standard message input adapter.

Converts a small whitelist of common ROS2 messages into RCP observation values.
This is intentionally not a generic field-mapping adapter.
"""

from __future__ import annotations

import time
from typing import Any, Dict, Tuple

from .base_input_adapter import BaseInputAdapter


class Ros2StandardInputAdapter(BaseInputAdapter):
    """Normalize supported ROS2 standard messages into one RCP observation."""

    def __init__(self, params: Dict[str, Any] | None = None) -> None:
        if params is None:
            raise ValueError("Ros2StandardInputAdapter requires params")
        self._object_name = str(params["object_name"])
        self._observation_type = str(params["rcp_observation_type"])

    def parse(self, msg: Any) -> Tuple[float, Dict[str, Any]]:
        ts = _timestamp_from_header(msg, default=time.time())
        if self._observation_type == "joint_state":
            return ts, {self._object_name: _joint_state_value(msg)}
        if self._observation_type == "ee_pose":
            return ts, {self._object_name: _pose_value(getattr(msg, "pose", msg))}
        raise ValueError(f"Unsupported ROS2 standard observation type: {self._observation_type}")


def _joint_state_value(msg: Any) -> Dict[str, Any]:
    positions = list(getattr(msg, "position", []) or [])
    if not positions:
        raise ValueError("sensor_msgs.msg.JointState requires position")
    value: Dict[str, Any] = {"joint_positions": [float(item) for item in positions]}
    velocities = list(getattr(msg, "velocity", []) or [])
    if velocities:
        value["joint_velocities"] = [float(item) for item in velocities]
    return value


def _pose_value(pose: Any) -> Dict[str, list[float]]:
    position = getattr(pose, "position")
    orientation = getattr(pose, "orientation")
    return {
        "position": [float(position.x), float(position.y), float(position.z)],
        "orientation_quat_xyzw": [
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ],
    }


def _timestamp_from_header(msg: Any, default: float) -> float:
    try:
        header = getattr(msg, "header", None)
        stamp = getattr(header, "stamp", None)
        if stamp is not None:
            return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except Exception:
        pass
    return default
