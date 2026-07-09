"""ROS2 standard message action output adapter."""

from __future__ import annotations

from typing import Any, Dict, List, Tuple

from .base_output_adapter import BaseOutputAdapter
from rynnrcp.protocol.action_codecs import protocol_action_value
from rynnrcp.utils.imports import get_message_class


class Ros2StandardActionOutputAdapter(BaseOutputAdapter):
    """Convert supported RCP action values into ROS2 standard messages."""

    def build_step_output(
        self,
        step_output: Any,
        outputs: List[Dict[str, Any]],
        fps: float,
    ) -> List[List[Tuple[str, Dict, Any, float, int]]]:
        interval = 1.0 / fps if fps > 0 else 0.0
        frame_group: List[Tuple[str, Dict, Any, float, int]] = []
        for params in outputs:
            action_name = str(params["rcp_action_name"])
            action_type = str(params["rcp_action_type"])
            raw_value = step_output[action_name] if isinstance(step_output, dict) else step_output
            value = protocol_action_value(action_type, raw_value, action_name)
            msg = _message_from_value(params, action_type, value)
            frame_group.append((str(params["connector"]), params, msg, interval, 1))
        return [frame_group] if frame_group else []


def _message_from_value(params: Dict[str, Any], action_type: str, value: Dict[str, Any]) -> Any:
    msg = get_message_class(str(params["msg_type"]))()
    if action_type == "joint_position":
        msg.position = list(value["joint_positions"])
        return msg
    if action_type == "joint_velocity":
        msg.velocity = list(value["joint_velocities"])
        return msg
    if action_type == "ee_pose":
        pose = getattr(msg, "pose")
        _assign_xyz(pose.position, value["position"])
        _assign_xyzw(pose.orientation, value["orientation_quat_xyzw"])
        return msg
    if action_type == "base_velocity":
        if "linear_vel" in value:
            _assign_xyz(msg.linear, value["linear_vel"])
        if "angular_vel" in value:
            _assign_xyz(msg.angular, value["angular_vel"])
        return msg
    raise ValueError(f"Unsupported ROS2 standard action type: {action_type}")


def _assign_xyz(target: Any, values: list[float]) -> None:
    target.x = float(values[0])
    target.y = float(values[1])
    target.z = float(values[2])


def _assign_xyzw(target: Any, values: list[float]) -> None:
    target.x = float(values[0])
    target.y = float(values[1])
    target.z = float(values[2])
    target.w = float(values[3])
