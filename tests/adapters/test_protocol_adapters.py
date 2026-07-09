"""Tests for the current protocol adapters."""

from __future__ import annotations

import json

import pytest

from rynnrcp.adapters.protocol_action_output_adapter import ProtocolActionOutputAdapter
from rynnrcp.adapters.protocol_image_input_adapter import ProtocolImageInputAdapter
from rynnrcp.adapters.protocol_input_adapter import ProtocolInputAdapter
from rynnrcp.adapters.ros2_standard_action_output_adapter import Ros2StandardActionOutputAdapter
from rynnrcp.adapters.ros2_standard_input_adapter import Ros2StandardInputAdapter
from rynnrcp.protocol.action_codecs import action_input_schema, encode_action_value


class _Msg:
    position = [1.0, 2.0, 3.0]


class _StringMsg:
    def __init__(self) -> None:
        self.data = ""


class _JointStateMsg:
    def __init__(self) -> None:
        self.position = []
        self.velocity = []


class _XYZ:
    x = 0.0
    y = 0.0
    z = 0.0


class _XYZW:
    x = 0.0
    y = 0.0
    z = 0.0
    w = 1.0


class _Pose:
    def __init__(self) -> None:
        self.position = _XYZ()
        self.orientation = _XYZW()


class _PoseStampedMsg:
    def __init__(self) -> None:
        self.pose = _Pose()


class _TwistMsg:
    def __init__(self) -> None:
        self.linear = _XYZ()
        self.angular = _XYZ()


def test_protocol_input_adapter_maps_fields_to_protocol_object() -> None:
    adapter = ProtocolInputAdapter(
        {
            "object_name": "observation.robot.joint_state",
            "mappings": [
                {
                    "field": "position",
                    "object_name": "observation.robot.joint_state",
                }
            ],
        }
    )

    _, payload = adapter.parse(_Msg())

    assert payload == {"observation.robot.joint_state": [1.0, 2.0, 3.0]}


def test_protocol_input_adapter_accepts_protocol_json_payload() -> None:
    adapter = ProtocolInputAdapter(
        {
            "object_name": "observation.robot.joint_state",
            "payload_mode": "protocol_json",
        }
    )

    _, payload = adapter.parse(
        {
            "name": "observation.robot.joint_state",
            "value": {"joint_positions": [1.0, 2.0, 3.0]},
        }
    )

    assert payload == {
        "observation.robot.joint_state": {
            "joint_positions": [1.0, 2.0, 3.0],
        }
    }


def test_protocol_image_input_adapter_normalizes_camera_tuple() -> None:
    adapter = ProtocolImageInputAdapter({"object_name": "observation.front.image"})

    _, payload = adapter.parse((True, 640, 360, "jpg", b"image-bytes"))

    assert payload == {
        "observation.front.image": {
            "width": 640,
            "height": 360,
            "encoding": "jpg",
            "image": b"image-bytes",
        }
    }


def test_protocol_action_output_adapter_serializes_ros2_protocol_json() -> None:
    adapter = ProtocolActionOutputAdapter()
    outputs = [
        {
            "connector": "ros2",
            "rcp_action_name": "action.robot.joint_position",
            "rcp_action_type": "joint_position",
            "payload_mode": "protocol_json",
            "msg_type": f"{__name__}._StringMsg",
        }
    ]

    [(connector, params, msg, interval, step)] = adapter.build_step_output(
        {"action.robot.joint_position": {"joint_positions": [1.0, 2.0, 3.0]}},
        outputs,
        fps=20,
    )[0]

    assert connector == "ros2"
    assert params is outputs[0]
    assert interval == 0.05
    assert step == 1
    assert json.loads(msg.data) == {
        "name": "action.robot.joint_position",
        "type": "joint_position",
        "value": {"joint_positions": [1.0, 2.0, 3.0]},
    }


def test_custom_action_schema_is_open_object() -> None:
    schema = action_input_schema("custom", "action.robot.anything")

    assert schema == {"type": "object", "fields": {}}


def test_custom_action_value_is_direct() -> None:
    assert encode_action_value(
        "custom",
        {"foo": 1, "joint_positions": [1, 2, 3]},
        "action.robot.anything",
    ) == {"foo": 1, "joint_positions": [1, 2, 3]}


def test_custom_action_value_requires_object() -> None:
    with pytest.raises(TypeError):
        encode_action_value("custom", [1, 2, 3], "action.robot.anything")


def test_ros2_standard_input_adapter_normalizes_joint_state() -> None:
    adapter = Ros2StandardInputAdapter(
        {
            "object_name": "observation.robot.joint_state",
            "rcp_observation_type": "joint_state",
        }
    )
    msg = _JointStateMsg()
    msg.position = [1, 2, 3]
    msg.velocity = [0.1, 0.2, 0.3]

    _, payload = adapter.parse(msg)

    assert payload == {
        "observation.robot.joint_state": {
            "joint_positions": [1.0, 2.0, 3.0],
            "joint_velocities": [0.1, 0.2, 0.3],
        }
    }


def test_ros2_standard_input_adapter_normalizes_pose_stamped() -> None:
    adapter = Ros2StandardInputAdapter(
        {
            "object_name": "observation.arm.ee_pose",
            "rcp_observation_type": "ee_pose",
        }
    )
    msg = _PoseStampedMsg()
    msg.pose.position.x = 0.1
    msg.pose.position.y = 0.2
    msg.pose.position.z = 0.3
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0

    _, payload = adapter.parse(msg)

    assert payload == {
        "observation.arm.ee_pose": {
            "position": [0.1, 0.2, 0.3],
            "orientation_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
        }
    }


def test_ros2_standard_action_output_adapter_normalizes_twist() -> None:
    adapter = Ros2StandardActionOutputAdapter()
    outputs = [
        {
            "connector": "ros2",
            "rcp_action_name": "action.mobile_base.velocity",
            "rcp_action_type": "base_velocity",
            "payload_mode": "ros2_standard",
            "msg_type": f"{__name__}._TwistMsg",
        }
    ]

    [(connector, _, msg, interval, step)] = adapter.build_step_output(
        {"action.mobile_base.velocity": {"linear_vel": [1.0, 0.0, 0.0], "angular_vel": [0.0, 0.0, 0.5]}},
        outputs,
        fps=10,
    )[0]

    assert connector == "ros2"
    assert interval == 0.1
    assert step == 1
    assert (msg.linear.x, msg.linear.y, msg.linear.z) == (1.0, 0.0, 0.0)
    assert (msg.angular.x, msg.angular.y, msg.angular.z) == (0.0, 0.0, 0.5)
