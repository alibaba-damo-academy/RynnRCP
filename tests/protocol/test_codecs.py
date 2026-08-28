"""Tests for protocol action/observation codecs."""

from __future__ import annotations

from typing import Any

import pytest

from rynnrcp.protocol.action_codecs import (
    ACTION_CODECS,
    action_channel_type,
    action_input_schema,
    encode_action_value,
    is_supported_channel_action_type,
    protocol_action_value,
)
from rynnrcp.protocol.observation_codecs import (
    observation_value_schema,
    protocol_observation_value,
)


# ---------------------------------------------------------------------------
# Action codecs
# ---------------------------------------------------------------------------

def test_action_input_schema_and_channel_type_cover_all_codecs() -> None:
    for action_type in ACTION_CODECS:
        schema = action_input_schema(action_type)
        assert schema["type"] == "object"
        assert action_channel_type(action_type) == action_type
        assert is_supported_channel_action_type(action_type)
    assert not is_supported_channel_action_type("teleport")


def test_action_helpers_reject_unknown_type() -> None:
    with pytest.raises(ValueError, match="Unsupported action type"):
        action_input_schema("warp")
    with pytest.raises(ValueError, match="Unsupported action type"):
        encode_action_value("warp", {})


@pytest.mark.parametrize(
    "action_type, value, expected",
    [
        (
            "joint_position",
            {"joint_positions": [0, 1.5]},
            {"joint_positions": [0.0, 1.5]},
        ),
        (
            "joint_velocity",
            {"joint_velocities": [2]},
            {"joint_velocities": [2.0]},
        ),
        (
            "ee_pose",
            {"position": [1, 2, 3], "orientation_quat_xyzw": [0, 0, 0, 1]},
            {"position": [1.0, 2.0, 3.0], "orientation_quat_xyzw": [0.0, 0.0, 0.0, 1.0]},
        ),
        ("base_velocity", {"linear_vel": [1, 0, 0]}, {"linear_vel": [1.0, 0.0, 0.0]}),
        (
            "base_velocity",
            {"angular_vel": [0, 0, 1]},
            {"angular_vel": [0.0, 0.0, 1.0]},
        ),
        ("gripper", {"position": 0.5}, {"position": 0.5}),
        ("gripper", {"force": 2}, {"force": 2.0}),
        ("head_pose", {"yaw_pitch": [0.1, 0.2]}, {"yaw_pitch": [0.1, 0.2]}),
        (
            "head_pose",
            {"position": [0, 0, 1], "orientation_quat_xyzw": [0, 0, 0, 1]},
            {"position": [0.0, 0.0, 1.0], "orientation_quat_xyzw": [0.0, 0.0, 0.0, 1.0]},
        ),
        ("prearranged", {}, {}),
        ("custom", {"anything": ["goes", 1]}, {"anything": ["goes", 1]}),
    ],
)
def test_encode_and_protocol_action_values(
    action_type: str, value: Any, expected: Any
) -> None:
    assert encode_action_value(action_type, value) == expected
    assert protocol_action_value(action_type, value) == expected


@pytest.mark.parametrize(
    "action_type, value, error",
    [
        ("joint_position", "not-a-dict", TypeError),
        ("joint_position", {}, KeyError),
        ("joint_position", {"joint_positions": "x"}, TypeError),
        ("joint_position", {"joint_positions": [True]}, TypeError),
        ("joint_velocity", 5, TypeError),
        ("joint_velocity", {}, KeyError),
        ("ee_pose", [], TypeError),
        ("ee_pose", {"position": [1, 2], "orientation_quat_xyzw": [0, 0, 0, 1]}, ValueError),
        ("base_velocity", None, TypeError),
        ("base_velocity", {}, KeyError),
        ("gripper", None, TypeError),
        ("gripper", {}, KeyError),
        ("gripper", {"position": "wide"}, TypeError),
        ("head_pose", None, TypeError),
        ("head_pose", {}, KeyError),
        ("prearranged", None, TypeError),
        ("prearranged", {"extra": 1}, ValueError),
        ("custom", "text", TypeError),
    ],
)
def test_encode_action_value_rejects_invalid_values(
    action_type: str, value: Any, error: type[Exception]
) -> None:
    with pytest.raises(error):
        encode_action_value(action_type, value)


def test_protocol_action_value_requires_declared_keys() -> None:
    with pytest.raises(TypeError, match="requires joint_positions"):
        protocol_action_value("joint_position", {"other": 1})
    with pytest.raises(TypeError, match="requires joint_velocities"):
        protocol_action_value("joint_velocity", {"other": 1})


# ---------------------------------------------------------------------------
# Observation codecs
# ---------------------------------------------------------------------------

def test_observation_value_schema_supports_all_known_types() -> None:
    for observation_type in (
        "image",
        "joint_state",
        "ee_pose",
        "gripper_state",
        "imu",
        "force_torque",
        "tactile",
        "point_cloud",
        "robot_state",
    ):
        schema = observation_value_schema(observation_type)
        assert schema["type"] == "object"

    image_schema = observation_value_schema("image", {"encoding": "jpeg"})
    assert image_schema["fields"]["image"]["encoding"] == "jpeg"

    with pytest.raises(ValueError, match="Unsupported observation type"):
        observation_value_schema("thermal")


def test_protocol_observation_joint_state() -> None:
    value = {"joint_positions": [1, 2], "joint_velocities": [0.5, 0.5]}
    assert protocol_observation_value("joint_state", value) == {
        "joint_positions": [1.0, 2.0],
        "joint_velocities": [0.5, 0.5],
    }
    assert protocol_observation_value("joint_state", {"joint_positions": [0]}) == {
        "joint_positions": [0.0]
    }
    with pytest.raises(TypeError, match="requires joint_positions"):
        protocol_observation_value("joint_state", {"velocity": [1]})


def test_protocol_observation_ee_pose_and_gripper() -> None:
    pose = protocol_observation_value(
        "ee_pose", {"position": [1, 2, 3], "orientation_quat_xyzw": [0, 0, 0, 1]}
    )
    assert pose["position"] == [1.0, 2.0, 3.0]

    grip = protocol_observation_value("gripper_state", {"position": 0.4, "force": 1})
    assert grip == {"position": 0.4, "force": 1.0}
    assert protocol_observation_value("gripper_state", {"position": 0.4}) == {
        "position": 0.4
    }
    with pytest.raises(KeyError, match="requires position"):
        protocol_observation_value("gripper_state", {})


def test_protocol_observation_imu_and_force_torque() -> None:
    imu = protocol_observation_value(
        "imu",
        {
            "accel": [0, 0, 9.8],
            "gyro": [0, 0, 0],
            "mag": [1, 0, 0],
            "orientation_quat_wxyz": [1, 0, 0, 0],
        },
    )
    assert imu["mag"] == [1.0, 0.0, 0.0]
    assert imu["orientation_quat_wxyz"] == [1.0, 0.0, 0.0, 0.0]

    minimal = protocol_observation_value("imu", {"accel": [0, 0, 0], "gyro": [0, 0, 0]})
    assert set(minimal) == {"accel", "gyro"}

    wrench = protocol_observation_value(
        "force_torque", {"force": [1, 2, 3], "torque": [0, 0, 1]}
    )
    assert wrench == {"force": [1.0, 2.0, 3.0], "torque": [0.0, 0.0, 1.0]}
    assert protocol_observation_value("force_torque", {"force": [1, 2, 3]}) == {
        "force": [1.0, 2.0, 3.0]
    }


def test_protocol_observation_image() -> None:
    image = protocol_observation_value(
        "image", {"width": 4, "height": 2, "encoding": "jpeg", "image": b"data"}
    )
    assert image == {"width": 4, "height": 2, "encoding": "jpeg", "image": b"data"}

    with pytest.raises(TypeError, match="must be an object"):
        protocol_observation_value("image", b"raw")
    with pytest.raises(KeyError, match="requires image"):
        protocol_observation_value(
            "image", {"width": 1, "height": 1, "encoding": "jpeg"}
        )
    with pytest.raises(KeyError, match="requires width"):
        protocol_observation_value("image", {"height": 1, "encoding": "jpeg", "image": b""})


def test_protocol_observation_tactile() -> None:
    tactile = protocol_observation_value(
        "tactile",
        {
            "contact_press_map": {
                "width": 2,
                "height": 2,
                "encoding": "u8",
                "data": b"\x00\x01\x02\x03",
            },
            "pressure_map": [0.1, 0.2],
            "contact": True,
            "slip": False,
        },
    )
    assert tactile["contact_press_map"]["width"] == 2
    assert tactile["pressure_map"] == [0.1, 0.2]
    assert tactile["contact"] is True and tactile["slip"] is False

    assert protocol_observation_value("tactile", {}) == {}
    with pytest.raises(TypeError, match="must be an object"):
        protocol_observation_value("tactile", [])
    with pytest.raises(TypeError, match="contact_press_map must be an object"):
        protocol_observation_value("tactile", {"contact_press_map": [1]})
    with pytest.raises(TypeError, match="contact must be a bool"):
        protocol_observation_value("tactile", {"contact": 1})


def test_protocol_observation_object_passthrough_types() -> None:
    assert protocol_observation_value("point_cloud", {"points": []}) == {"points": []}
    assert protocol_observation_value("robot_state", {"mode": "idle"}) == {"mode": "idle"}
    with pytest.raises(TypeError, match="must be an object"):
        protocol_observation_value("robot_state", "idle")
    with pytest.raises(ValueError, match="Unsupported observation type"):
        protocol_observation_value("thermal", {})
