"""Tests for RCP <-> RynnBot key mapping."""

from __future__ import annotations

from rynnrcp_app_common.protocol_key_mapping import protocol_to_rynnbot_key, rynnbot_to_protocol_key


def test_default_rynnbot_mapping_uses_robot_component_for_whole_body_keys() -> None:
    protocol_names = [
        "observation.robot.joint_state",
        "action.robot.joint_position",
        "observation.arm.joint_state",
        "action.arm.joint_position",
        "observation.wrist.image",
    ]

    assert protocol_to_rynnbot_key("observation.robot.joint_state") == "observation.state"
    assert protocol_to_rynnbot_key("action.robot.joint_position") == "action"
    assert protocol_to_rynnbot_key("observation.arm.joint_state") == "observation.arm.joint_state"
    assert protocol_to_rynnbot_key("action.arm.joint_position") == "action.arm.joint_position"
    assert protocol_to_rynnbot_key("observation.wrist.image") == "observation.images.wrist"
    assert rynnbot_to_protocol_key("observation.state", protocol_names) == "observation.robot.joint_state"
    assert rynnbot_to_protocol_key("action", protocol_names) == "action.robot.joint_position"


def test_default_rynnbot_mapping_does_not_guess_non_robot_joint_keys() -> None:
    protocol_names = [
        "observation.arm.joint_state",
        "action.arm.joint_position",
    ]

    assert rynnbot_to_protocol_key("observation.state", protocol_names) == ""
    assert rynnbot_to_protocol_key("action", protocol_names) == ""
