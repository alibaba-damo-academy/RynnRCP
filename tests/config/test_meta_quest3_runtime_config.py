from __future__ import annotations

from pathlib import Path

import pytest

from rynnrcp.config.loader import load_config
from rynnrcp.config.runner_config import build_runner_config
from rynnrcp.config.runtime_config import RuntimeConfig
from rynnrcp_robot_meta_quest3.server_cli import load_config_with_source_ip


CONFIG_DIR = (
    Path(__file__).resolve().parents[2]
    / "robots"
    / "meta_quest3"
    / "rynnrcp_robot_meta_quest3"
    / "config"
)


def test_server_source_ip_parameter_overrides_yaml_without_writing_it() -> None:
    config_path = CONFIG_DIR / "meta_quest3_franka_fr3_right_server.yaml"
    original = config_path.read_text(encoding="utf-8")

    config = load_config_with_source_ip(str(config_path), "172.16.1.29")

    assert config["components"]["robot"]["source_ip"] == "172.16.1.29"
    assert config_path.read_text(encoding="utf-8") == original


def test_server_source_ip_parameter_requires_ipv4() -> None:
    config_path = CONFIG_DIR / "meta_quest3_franka_fr3_right_server.yaml"

    with pytest.raises(ValueError, match="valid IPv4"):
        load_config_with_source_ip(str(config_path), "quest.local")


def test_franka_fr3_right_config_converts_quest_motion_to_joint_targets() -> None:
    runtime_config = RuntimeConfig.load(
        str(CONFIG_DIR / "meta_quest3_franka_fr3_right_server.yaml")
    )
    runner_config = build_runner_config(runtime_config)

    assert [spec.name for spec in runner_config.input_specs] == [
        "observation.robot.joint_state",
        "observation.robot.controller_state",
    ]
    assert runner_config.output_specs == []
    assert all(
        configured_input.params["module_name"].endswith(
            "MetaQuest3UrdfJointController"
        )
        for configured_input in runner_config.inputs
    )
    init_args = runner_config.inputs[0].params["init_args"]
    assert init_args["target_model"] == "franka_fr3"
    assert init_args["target_urdf"] == (
        "package://rynnrcp_robot_meta_quest3/models/fr3.urdf"
    )
    assert init_args["target_dof"] == 7
    assert init_args["control_dof"] == 8
    assert init_args["joint_order"] == "joints_then_gripper"
    assert init_args["base_link"] == "fr3_link0"
    assert init_args["tip_link"] == "fr3_hand_tcp"
    assert init_args["controller_side"] == "right"
    assert init_args["home_joint_positions"] == pytest.approx(
        [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    )
    assert init_args["workspace_delta_limits_m"] == [
        [-0.18, 0.40],
        [-0.50, 0.50],
        [-0.50, 0.00],
    ]
    manifest = runtime_config.runtime_context["manifest"]
    assert runtime_config.robot_id == "meta_quest3_franka_fr3_right"
    assert manifest["embodiment_type"] == "single_arm"
    assert manifest["metadata"]["control_output"] == "joint_position"
    assert manifest["metadata"]["joint_order"] == "joints_then_gripper"
    assert manifest["components"][0]["dof"] == 8


def test_franka_fr3_dual_config_exposes_one_sixteen_value_vector() -> None:
    runtime_config = RuntimeConfig.load(
        str(CONFIG_DIR / "meta_quest3_franka_fr3_dual_server.yaml")
    )
    runner_config = build_runner_config(runtime_config)

    assert [spec.name for spec in runner_config.input_specs] == [
        "observation.robot.joint_state",
        "observation.robot.controller_state",
    ]
    assert all(
        configured_input.params["module_name"].endswith(
            "MetaQuest3DualUrdfJointController"
        )
        for configured_input in runner_config.inputs
    )
    init_args = runner_config.inputs[0].params["init_args"]
    assert set(init_args["arms"]) == {"left", "right"}
    assert init_args["control_dof"] == 16
    assert init_args["joint_order"] == (
        "left_joints_left_gripper_right_joints_right_gripper"
    )
    assert init_args["arms"]["left"]["target_dof"] == 7
    assert init_args["arms"]["right"]["target_dof"] == 7
    assert runtime_config.robot_id == "meta_quest3_franka_fr3_dual"
    manifest = runtime_config.runtime_context["manifest"]
    assert manifest["embodiment_type"] == "dual_arm"
    assert manifest["components"][0]["dof"] == 16
    assert manifest["metadata"]["joint_order"] == (
        "left_joints_left_gripper_right_joints_right_gripper"
    )


@pytest.mark.parametrize(
    ("config_name", "control_dof", "joint_order"),
    [
        ("meta_quest3_franka_fr3_right_server.yaml", 6, "joints"),
        (
            "meta_quest3_franka_fr3_dual_server.yaml",
            14,
            "left_joints_right_joints",
        ),
    ],
)
def test_integration_protocol_shape_comes_from_server_config(
    config_name: str,
    control_dof: int,
    joint_order: str,
) -> None:
    server_config = load_config(str(CONFIG_DIR / config_name))
    server_config["components"]["robot"]["control_dof"] = control_dof
    server_config["components"]["robot"]["joint_order"] = joint_order

    runtime_config = RuntimeConfig.from_mapping(server_config)
    manifest = runtime_config.runtime_context["manifest"]

    assert manifest["components"][0]["dof"] == control_dof
    assert manifest["metadata"]["joint_order"] == joint_order


@pytest.mark.parametrize("control_dof", ["8", 0, -1, True])
def test_integration_rejects_invalid_resolved_control_dof(
    control_dof: object,
) -> None:
    server_config = load_config(
        str(CONFIG_DIR / "meta_quest3_franka_fr3_right_server.yaml")
    )
    server_config["components"]["robot"]["control_dof"] = control_dof

    with pytest.raises(
        ValueError,
        match="components.robot.dof must resolve to a positive integer",
    ):
        RuntimeConfig.from_mapping(server_config)
