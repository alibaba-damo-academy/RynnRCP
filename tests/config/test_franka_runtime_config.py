from __future__ import annotations

from pathlib import Path

from rynnrcp.config.runner_config import build_runner_config
from rynnrcp.config.runtime_config import RuntimeConfig


CONFIG = (
    Path(__file__).resolve().parents[2]
    / "robots"
    / "franka_fr3"
    / "rynnrcp_robot_franka_fr3"
    / "config"
    / "franka_fr3_server.yaml"
)


def test_franka_runtime_exposes_joint_position_and_home_actions() -> None:
    runtime_config = RuntimeConfig.load(str(CONFIG))
    runner_config = build_runner_config(runtime_config)

    assert [spec.name for spec in runner_config.input_specs] == [
        "observation.robot.joint_state",
    ]
    assert [spec.name for spec in runner_config.output_specs] == [
        "action.robot.joint_position",
        "action.robot.home",
    ]
    joint_action = runner_config.output_specs[0]
    assert joint_action.params["init_args"]["realtime_enforce"] is True
    assert joint_action.params["init_args"]["max_joint_velocity_rad_s"] == 0.25
    assert joint_action.params["init_args"]["max_joint_acceleration_rad_s2"] == 0.5
    assert joint_action.params["init_args"]["max_joint_jerk_rad_s3"] == 2.5
    assert joint_action.params["init_args"]["max_cartesian_velocity_m_s"] == 0.05
    assert joint_action.params["init_args"]["max_cartesian_rotation_rad_s"] == 0.25
    assert joint_action.params["init_args"]["robot_ip"] == "192.168.0.110"
    assert joint_action.params["init_args"]["with_gripper"] is True
    assert len(joint_action.params["init_args"]["home_joint_positions"]) == 7
    assert joint_action.params["method_name"] == "set_control_positions"
    assert runtime_config.integration_config["components"][0]["dof"] == 8
    robot = runtime_config.integration_config["components"][0]
    assert robot["observations"][0]["frame_rate"] == 60
    assert robot["actions"][0]["frame_rate"] == 60
    assert robot["actions"][1]["frame_rate"] == 1
    cameras = runtime_config.integration_config["components"][1:]
    assert [camera["name"] for camera in cameras] == [
        "cam_arm",
        "cam_main",
        "cam_side",
    ]
    assert all(camera["type"] == "camera" for camera in cameras)
    assert all(camera["observations"][0]["type"] == "image" for camera in cameras)
    assert all(camera["observations"][0]["frame_rate"] == 30 for camera in cameras)
