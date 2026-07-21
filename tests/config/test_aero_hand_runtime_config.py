"""Runtime config checks for the Aero Hand robot package."""

from __future__ import annotations

from pathlib import Path

import pytest

from rynnrcp.config.runtime_config import RuntimeConfig
from rynnrcp.config.runner_config import build_runner_config
from rynnrcp_robot_aero_hand.controller import _step_toward


CONFIG_DIR = Path(__file__).resolve().parents[2] / "robots" / "tetheria_aerohand" / "rynnrcp_robot_aero_hand" / "config"


def test_aero_hand_single_runtime_exposes_one_robot_vector() -> None:
    runtime_config = RuntimeConfig.load(str(CONFIG_DIR / "aero_hand_single_server.yaml"))
    runner_config = build_runner_config(runtime_config)

    assert runner_config.runner_names == ["robot", "front"]
    assert [spec.name for spec in runner_config.input_specs] == [
        "observation.robot.joint_state",
        "observation.front.image",
    ]
    assert [spec.name for spec in runner_config.output_specs] == [
        "action.robot.joint_position",
    ]
    init_args = runner_config.output_specs[0].params["init_args"]
    assert init_args["mode"] == "single"
    assert "port" in init_args


def test_aero_hand_dual_runtime_exposes_one_robot_vector() -> None:
    runtime_config = RuntimeConfig.load(str(CONFIG_DIR / "aero_hand_dual_server.yaml"))
    runner_config = build_runner_config(runtime_config)

    assert runner_config.runner_names == ["robot", "front"]
    assert [spec.name for spec in runner_config.input_specs] == [
        "observation.robot.joint_state",
        "observation.front.image",
    ]
    assert [spec.name for spec in runner_config.output_specs] == [
        "action.robot.joint_position",
    ]
    init_args = runner_config.output_specs[0].params["init_args"]
    assert init_args["mode"] == "dual"
    assert "left_port" in init_args
    assert "right_port" in init_args


def test_aero_hand_single_hand_master_exposes_seven_dof_observation_only() -> None:
    runtime_config = RuntimeConfig.load(str(CONFIG_DIR / "aero_hand_single_hand_master_server.yaml"))
    runner_config = build_runner_config(runtime_config)

    assert runner_config.runner_names == ["robot"]
    assert [spec.name for spec in runner_config.input_specs] == ["observation.robot.joint_state"]
    assert runner_config.output_specs == []
    assert runtime_config.server_config["components"]["robot"]["embodiment_type"] == "single_dexterous_hand"
    assert runtime_config.runtime_context["manifest"]["embodiment_type"] == "single_dexterous_hand"
    assert "Leader" in runtime_config.runtime_context["manifest"]["robot_name"]
    init_args = runner_config.inputs[0].params["init_args"]
    assert init_args["mode"] == "single"
    assert init_args["side"] == "auto"
    assert init_args["camera_index"] == 0
    assert init_args["frame_server_port"] == 28412


def test_aero_hand_dual_hand_master_exposes_left_then_right_vector() -> None:
    runtime_config = RuntimeConfig.load(str(CONFIG_DIR / "aero_hand_dual_hand_master_server.yaml"))
    runner_config = build_runner_config(runtime_config)

    assert runner_config.runner_names == ["robot"]
    assert [spec.name for spec in runner_config.input_specs] == ["observation.robot.joint_state"]
    assert runner_config.output_specs == []
    assert runtime_config.server_config["components"]["robot"]["embodiment_type"] == "dual_dexterous_hand"
    assert runtime_config.runtime_context["manifest"]["embodiment_type"] == "dual_dexterous_hand"
    assert "Leader" in runtime_config.runtime_context["manifest"]["robot_name"]
    assert runner_config.inputs[0].params["init_args"]["mode"] == "dual"


def test_aero_hand_interpolation_steps_toward_target() -> None:
    next_positions, reached = _step_toward([0.0, 1.0], [1.0, 0.5], 0.2)
    assert next_positions == pytest.approx([0.18, 0.91])
    assert reached is False

    next_positions, reached = _step_toward([0.95, 0.51], [1.0, 0.5], 0.2)
    assert next_positions == pytest.approx([0.959, 0.5082])
    assert reached is False
