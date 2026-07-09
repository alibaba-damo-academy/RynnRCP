"""Runtime config checks for the Aero Hand robot package."""

from __future__ import annotations

from pathlib import Path

from rynnrcp.config.runtime_config import RuntimeConfig
from rynnrcp.config.runner_config import build_runner_config
from rynnrcp_robot_aero_hand.controller import _step_toward


CONFIG_DIR = Path(__file__).resolve().parents[2] / "robots" / "aero_hand" / "rynnrcp_robot_aero_hand" / "config"


def test_aero_hand_single_runtime_exposes_one_robot_vector() -> None:
    runtime_config = RuntimeConfig.load(str(CONFIG_DIR / "aero_hand_single_server.yaml"))
    runner_config = build_runner_config(runtime_config)

    assert runner_config.runner_names == ["robot"]
    assert [spec.name for spec in runner_config.input_specs] == [
        "observation.robot.joint_state",
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

    assert runner_config.runner_names == ["robot"]
    assert [spec.name for spec in runner_config.input_specs] == [
        "observation.robot.joint_state",
    ]
    assert [spec.name for spec in runner_config.output_specs] == [
        "action.robot.joint_position",
    ]
    init_args = runner_config.output_specs[0].params["init_args"]
    assert init_args["mode"] == "dual"
    assert "left_port" in init_args
    assert "right_port" in init_args


def test_aero_hand_interpolation_steps_toward_target() -> None:
    next_positions, reached = _step_toward([0.0, 1.0], [1.0, 0.5], 0.2)
    assert next_positions == [0.2, 0.8]
    assert reached is False

    next_positions, reached = _step_toward([0.95, 0.51], [1.0, 0.5], 0.2)
    assert next_positions == [1.0, 0.5]
    assert reached is True
