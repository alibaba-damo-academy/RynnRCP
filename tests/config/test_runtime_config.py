"""Tests for direct Runtime initialization from source configs."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import yaml

from rynnrcp.config.loader import load_config
from rynnrcp.config.runtime_config import RuntimeConfig
from rynnrcp.config.runner_config import build_runner_config


SO101_CONFIG_DIR = (
    Path(__file__).resolve().parents[2]
    / "robots"
    / "so101"
    / "rynnrcp_robot_so101"
    / "config"
)


def test_loader_returns_server_config_without_expansion() -> None:
    config = load_config(str(SO101_CONFIG_DIR / "so101_follower_server.yaml"))

    assert config["config_type"] == "rynnrcp_server_config"
    assert config["manifest"]["robot_id"] == "so101_follower"
    assert "runners" not in config
    assert "robot_type" not in config


def test_leader_runtime_skips_camera_runner_when_cameras_are_absent() -> None:
    runtime_config = RuntimeConfig.load(str(SO101_CONFIG_DIR / "so101_leader_server.yaml"))
    runner_config = build_runner_config(runtime_config)

    assert runtime_config.runner_mode == "process"
    assert runtime_config.runtime_context["manifest"]["robot_id"] == "so101_leader"
    assert runner_config.runner_names == ["robot"]
    assert runner_config.capabilities == {
        "observations": True,
        "actions": False,
        "health": True,
        "resources": True,
        "data_collection": False,
        "policy_service": False,
    }

    input_specs = runner_config.input_specs
    output_specs = runner_config.output_specs
    assert [spec.name for spec in input_specs] == ["observation.robot.joint_state"]
    assert output_specs == []
    assert input_specs[0].channel == "observation.robot.joint_state"
    assert input_specs[0].object_name == "observation.robot.joint_state"
    assert "plugins" not in runtime_config.server_config


def test_full_runtime_has_robot_action_and_enabled_cameras() -> None:
    runtime_config = RuntimeConfig.load(str(SO101_CONFIG_DIR / "so101_follower_server.yaml"))
    runner_config = build_runner_config(runtime_config)

    assert runner_config.runner_names == ["robot", "front", "wrist"]
    assert runner_config.capabilities == {
        "observations": True,
        "actions": True,
        "health": True,
        "resources": True,
        "data_collection": True,
        "policy_service": False,
    }

    outputs = {spec.name: spec for spec in runner_config.output_specs}
    assert {
        "action.robot.joint_position",
        "action.robot.home",
        "action.robot.move_to",
    }.issubset(outputs)
    robot_action = outputs["action.robot.joint_position"]
    assert robot_action.channel == "action.robot.joint_position"
    assert robot_action.params["action_consume_mode"] == "queue"
    assert robot_action.params["init_args"]["role"] == "follower"
    assert robot_action.params["init_args"]["port"] == runtime_config.server_config["components"]["robot"]["port"]
    move_to_action = outputs["action.robot.move_to"]
    assert move_to_action.params["method_name"] == "move_to"
    assert move_to_action.params["input_schema"]["fields"]["joint_positions"]["type"] == "array"

    cameras = {spec.object_name: spec for spec in runner_config.input_specs if spec.protocol == "port"}
    assert set(cameras) == {"observation.front.image", "observation.wrist.image"}
    assert cameras["observation.front.image"].info["device_id"] == 0
    assert cameras["observation.wrist.image"].info["device_id"] == 1
    assert cameras["observation.front.image"].channel == "observation.front.image"
    assert cameras["observation.front.image"].msg_size == 16384


def test_joint_state_msg_size_depends_on_type_not_protocol_name(tmp_path: Path) -> None:
    config = yaml.safe_load((SO101_CONFIG_DIR / "so101_leader_server.yaml").read_text())
    config = deepcopy(config)
    config["components"]["arm"] = config["components"].pop("robot")
    config["components"]["arm"]["role"] = "leader"

    integration = yaml.safe_load((SO101_CONFIG_DIR / "robot_integration.yaml").read_text())
    integration = deepcopy(integration)
    integration["components"][0]["name"] = "arm"
    integration["components"][0]["enabled"] = "${components.arm.enabled}"
    integration["components"][0]["health"]["source"]["init"]["port"] = "${components.arm.port}"
    integration["components"][0]["health"]["source"]["init"]["role"] = "${components.arm.role}"
    integration["components"][0]["observations"][0]["source"]["init"]["port"] = "${components.arm.port}"
    integration["components"][0]["observations"][0]["source"]["init"]["role"] = "${components.arm.role}"
    integration["components"][0]["actions"] = []
    integration_path = tmp_path / "robot_integration.yaml"
    integration_path.write_text(yaml.safe_dump(integration), encoding="utf-8")
    config["integration"] = {"config": str(integration_path)}

    runtime_config = RuntimeConfig.from_mapping(config)
    runner_config = build_runner_config(runtime_config)

    assert runner_config.input_specs[0].object_name == "observation.arm.joint_state"
    assert runner_config.input_specs[0].msg_size == 4096


def test_disabled_server_camera_is_not_initialized() -> None:
    config = yaml.safe_load((SO101_CONFIG_DIR / "so101_follower_server.yaml").read_text())
    config = deepcopy(config)
    config["components"]["wrist_camera"]["enabled"] = False

    runtime_config = RuntimeConfig.from_mapping(config)
    runner_config = build_runner_config(runtime_config)
    camera_specs = [
        spec
        for spec in runner_config.input_specs
        if spec.protocol == "port"
    ]

    assert [spec.object_name for spec in camera_specs] == ["observation.front.image"]
