from __future__ import annotations

import builtins
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from rynnrcp.config.validator import ConfigValidator
from rynnrcp.config.runner_config import build_runner_config
from rynnrcp.config.runtime_config import RuntimeConfig


ROBOT_ROOT = Path(__file__).resolve().parents[2] / "robots" / "astribot_s1"
sys.path.insert(0, str(ROBOT_ROOT))

from rynnrcp_robot_astribot_s1.controller import (  # noqa: E402
    JOINT_COUNT,
    LEFT_GRIPPER_NAME,
    PART_DOFS,
    PART_NAMES,
    RIGHT_GRIPPER_NAME,
    SDK_WHOLE_BODY_DOFS,
    AstribotS1Controller,
)


class FakeInterface:
    def __init__(self) -> None:
        self.shutdown_called = False

    def shutdown(self) -> None:
        self.shutdown_called = True


class FakeAstribot:
    whole_body_names = [
        "astribot_chassis",
        "astribot_torso",
        "astribot_arm_left",
        "astribot_gripper_left",
        "astribot_arm_right",
        "astribot_gripper_right",
        "astribot_head",
    ]
    whole_body_dofs = list(SDK_WHOLE_BODY_DOFS)

    def __init__(self) -> None:
        self.kwargs = {}
        self.is_alive = True
        self.astribot_interface = FakeInterface()
        self.position_calls = []
        self.velocity_calls = []
        self.force_calls = []
        self.camera_activated = False
        self.repeated_position_command = threading.Event()

    def get_control_rights_status(self):
        return True

    def get_current_joints_position(self, names):
        if names == ["astribot_chassis"]:
            return [[1.0, 2.0, 0.5]]
        if names == [LEFT_GRIPPER_NAME]:
            return [[25.0]]
        if names == [RIGHT_GRIPPER_NAME]:
            return [[75.0]]
        if len(names) == 1 and names[0] in PART_NAMES:
            dof = PART_DOFS[PART_NAMES.index(names[0])]
            return [list(range(dof))]
        assert names == list(PART_NAMES)
        result = [list(range(dof)) for dof in PART_DOFS]
        result[2] = [25.0]
        result[4] = [75.0]
        return result

    def get_current_joints_velocity(self, names):
        if names == ["astribot_chassis"]:
            return [[0.1, 0.2, 0.3]]
        if names == [LEFT_GRIPPER_NAME]:
            return [[50.0]]
        if names == [RIGHT_GRIPPER_NAME]:
            return [[25.0]]
        assert names == list(PART_NAMES)
        result = [[0.25] * dof for dof in PART_DOFS]
        result[2] = [50.0]
        result[4] = [25.0]
        return result

    def get_desired_joints_position(self, names):
        assert names == ["astribot_chassis"]
        return [[1.0, 2.0, 0.5]]

    def get_joints_position_limit(self, names):
        assert names == list(PART_NAMES)
        lower, upper = (
            [[-float(dof)] * dof for dof in PART_DOFS],
            [[float(dof)] * dof for dof in PART_DOFS],
        )
        lower[2] = lower[4] = [0.0]
        upper[2] = upper[4] = [100.0]
        return lower, upper

    def set_joints_position(self, *args, **kwargs):
        self.position_calls.append((args, kwargs))
        if len(self.position_calls) >= 2:
            self.repeated_position_command.set()

    def set_joints_velocity(self, *args, **kwargs):
        self.velocity_calls.append((args, kwargs))

    def set_effector_max_force(self, *args):
        self.force_calls.append(args)

    def activate_camera(self):
        self.camera_activated = True

    def move_to_home(self):
        return "home ok"

    def stop_robot(self):
        return "stop ok"

    def restart_robot(self):
        return "restart ok"


@pytest.fixture
def controller(tmp_path, monkeypatch):
    fake = FakeAstribot()

    def create(**kwargs):
        fake.kwargs = kwargs
        return fake

    module = SimpleNamespace(Astribot=create)
    monkeypatch.setattr(
        "rynnrcp_robot_astribot_s1.controller.importlib.import_module", lambda name: module
    )
    result = AstribotS1Controller(sdk_root=str(tmp_path))
    result.start()
    yield result, fake
    result.shutdown()


def test_start_and_joint_mapping(controller):
    adapter, fake = controller

    state = adapter.get_joint_positions()
    assert len(state["joint_positions"]) == JOINT_COUNT == 22
    assert state["joint_positions"][:6] == [0.0, 1.0, 2.0, 3.0, 0.0, 1.0]
    assert state["joint_positions"][11] == 0.25
    assert state["joint_positions"][19] == 0.75
    assert state["joint_velocities"][11] == 0.5
    assert state["joint_velocities"][19] == 0.25
    assert fake.camera_activated is True

    target = [index / 10.0 for index in range(JOINT_COUNT)]
    target[11] = 0.25
    target[19] = 0.75
    assert adapter.set_joint_positions({"joint_positions": target}) == {"joint_positions": target}
    command = adapter._streaming_command
    assert command is not None
    assert [len(part) for part in command.target] == list(PART_DOFS)
    assert command.target[2] == [25.0]
    assert command.target[4] == [75.0]
    assert command.kwargs == {
        "control_way": "filter",
        "use_wbc": False,
        "add_default_torso": False,
    }
    assert fake.kwargs["high_control_rights"] is False
    assert adapter.has_control_rights() is True

    invalid = list(target)
    invalid[11] = 1.01
    with pytest.raises(ValueError, match=r"joint_positions\[11\]"):
        adapter.set_joint_positions({"joint_positions": invalid})

    velocity = [0.1] * JOINT_COUNT
    velocity[11] = 0.5
    velocity[19] = 0.25
    assert adapter.set_joint_velocities({"joint_velocities": velocity}) == {
        "joint_velocities": velocity
    }
    args, _ = fake.velocity_calls[-1]
    assert args[1][2] == [50.0]
    assert args[1][4] == [25.0]


def test_joint_limits_and_single_joint_mapping(controller):
    adapter, fake = controller

    limits = adapter.get_joint_limits()
    assert len(limits["lower"]) == len(limits["upper"]) == JOINT_COUNT == 22
    assert limits["lower"][11] == limits["lower"][19] == 0.0
    assert limits["upper"][11] == limits["upper"][19] == 1.0

    result = adapter.set_single_joint_position(5, 0.125)
    assert result == {
        "index": 5,
        "part": "astribot_arm_left",
        "part_index": 1,
        "position": 0.125,
    }
    assert fake.repeated_position_command.wait(timeout=0.2)
    args, kwargs = fake.position_calls[-1]
    assert args[0] == ["astribot_arm_left"]
    assert args[1][0][0] == 0.0
    assert 0.125 < args[1][0][1] < 1.0
    assert args[1][0][2:] == [2.0, 3.0, 4.0, 5.0, 6.0]
    assert kwargs == {
        "control_way": "direct",
        "use_wbc": False,
        "add_default_torso": False,
    }
    assert adapter._streaming_command.target[0][1] == pytest.approx(0.125)

    with pytest.raises(ValueError, match="gripper"):
        adapter.set_single_joint_position(11, 0.0)

    adapter.clear_command()
    call_count = len(fake.position_calls)
    time.sleep(0.03)
    assert len(fake.position_calls) == call_count


def test_chassis_and_gripper_mapping(controller):
    adapter, fake = controller

    assert adapter.get_left_gripper_state() == {
        "position": 0.25,
        "velocity": 0.5,
        "sdk_joint_position": 25.0,
        "sdk_joint_velocity": 50.0,
    }
    assert adapter.get_right_gripper_state() == {
        "position": 0.75,
        "velocity": 0.25,
        "sdk_joint_position": 75.0,
        "sdk_joint_velocity": 25.0,
    }

    command = adapter.set_base_velocity(
        {"linear_vel": [1.0, -1.0, 0.0], "angular_vel": [0.0, 0.0, 2.0]}
    )
    assert command == {"linear_vel": [0.3, -0.3, 0.0], "angular_vel": [0.0, 0.0, 0.5]}
    assert adapter._streaming_command.names == ["astribot_chassis"]
    assert adapter._streaming_command.target[0] == pytest.approx(
        [1.006, 1.994, 0.51]
    )

    result = adapter.set_left_gripper({"position": 0.25, "force": 20.0})
    assert result == {"position": 0.25, "force": 20.0}
    assert fake.force_calls[-1] == (["astribot_gripper_left"], [20.0])
    assert adapter._streaming_command.names == ["astribot_gripper_left"]
    assert adapter._streaming_command.target == [[25.0]]
    assert adapter._streaming_command.kwargs["control_way"] == "direct"


def test_shutdown_and_health(controller):
    adapter, fake = controller
    assert adapter.get_health() == {"errors": [], "warnings": []}
    adapter.shutdown()
    assert fake.astribot_interface.shutdown_called is True
    assert adapter.get_health()["warnings"][0]["code"] == "astribot_s1.not_started"


def test_configs_validate():
    config_dir = ROBOT_ROOT / "rynnrcp_robot_astribot_s1" / "config"
    for filename in (
        "robot_integration.yaml",
        "astribot_s1_server.yaml",
        "astribot_s1_rynnbot_app.yaml",
    ):
        mapping = yaml.safe_load((config_dir / filename).read_text(encoding="utf-8"))
        ConfigValidator.validate_source(mapping)


@pytest.mark.parametrize(
    ("mode", "expected_input", "expected_high_control"),
    [("read_only", "", False), ("force", "yes", True)],
)
def test_configure_control_rights_modes_do_not_wait_for_terminal(
    tmp_path, monkeypatch, mode, expected_input, expected_high_control
):
    fake = FakeAstribot()
    received_input = []

    def create(**kwargs):
        fake.kwargs = kwargs
        received_input.append(input())
        return fake

    module = SimpleNamespace(Astribot=create)
    original_input = builtins.input
    monkeypatch.setattr(
        "rynnrcp_robot_astribot_s1.controller.importlib.import_module",
        lambda name: module,
    )
    adapter = AstribotS1Controller(
        sdk_root=str(tmp_path), control_rights_mode=mode
    )
    adapter.start()
    adapter.shutdown()

    assert received_input == [expected_input]
    assert fake.kwargs["high_control_rights"] is expected_high_control
    assert builtins.input is original_input


def test_runtime_config_exposes_robot_and_three_camera_runners():
    config_path = (
        ROBOT_ROOT
        / "rynnrcp_robot_astribot_s1"
        / "config"
        / "astribot_s1_server.yaml"
    )
    runtime = RuntimeConfig.load(str(config_path))
    runners = build_runner_config(runtime)

    assert runners.runner_names == [
        "robot",
        "head_camera",
        "left_wrist_camera",
        "right_wrist_camera",
    ]
    assert [item.name for item in runners.inputs] == [
        "observation.robot.joint_state",
        "observation.robot.chassis_state",
        "observation.robot.left_gripper_state",
        "observation.robot.right_gripper_state",
        "observation.head_camera.image",
        "observation.left_wrist_camera.image",
        "observation.right_wrist_camera.image",
    ]
    assert len(runners.outputs) == 8
