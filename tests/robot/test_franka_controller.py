from __future__ import annotations

import sys
import threading
import time
import types
from typing import Any

import pytest

from rynnrcp_robot_franka_fr3.controller import FrankaController


INITIAL_Q = [0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0]
IDENTITY_POSE = [
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.4, 0.1, 0.5, 1.0,
]


class FakeHardware:
    instances: list["FakeHardware"] = []

    def __init__(self, *args: Any) -> None:
        self.args = args
        self.connected = False
        self.control_enabled = False
        self.gripper_connected = False
        self.watchdog_triggered = False
        self.control_error = ""
        self.q = list(INITIAL_Q)
        self.target: list[float] | None = None
        self.ee_target: tuple[list[float], list[float]] | None = None
        self.gripper_calls: list[tuple[Any, ...]] = []
        self.gripper_read_count = 0
        self.gripper_read_delay_s = 0.0
        self.gripper_read_started = threading.Event()
        self.recovery_calls = 0
        self.start_control_calls = 0
        self.__class__.instances.append(self)

    def connect(self) -> None:
        self.connected = True
        self.gripper_connected = bool(self.args[1])

    def disconnect(self) -> None:
        self.control_enabled = False
        self.connected = False

    def start_control(self) -> None:
        if not self.connected:
            raise RuntimeError("not connected")
        self.start_control_calls += 1
        self.control_enabled = True

    def stop_control(self) -> None:
        self.control_enabled = False

    def set_joint_target(self, positions: list[float]) -> None:
        if not self.control_enabled:
            raise RuntimeError("Franka control is disabled")
        for position, low, high in zip(positions, self.args[9], self.args[10]):
            if not low <= position <= high:
                raise ValueError("Franka joint target is outside configured limits")
        self.target = list(positions)

    def set_ee_target(
        self, position: list[float], quaternion: list[float]
    ) -> None:
        if not self.control_enabled:
            raise RuntimeError("Franka control is disabled")
        self.ee_target = (list(position), list(quaternion))

    def get_state(self) -> dict[str, list[float]]:
        return {
            "q": self.q,
            "dq": [0.0] * 7,
            "tau_J": [0.0] * 7,
            "O_T_EE": IDENTITY_POSE,
        }

    def get_gripper_state(self) -> dict[str, Any]:
        self.gripper_read_count += 1
        if self.gripper_read_delay_s > 0.0:
            self.gripper_read_started.set()
            time.sleep(self.gripper_read_delay_s)
        return {
            "width": 0.08,
            "max_width": 0.08,
            "is_grasped": False,
            "temperature": 30,
        }

    def gripper_move(self, width: float, speed: float) -> None:
        self.gripper_calls.append(("move", width, speed))

    def gripper_grasp(
        self,
        width: float,
        speed: float,
        force: float,
        epsilon_inner: float,
        epsilon_outer: float,
    ) -> None:
        self.gripper_calls.append(
            ("grasp", width, speed, force, epsilon_inner, epsilon_outer)
        )

    def automatic_error_recovery(self) -> None:
        if self.control_enabled:
            raise RuntimeError("stop Franka control before automatic error recovery")
        self.recovery_calls += 1

    def status(self) -> dict[str, Any]:
        return {
            "connected": self.connected,
            "control_enabled": self.control_enabled,
            "gripper_connected": self.gripper_connected,
            "realtime_enforce": bool(self.args[2]),
            "watchdog_triggered": self.watchdog_triggered,
            "control_error": self.control_error,
        }


@pytest.fixture(autouse=True)
def fake_native(monkeypatch: pytest.MonkeyPatch) -> None:
    FakeHardware.instances.clear()
    module = types.SimpleNamespace(FrankaHardware=FakeHardware)
    monkeypatch.setitem(sys.modules, "rynnrcp_robot_franka_fr3._franka_native", module)


def _controller(**kwargs: Any) -> FrankaController:
    return FrankaController(robot_ip="192.168.0.110", robot_id="test_fr3", **kwargs)


def _wait_for_gripper(native: FakeHardware, count: int = 1) -> None:
    deadline = time.monotonic() + 0.5
    while len(native.gripper_calls) < count and time.monotonic() < deadline:
        time.sleep(0.001)
    assert len(native.gripper_calls) >= count


def test_start_runs_single_control_loop_and_joint_observation_has_seven_dimensions() -> None:
    controller = _controller()
    controller.start()
    native = FakeHardware.instances[-1]

    assert native.connected is True
    assert native.control_enabled is True
    assert native.start_control_calls == 1
    assert native.gripper_calls == []
    assert controller.get_joint_positions() == {
        "joint_positions": INITIAL_Q,
        "joint_velocities": [0.0] * 7,
    }

    controller.shutdown()


def test_joint_action_updates_running_loop_without_moving_gripper() -> None:
    controller = _controller()
    controller.start()
    native = FakeHardware.instances[-1]
    target = [0.1, 0.1, -0.1, -1.1, 0.1, 1.1, -0.1]

    assert controller.set_joint_positions({"joint_positions": target}) == {
        "joint_positions": target
    }

    assert native.control_enabled is True
    assert native.start_control_calls == 1
    assert native.target == target
    assert native.gripper_calls == []
    assert native.args[3:6] == (0.25, 0.5, 2.5)
    assert native.args[6:8] == (0.05, 0.25)
    controller.shutdown()


def test_combined_control_action_splits_arm_and_gripper() -> None:
    controller = _controller(gripper_command_deadband=0.0)
    controller.start()
    native = FakeHardware.instances[-1]
    arm_target = [0.1, 0.1, -0.1, -1.1, 0.1, 1.1, -0.1]
    target = [*arm_target, 0.25]

    assert controller.get_control_positions() == {
        "joint_positions": [*INITIAL_Q, 1.0],
        "joint_velocities": [0.0] * 8,
    }
    assert controller.set_control_positions(
        {"joint_positions": target}
    ) == {"joint_positions": target}
    _wait_for_gripper(native)

    assert native.target == arm_target
    assert native.gripper_calls == [("move", 0.02, 0.05)]
    controller.shutdown()


def test_joint_observation_uses_cached_gripper_without_blocking() -> None:
    controller = _controller()
    controller.start()
    native = FakeHardware.instances[-1]
    native.gripper_read_delay_s = 0.1
    assert native.gripper_read_started.wait(timeout=0.2)

    started = time.perf_counter()
    state = controller.get_control_positions()
    elapsed = time.perf_counter() - started

    assert elapsed < 0.02
    assert state["joint_positions"] == [*INITIAL_Q, 1.0]
    controller.shutdown()


def test_separate_gripper_action_closes_and_opens() -> None:
    controller = _controller(gripper_command_deadband=0.0)
    controller.start()
    native = FakeHardware.instances[-1]

    controller.set_gripper({"position": 0.0})
    _wait_for_gripper(native)
    controller.set_gripper({"position": 1.0})
    _wait_for_gripper(native, count=2)

    assert native.gripper_calls == [
        ("grasp", 0.0, 0.05, 20.0, 0.01, 0.07),
        ("move", 0.08, 0.05),
    ]
    controller.shutdown()


def test_home_uses_configured_seven_joint_target() -> None:
    home = [0.0, -0.8, 0.0, -2.2, 0.0, 1.5, 0.7]
    controller = _controller(home_joint_positions=home)
    controller.start()
    native = FakeHardware.instances[-1]

    assert controller.home({}) == {"joint_positions": home}
    assert native.target == home
    assert native.gripper_calls == []
    controller.shutdown()


@pytest.mark.parametrize(
    "target, message",
    [
        ([10.0, *INITIAL_Q[1:]], "joint 1"),
    ],
)
def test_joint_target_limits(target: list[float], message: str) -> None:
    controller = _controller()
    controller.start()
    with pytest.raises(ValueError, match=message):
        controller.set_joint_positions({"joint_positions": target})
    controller.shutdown()


@pytest.mark.parametrize("position", [-0.1, 1.1])
def test_gripper_target_limits(position: float) -> None:
    controller = _controller()
    controller.start()
    with pytest.raises(ValueError, match="position"):
        controller.set_gripper({"position": position})
    controller.shutdown()


def test_ee_pose_observation_and_action_use_xyzw() -> None:
    controller = _controller()
    controller.start()
    native = FakeHardware.instances[-1]

    assert controller.get_ee_pose() == {
        "position": [0.4, 0.1, 0.5],
        "orientation_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    result = controller.set_ee_pose(
        {
            "position": [0.5, 0.0, 0.4],
            "orientation_quat_xyzw": [0.0, 0.0, 0.0, 2.0],
        }
    )

    assert result == {
        "position": [0.5, 0.0, 0.4],
        "orientation_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    assert native.ee_target == (
        [0.5, 0.0, 0.4],
        [0.0, 0.0, 0.0, 1.0],
    )
    controller.shutdown()


def test_health_reports_watchdog_and_native_error() -> None:
    controller = _controller(realtime_enforce=False)
    controller.start()
    native = FakeHardware.instances[-1]
    native.watchdog_triggered = True
    native.control_error = "network failure"

    health = controller.get_health()
    assert {item["code"] for item in health["warnings"]} == {
        "franka.target_timeout",
        "franka.realtime_not_enforced",
    }
    assert [item["code"] for item in health["errors"]] == ["franka.control_error"]
    controller.shutdown()


def test_previous_control_error_must_be_recovered_before_restart() -> None:
    controller = _controller()
    controller.start()
    native = FakeHardware.instances[-1]
    native.control_enabled = False
    native.control_error = "joint motion generator velocity discontinuity"

    with pytest.raises(RuntimeError, match="previous control session failed"):
        controller.set_joint_positions({"joint_positions": INITIAL_Q})

    assert native.control_enabled is False
    controller.shutdown()


def test_error_recovery_restarts_single_control_loop() -> None:
    controller = _controller()
    controller.start()
    native = FakeHardware.instances[-1]
    native.control_enabled = False
    native.control_error = "reflex"

    assert controller.automatic_error_recovery({}) == {"recovered": True}

    assert native.recovery_calls == 1
    assert native.control_enabled is True
    assert native.start_control_calls == 2
    controller.shutdown()


def test_watchdog_hold_accepts_next_action_without_restarting_loop() -> None:
    controller = _controller()
    controller.start()
    native = FakeHardware.instances[-1]
    native.watchdog_triggered = True

    controller.set_joint_positions({"joint_positions": INITIAL_Q})

    assert native.start_control_calls == 1
    assert native.target == INITIAL_Q
    controller.shutdown()
