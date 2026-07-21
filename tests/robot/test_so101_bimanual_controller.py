from __future__ import annotations

from typing import Any

import pytest

from rynnrcp_robot_so101 import controller as controller_module


class FakeArmController:
    def __init__(self, *, port: str, robot_id: str, role: str, **_options: Any) -> None:
        self.port = port
        self.robot_id = robot_id
        self.role = role
        self.positions = [0.0] * 6 if "left" in robot_id else [1.0] * 6
        self.started = False
        self.stopped = False

    def start(self) -> None:
        self.started = True

    def shutdown(self) -> None:
        self.stopped = True

    def get_joint_positions(self) -> dict[str, list[float]]:
        return {"joint_positions": list(self.positions)}

    def set_joint_positions(self, value: dict[str, Any]) -> None:
        self.positions = [float(item) for item in value["joint_positions"]]

    def get_health(self) -> dict[str, list[dict[str, Any]]]:
        return {"errors": [], "warnings": []}

    def stop_motion(self) -> dict[str, bool]:
        return {"stopped": True}


@pytest.fixture
def bimanual(monkeypatch: pytest.MonkeyPatch) -> controller_module.SO101BimanualController:
    monkeypatch.setattr(controller_module, "SO101Controller", FakeArmController)
    return controller_module.SO101BimanualController(
        left_port="/dev/left",
        right_port="/dev/right",
        left_robot_id="test_left",
        right_robot_id="test_right",
        role="follower",
    )


def test_bimanual_controller_exposes_requested_metadata(
    bimanual: controller_module.SO101BimanualController,
) -> None:
    assert bimanual.n_dof == 12
    assert bimanual.task_keys == [
        "observation.state",
        "observation.images.front",
        "observation.images.left_wrist",
        "observation.images.right_wrist",
        "action",
    ]


def test_bimanual_state_and_action_are_left_then_right(
    bimanual: controller_module.SO101BimanualController,
) -> None:
    bimanual.start()
    assert bimanual.get_joint_positions()["joint_positions"] == [0.0] * 6 + [1.0] * 6

    target = [float(index) / 20.0 for index in range(12)]
    bimanual.set_joint_positions({"joint_positions": target})

    assert bimanual._arms["left"].positions == target[:6]
    assert bimanual._arms["right"].positions == target[6:]
    assert bimanual.get_joint_positions()["joint_positions"] == target


def test_bimanual_action_requires_exactly_twelve_values(
    bimanual: controller_module.SO101BimanualController,
) -> None:
    with pytest.raises(ValueError, match="expects 12"):
        bimanual.set_joint_positions({"joint_positions": [0.0] * 6})
