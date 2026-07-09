"""Tests for the shared robot controller helper."""

from __future__ import annotations

from rynnrcp.robot import BaseRobotController


class MockRobotController(BaseRobotController):
    pass


def test_base_robot_controller_only_provides_logger() -> None:
    robot = MockRobotController()

    assert robot.logger.name == "MockRobotController"


def test_base_robot_controller_protocol_methods_are_overridable_templates() -> None:
    robot = MockRobotController()

    assert robot.get_health() == {"errors": [], "warnings": []}
    for method in (
        robot.get_joint_positions,
        lambda: robot.set_joint_positions({"joint_positions": []}),
        robot.get_ee_pose,
        lambda: robot.set_ee_pose({}),
    ):
        try:
            method()
        except NotImplementedError:
            pass
        else:
            raise AssertionError("unused protocol method template should raise NotImplementedError")
