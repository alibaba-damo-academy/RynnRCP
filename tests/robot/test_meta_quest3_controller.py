from __future__ import annotations

import json
import math
import socket
import threading
import time

import pytest

from rynnrcp_robot_meta_quest3.controller import (
    MetaQuest3DualController,
    MetaQuest3RightController,
)
from rynnrcp_robot_meta_quest3.joint_leader import (
    MetaQuest3DualUrdfJointController,
    MetaQuest3UrdfJointController,
)
from rynnrcp_robot_meta_quest3.web_sim import WORKSPACE_DELTA_LIMITS_M


HOME_JOINT_POSITIONS = (
    0.0,
    -0.785,
    0.0,
    -2.356,
    0.0,
    1.571,
    0.785,
)


def _controller(
    *,
    position: tuple[float, float, float],
    primary_pressed: bool = False,
    grip_pressed: bool = False,
    trigger: float = 0.0,
    timestamp: int,
) -> dict:
    return {
        "position": {
            "x": position[0],
            "y": position[1],
            "z": position[2],
        },
        "rotation": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "w": 1.0,
        },
        "input": {
            "primaryButton": primary_pressed,
            "gripPressed": grip_pressed,
            "trigger": trigger,
        },
        "timestamp": timestamp,
    }


def _payload(
    *,
    left_position: tuple[float, float, float],
    right_position: tuple[float, float, float],
    x_pressed: bool = False,
    a_pressed: bool = False,
    left_trigger: float = 0.0,
    right_trigger: float = 0.0,
    left_grip: bool = False,
    right_grip: bool = False,
    timestamp: int,
) -> dict:
    return {
        "leftController": _controller(
            position=left_position,
            primary_pressed=x_pressed,
            grip_pressed=left_grip,
            trigger=left_trigger,
            timestamp=timestamp,
        ),
        "rightController": _controller(
            position=right_position,
            primary_pressed=a_pressed,
            grip_pressed=right_grip,
            trigger=right_trigger,
            timestamp=timestamp,
        ),
    }


def _process(
    controller,
    payload: dict,
) -> None:
    controller._process_payload(
        payload,
        ("192.168.1.120", 59740),
        time.monotonic(),
    )


def test_x_and_a_calibrate_both_controllers_to_zero() -> None:
    controller = MetaQuest3DualController(
        stale_timeout_s=1.0,
    )
    neutral_left = (1.0, 2.0, 3.0)
    neutral_right = (-1.0, 2.0, 3.0)

    _process(
        controller,
        _payload(
            left_position=neutral_left,
            right_position=neutral_right,
            timestamp=1,
        ),
    )
    with pytest.raises(RuntimeError, match="Calibration required"):
        controller.get_left_ee_pose()

    _process(
        controller,
        _payload(
            left_position=neutral_left,
            right_position=neutral_right,
            x_pressed=True,
            a_pressed=True,
            timestamp=2,
        ),
    )

    assert controller.get_left_ee_pose() == {
        "position": pytest.approx([0.0, 0.0, 0.0]),
        "orientation_quat_xyzw": pytest.approx([0.0, 0.0, 0.0, 1.0]),
    }
    assert controller.get_right_ee_pose() == {
        "position": pytest.approx([0.0, 0.0, 0.0]),
        "orientation_quat_xyzw": pytest.approx([0.0, 0.0, 0.0, 1.0]),
    }
    state = controller.get_controller_state()
    assert state["calibrated"] is True
    assert state["calibration_count"] == 1


def test_calibrated_motion_uses_the_same_standard_axes_for_both_arms() -> None:
    controller = MetaQuest3DualController()
    _process(
        controller,
        _payload(
            left_position=(1.0, 2.0, 3.0),
            right_position=(-1.0, 2.0, 3.0),
            x_pressed=True,
            a_pressed=True,
            timestamp=1,
        ),
    )
    _process(
        controller,
        _payload(
            left_position=(1.2, 2.3, 3.1),
            right_position=(-1.1, 2.2, 3.4),
            left_trigger=0.25,
            right_trigger=0.75,
            timestamp=2,
        ),
    )

    assert controller.get_left_ee_pose()["position"] == pytest.approx(
        [-0.1, 0.2, 0.3]
    )
    assert controller.get_right_ee_pose()["position"] == pytest.approx(
        [-0.4, -0.1, 0.2]
    )
    assert controller.get_left_gripper_state() == {"position": 0.75}
    assert controller.get_right_gripper_state() == {"position": 0.25}


def test_x_or_a_alone_does_not_calibrate() -> None:
    controller = MetaQuest3DualController()
    _process(
        controller,
        _payload(
            left_position=(0.0, 0.0, 0.0),
            right_position=(0.0, 0.0, 0.0),
            x_pressed=True,
            a_pressed=False,
            timestamp=1,
        ),
    )
    assert controller.get_controller_state()["calibrated"] is False


def test_recalibration_requires_releasing_and_pressing_the_chord_again() -> None:
    controller = MetaQuest3DualController()
    for timestamp, pressed, position in [
        (1, True, (0.0, 0.0, 0.0)),
        (2, True, (1.0, 1.0, 1.0)),
        (3, False, (1.0, 1.0, 1.0)),
        (4, True, (1.0, 1.0, 1.0)),
    ]:
        _process(
            controller,
            _payload(
                left_position=position,
                right_position=position,
                x_pressed=pressed,
                a_pressed=pressed,
                timestamp=timestamp,
            ),
        )
    assert controller.get_controller_state()["calibration_count"] == 2
    assert controller.get_left_ee_pose()["position"] == pytest.approx(
        [0.0, 0.0, 0.0]
    )
    assert controller.get_right_ee_pose()["position"] == pytest.approx(
        [0.0, 0.0, 0.0]
    )


def test_right_controller_calibrates_with_a_only() -> None:
    controller = MetaQuest3RightController()
    _process(
        controller,
        _payload(
            left_position=(0.0, 0.0, 0.0),
            right_position=(0.0, 0.0, 0.0),
            x_pressed=False,
            a_pressed=True,
            timestamp=1,
        ),
    )
    _process(
        controller,
        _payload(
            left_position=(0.0, 0.0, 1.0),
            right_position=(0.0, 0.0, 0.3),
            timestamp=2,
        ),
    )
    assert controller.get_ee_pose()["position"] == pytest.approx(
        [-0.3, 0.0, 0.0]
    )


def test_stale_udp_data_pauses_pose_and_gripper_output() -> None:
    controller = MetaQuest3RightController(stale_timeout_s=0.05)
    _process(
        controller,
        _payload(
            left_position=(0.0, 0.0, 0.0),
            right_position=(0.0, 0.0, 0.0),
            a_pressed=True,
            timestamp=1,
        ),
    )
    controller._last_packet_at = time.monotonic() - 0.1

    with pytest.raises(RuntimeError, match="UDP data timed out"):
        controller.get_ee_pose()
    with pytest.raises(RuntimeError, match="UDP data timed out"):
        controller.get_gripper_state()


def test_fr3_right_leader_emits_eight_value_arm_and_gripper_target() -> None:
    leader = MetaQuest3UrdfJointController(
        robot_id="meta_quest3_franka_fr3_right",
        target_model="franka_fr3",
        target_urdf="package://rynnrcp_robot_meta_quest3/models/fr3.urdf",
        target_dof=7,
        has_gripper=True,
        base_link="fr3_link0",
        tip_link="fr3_hand_tcp",
        controller_side="right",
        home_joint_positions=HOME_JOINT_POSITIONS,
    )
    _process(
        leader.quest,
        _payload(
            left_position=(0.0, 0.0, 0.0),
            right_position=(0.0, 0.0, 0.0),
            a_pressed=True,
            timestamp=1,
        ),
    )
    leader.bridge.update_once()

    first_sample: dict[str, object] = {}
    reader = threading.Thread(
        target=lambda: first_sample.update(leader.get_joint_positions()),
        daemon=True,
    )
    reader.start()
    time.sleep(0.02)
    assert reader.is_alive()

    _process(
        leader.quest,
        _payload(
            left_position=(0.0, 0.0, 0.0),
            right_position=(0.0, 0.0, 0.0),
            right_grip=True,
            right_trigger=0.75,
            timestamp=2,
        ),
    )
    leader.bridge.update_once()
    reader.join(timeout=0.2)
    assert not reader.is_alive()
    assert first_sample["joint_positions"] == pytest.approx(
        [*HOME_JOINT_POSITIONS, 0.25]
    )
    _process(
        leader.quest,
        _payload(
            left_position=(0.0, 0.0, 0.0),
            right_position=(0.0, 0.0, -0.03),
            right_grip=True,
            right_trigger=0.75,
            timestamp=3,
        ),
    )
    leader.bridge.update_once()
    for _ in range(60):
        leader.simulation.advance(1.0 / 60.0)

    target = leader.get_joint_positions()["joint_positions"]
    assert len(target) == 8
    assert target[:7] != pytest.approx(HOME_JOINT_POSITIONS)
    assert target[7] == pytest.approx(0.25)
    controller_state = leader.get_controller_state()
    assert controller_state["target_model"] == "franka_fr3"
    assert controller_state["target_dof"] == 7
    assert controller_state["control_dof"] == 8
    assert controller_state["joint_order"] == "joints_then_gripper"
    assert controller_state["controller_side"] == "right"
    assert controller_state["ik"]["converged"] is True


def test_joint_leader_validates_configured_urdf_dimension() -> None:
    with pytest.raises(ValueError, match="contains 7 active joints"):
        MetaQuest3UrdfJointController(
            target_model="wrong_dimension",
            target_urdf=(
                "package://rynnrcp_robot_meta_quest3/models/fr3.urdf"
            ),
            target_dof=6,
            has_gripper=True,
            base_link="fr3_link0",
            tip_link="fr3_hand_tcp",
            controller_side="right",
            home_joint_positions=HOME_JOINT_POSITIONS,
        )


def test_joint_leader_rejects_left_controller_for_single_arm() -> None:
    with pytest.raises(
        ValueError,
        match="single-arm mode requires controller_side: right",
    ):
        MetaQuest3UrdfJointController(
            target_model="franka_fr3",
            target_urdf=(
                "package://rynnrcp_robot_meta_quest3/models/fr3.urdf"
            ),
            target_dof=7,
            has_gripper=True,
            base_link="fr3_link0",
            tip_link="fr3_hand_tcp",
            controller_side="left",
            home_joint_positions=HOME_JOINT_POSITIONS,
        )


def test_joint_leader_validates_control_dimension() -> None:
    with pytest.raises(ValueError, match="has_gripper require 8"):
        MetaQuest3UrdfJointController(
            target_model="franka_fr3",
            target_urdf=(
                "package://rynnrcp_robot_meta_quest3/models/fr3.urdf"
            ),
            target_dof=7,
            has_gripper=True,
            control_dof=7,
            base_link="fr3_link0",
            tip_link="fr3_hand_tcp",
            controller_side="right",
            home_joint_positions=HOME_JOINT_POSITIONS,
        )


def test_dual_joint_leader_publishes_two_eight_value_arm_blocks() -> None:
    arm_config = {
        "target_model": "franka_fr3",
        "target_urdf": (
            "package://rynnrcp_robot_meta_quest3/models/fr3.urdf"
        ),
        "target_dof": 7,
        "has_gripper": True,
        "base_link": "fr3_link0",
        "tip_link": "fr3_hand_tcp",
        "translation_scale": 1.0,
        "home_joint_positions": HOME_JOINT_POSITIONS,
        "workspace_delta_limits_m": (
            (-0.18, 0.40),
            (-0.50, 0.50),
            (-0.50, 0.00),
        ),
    }
    leader = MetaQuest3DualUrdfJointController(
        arms={"left": arm_config, "right": arm_config},
    )
    _process(
        leader.quest,
        _payload(
            left_position=(0.0, 0.0, 0.0),
            right_position=(0.0, 0.0, 0.0),
            x_pressed=True,
            a_pressed=True,
            timestamp=1,
        ),
    )
    leader.bridges["left"].update_once()
    leader.bridges["right"].update_once()

    first_sample: dict[str, object] = {}
    reader = threading.Thread(
        target=lambda: first_sample.update(leader.get_joint_positions()),
        daemon=True,
    )
    reader.start()
    time.sleep(0.02)
    assert reader.is_alive()

    _process(
        leader.quest,
        _payload(
            left_position=(0.0, 0.0, 0.0),
            right_position=(0.0, 0.0, 0.0),
            right_grip=True,
            left_trigger=0.25,
            right_trigger=0.75,
            timestamp=2,
        ),
    )
    leader.bridges["left"].update_once()
    leader.bridges["right"].update_once()
    reader.join(timeout=0.2)
    assert not reader.is_alive()
    assert first_sample["joint_positions"] == pytest.approx(
        [
            *HOME_JOINT_POSITIONS,
            1.0,
            *HOME_JOINT_POSITIONS,
            0.25,
        ]
    )
    _process(
        leader.quest,
        _payload(
            left_position=(0.0, 0.0, 0.0),
            right_position=(0.0, 0.0, -0.03),
            right_grip=True,
            left_trigger=0.25,
            right_trigger=0.75,
            timestamp=3,
        ),
    )
    leader.bridges["left"].update_once()
    leader.bridges["right"].update_once()
    for _ in range(60):
        leader.simulations["left"].advance(1.0 / 60.0)
        leader.simulations["right"].advance(1.0 / 60.0)

    joint_positions = leader.get_joint_positions()["joint_positions"]
    assert len(joint_positions) == 16
    assert joint_positions[:7] == pytest.approx(HOME_JOINT_POSITIONS)
    assert joint_positions[7] == pytest.approx(1.0)
    assert joint_positions[8:15] != pytest.approx(HOME_JOINT_POSITIONS)
    assert joint_positions[15] == pytest.approx(0.25)
    state = leader.get_controller_state()
    assert state["joint_order"] == (
        "left_joints_left_gripper_right_joints_right_gripper"
    )
    assert state["target_dof"] == 16


def test_joint_leader_stops_publishing_and_reanchors_after_grip_release() -> None:
    leader = MetaQuest3UrdfJointController(
        robot_id="meta_quest3_franka_fr3_right",
        target_model="franka_fr3",
        target_urdf="package://rynnrcp_robot_meta_quest3/models/fr3.urdf",
        target_dof=7,
        has_gripper=True,
        base_link="fr3_link0",
        tip_link="fr3_hand_tcp",
        controller_side="right",
        home_joint_positions=HOME_JOINT_POSITIONS,
    )
    _process(
        leader.quest,
        _payload(
            left_position=(0.0, 0.0, 0.0),
            right_position=(0.0, 0.0, 0.0),
            a_pressed=True,
            timestamp=1,
        ),
    )
    leader.bridge.update_once()
    _process(
        leader.quest,
        _payload(
            left_position=(0.0, 0.0, 0.0),
            right_position=(0.0, 0.0, 0.0),
            right_grip=True,
            timestamp=2,
        ),
    )
    leader.bridge.update_once()
    before_release = leader.get_joint_positions()["joint_positions"]

    _process(
        leader.quest,
        _payload(
            left_position=(0.0, 0.0, 0.0),
            right_position=(0.0, 0.0, -0.2),
            timestamp=3,
        ),
    )
    leader.bridge.update_once()
    for _ in range(30):
        leader.simulation.advance(1.0 / 60.0)
    resumed_sample: dict[str, object] = {}
    reader = threading.Thread(
        target=lambda: resumed_sample.update(leader.get_joint_positions()),
        daemon=True,
    )
    reader.start()
    time.sleep(0.02)
    assert reader.is_alive()
    assert leader.simulation.joint_state()["joint_positions"] == pytest.approx(
        before_release[:7]
    )

    _process(
        leader.quest,
        _payload(
            left_position=(0.0, 0.0, 0.0),
            right_position=(0.4, 0.2, -0.3),
            right_grip=True,
            timestamp=4,
        ),
    )
    leader.bridge.update_once()
    reader.join(timeout=0.2)
    assert not reader.is_alive()
    assert resumed_sample["joint_positions"] == pytest.approx(
        before_release
    )


def test_dual_joint_leader_derives_dimension_from_both_urdf_chains() -> None:
    fr3 = {
        "target_model": "franka_fr3",
        "target_urdf": "package://rynnrcp_robot_meta_quest3/models/fr3.urdf",
        "target_dof": 7,
        "has_gripper": True,
        "base_link": "fr3_link0",
        "tip_link": "fr3_hand_tcp",
        "home_joint_positions": HOME_JOINT_POSITIONS,
        "workspace_delta_limits_m": WORKSPACE_DELTA_LIMITS_M,
    }
    ur5e = {
        "target_model": "ur5e",
        "target_urdf": "package://rynnrcp_robot_meta_quest3/models/ur5e.urdf",
        "target_dof": 6,
        "has_gripper": False,
        "base_link": "base_link",
        "tip_link": "tool0",
        "home_joint_positions": (
            3.1415926,
            -1.5708,
            -1.5708,
            0.0,
            1.5708,
            0.0,
        ),
        "workspace_delta_limits_m": WORKSPACE_DELTA_LIMITS_M,
    }

    leader = MetaQuest3DualUrdfJointController(
        arms={"left": fr3, "right": ur5e},
    )

    assert leader.arm_dofs == {"left": 7, "right": 6}
    assert leader.arm_has_gripper == {"left": True, "right": False}
    assert leader.n_dof == 14
    assert leader.joint_order == (
        "left_joints_left_gripper_right_joints"
    )

    with pytest.raises(ValueError, match="configured arms require 14"):
        MetaQuest3DualUrdfJointController(
            arms={"left": fr3, "right": ur5e},
            control_dof=15,
        )


def test_single_joint_leader_rejects_mismatched_joint_order() -> None:
    with pytest.raises(ValueError, match="requires 'joints_then_gripper'"):
        MetaQuest3UrdfJointController(
            target_model="franka_fr3",
            target_urdf=(
                "package://rynnrcp_robot_meta_quest3/models/fr3.urdf"
            ),
            target_dof=7,
            has_gripper=True,
            joint_order="joints",
            base_link="fr3_link0",
            tip_link="fr3_hand_tcp",
            home_joint_positions=HOME_JOINT_POSITIONS,
        )


def test_dual_joint_leader_rejects_mismatched_joint_order() -> None:
    arm = {
        "target_model": "franka_fr3",
        "target_urdf": (
            "package://rynnrcp_robot_meta_quest3/models/fr3.urdf"
        ),
        "target_dof": 7,
        "has_gripper": True,
        "base_link": "fr3_link0",
        "tip_link": "fr3_hand_tcp",
        "home_joint_positions": HOME_JOINT_POSITIONS,
        "workspace_delta_limits_m": WORKSPACE_DELTA_LIMITS_M,
    }
    with pytest.raises(
        ValueError,
        match=(
            "requires "
            "'left_joints_left_gripper_right_joints_right_gripper'"
        ),
    ):
        MetaQuest3DualUrdfJointController(
            arms={"left": arm, "right": arm},
            joint_order="left_joints_right_joints",
        )


def test_right_controller_keeps_the_original_yaw_mapping() -> None:
    controller = MetaQuest3RightController()
    _process(
        controller,
        _payload(
            left_position=(0.0, 0.0, 0.0),
            right_position=(0.0, 0.0, 0.0),
            a_pressed=True,
            timestamp=1,
        ),
    )
    angle = 0.2
    payload = _payload(
        left_position=(0.0, 0.0, 0.0),
        right_position=(0.0, 0.0, 0.0),
        timestamp=2,
    )
    payload["rightController"]["rotation"] = {
        "x": 0.0,
        "y": math.sin(angle / 2.0),
        "z": 0.0,
        "w": math.cos(angle / 2.0),
    }

    _process(controller, payload)

    assert controller.get_ee_pose()["orientation_quat_xyzw"] == pytest.approx(
        [0.0, 0.0, -math.sin(angle / 2.0), math.cos(angle / 2.0)]
    )


@pytest.mark.parametrize(
    ("source_axis", "expected_quaternion"),
    [
        ("x", (0.0, -1.0, 0.0)),
        ("z", (1.0, 0.0, 0.0)),
    ],
)
def test_forward_and_lateral_rotation_axes_are_flipped(
    source_axis: str,
    expected_quaternion: tuple[float, float, float],
) -> None:
    controller = MetaQuest3RightController()
    _process(
        controller,
        _payload(
            left_position=(0.0, 0.0, 0.0),
            right_position=(0.0, 0.0, 0.0),
            a_pressed=True,
            timestamp=1,
        ),
    )
    angle = 0.2
    vector = {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": math.cos(angle / 2.0),
    }
    vector[source_axis] = math.sin(angle / 2.0)
    payload = _payload(
        left_position=(0.0, 0.0, 0.0),
        right_position=(0.0, 0.0, 0.0),
        timestamp=2,
    )
    payload["rightController"]["rotation"] = vector

    _process(controller, payload)

    expected = [
        expected_quaternion[index] * math.sin(angle / 2.0)
        for index in range(3)
    ] + [math.cos(angle / 2.0)]
    assert controller.get_ee_pose()["orientation_quat_xyzw"] == pytest.approx(
        expected
    )


def test_health_reports_waiting_calibration_and_stale_data() -> None:
    controller = MetaQuest3RightController(stale_timeout_s=0.01)
    assert {
        item["code"] for item in controller.get_health()["warnings"]
    } == {"meta_quest3.waiting"}

    controller._process_payload(
        _payload(
            left_position=(0.0, 0.0, 0.0),
            right_position=(0.0, 0.0, 0.0),
            timestamp=1,
        ),
        ("192.168.1.120", 59740),
        time.monotonic() - 1.0,
    )
    assert {
        item["code"] for item in controller.get_health()["warnings"]
    } == {
        "meta_quest3.calibration_required",
        "meta_quest3.stale",
    }


def test_coordinate_basis_must_be_orthonormal() -> None:
    with pytest.raises(ValueError, match="orthonormal"):
        MetaQuest3RightController(
            coordinate_basis=[
                [1.0, 0.0, 0.0],
                [0.0, 2.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        )


def test_udp_worker_receives_both_controllers() -> None:
    controller = MetaQuest3RightController(
        bind_host="127.0.0.1",
        udp_port=0,
        source_ip="",
    )
    controller.start()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(
            json.dumps(
                _payload(
                    left_position=(0.0, 0.0, 0.0),
                    right_position=(0.0, 0.0, 0.0),
                    timestamp=1,
                )
            ).encode("utf-8"),
            ("127.0.0.1", controller.udp_port),
        )
        deadline = time.monotonic() + 1.0
        while (
            controller.get_controller_state()["packet_count"] == 0
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        assert controller.get_controller_state()["packet_count"] == 1
    finally:
        sender.close()
        controller.shutdown()
