import sys
import pytest
import numpy as np
import mujoco
import pinocchio as pin
from pathlib import Path
from scipy.spatial.transform import Rotation as R

sys.path.append(str(Path(__file__).parent.parent))
from robots.lerobot.utils.ik_solver import IKSolver


# Fixed parameters
MODEL_PATH = "../../../models/0.lerobot/so100/urdf/so_arm100_base.xml"
TARGET_FRAME_NAME = "Fixed_Jaw"
TOLERANCE = 1e-3  # Position error tolerance
ROT_TOLERANCE = 1e-3  # Rotation error tolerance (radians)


class KinematicsSolver:
    def __init__(self, model_path, target_frame_name):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model)
        self.target_frame_name = target_frame_name

        # Initialize Pinocchio model
        self.pin_model, self.collision_model, self.visual_model = (
            pin.buildModelsFromMJCF(model_path)
        )
        self.pin_data = self.pin_model.createData()

        # Initialize DIFFIK
        self.ik_solver = IKSolver(
            model_path,
            self.target_frame_name,
            np.array([0.5, -1, 2.0, 0, 1, 0]),
            np.array([0.0, 0.0, 0.0]),
            False,
        )

    def forward_kinematics_pinocchio(self, joint_angles):
        """Compute forward kinematics using Pinocchio"""
        q = np.concatenate([joint_angles, np.zeros(self.model.nq - len(joint_angles))])
        pin.forwardKinematics(self.pin_model, self.pin_data, q)
        pin.updateFramePlacements(self.pin_model, self.pin_data)
        pin_pose = self.pin_data.oMf[self.pin_model.getFrameId(self.target_frame_name)]
        rotation = R.from_matrix(pin_pose.rotation).as_quat()
        return {
            "position": self.pin_data.oMf[
                self.pin_model.getFrameId(self.target_frame_name)
            ].translation.copy(),
            "orientation": np.array(
                [rotation[3], rotation[0], rotation[1], rotation[2]]
            ),
        }

    def forward_kinematics_mujoco(self, joint_angles):
        """Compute forward kinematics using MuJoCo's mj_kinematics"""
        q = np.concatenate([joint_angles, np.zeros(self.model.nq - len(joint_angles))])
        self.data.qpos[:] = q
        mujoco.mj_kinematics(self.model, self.data)
        return {
            "position": self.data.body(self.target_frame_name).xpos.copy(),
            "orientation": self.data.body(self.target_frame_name).xquat.copy(),
        }

    def get_ground_truth_pose(self, joint_angles):
        """Get ground truth pose from MuJoCo's xpos/xquat"""
        q = np.concatenate([joint_angles, np.zeros(self.model.nq - len(joint_angles))])
        self.data.qpos[:] = q
        mujoco.mj_step(self.model, self.data)
        return {
            "position": self.data.body(self.target_frame_name).xpos.copy(),
            "orientation": self.data.body(self.target_frame_name).xquat.copy(),
        }

    def compute_ik(self, target_pos, target_quat):
        """Custom inverse kinematics implementation"""
        return self.ik_solver.compute_ik(target_pos, target_quat)


# Parameterized tests for forward kinematics
joint_angle_cases = [
    (np.array([0.1, -0.2, 0.3, 0.0, 0.0, 0.0]), "case_1"),
    (np.array([0.78, -1.68, 1.19, 1.23, -0.32, 0.0]), "case_2"),
    (np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), "case_3"),
]


def generate_inverse_cases():
    cases = []
    for joint_angles, case_name in joint_angle_cases:
        solver = KinematicsSolver(MODEL_PATH, TARGET_FRAME_NAME)
        gt_pose = solver.get_ground_truth_pose(joint_angles)
        cases.append(
            (gt_pose["position"], gt_pose["orientation"], joint_angles, case_name)
        )
    return cases


inverse_kinematics_cases = generate_inverse_cases()


@pytest.fixture(scope="module")
def solver():
    return KinematicsSolver(MODEL_PATH, TARGET_FRAME_NAME)


@pytest.mark.parametrize("joint_angles, case_name", joint_angle_cases)
def test_forward_kinematics(solver, joint_angles, case_name):
    print(f"\nForward Kinematics Test: {case_name}")
    print(f"Input joint angles: {joint_angles}")

    # Get ground truth from MuJoCo
    gt_pose = solver.get_ground_truth_pose(joint_angles)
    print(f"Ground truth position: {gt_pose['position']}")
    print(f"Ground truth orientation: {gt_pose['orientation']}")

    # Compute with Pinocchio
    pin_pose = solver.forward_kinematics_pinocchio(joint_angles)
    print(f"pinocchio position: {pin_pose['position']}")
    print(f"pinocchio orientation: {pin_pose['orientation']}")

    pos_error = np.linalg.norm(pin_pose["position"] - gt_pose["position"])
    print(f"Pinocchio position error: {pos_error:.6f}")
    assert (
        pos_error < TOLERANCE
    ), f"Pinocchio position error exceeds tolerance: {pos_error}"

    quat_error = abs(np.dot(pin_pose["orientation"], gt_pose["orientation"]))
    print(f"Pinocchio orientation error: {1 - quat_error:.6f}")
    assert (
        quat_error > 1 - ROT_TOLERANCE
    ), f"Pinocchio orientation error exceeds tolerance: {1 - quat_error}"

    # Compute with MuJoCo
    muj_pose = solver.forward_kinematics_mujoco(joint_angles)
    print(f"mujoco position: {muj_pose['position']}")
    print(f"mujoco orientation: {muj_pose['orientation']}")

    pos_error = np.linalg.norm(muj_pose["position"] - gt_pose["position"])
    print(f"MuJoCo position error: {pos_error:.6f}")
    assert (
        pos_error < TOLERANCE
    ), f"MuJoCo position error exceeds tolerance: {pos_error}"

    quat_error = abs(np.dot(muj_pose["orientation"], gt_pose["orientation"]))
    print(f"MuJoCo orientation error: {1 - quat_error:.6f}")
    assert (
        quat_error > 1 - ROT_TOLERANCE
    ), f"MuJoCo orientation error exceeds tolerance: {1 - quat_error}"


# Parameterized tests for inverse kinematics
@pytest.mark.parametrize(
    "target_pos, target_quat, ground_truth_angles, case_name", inverse_kinematics_cases
)
def test_inverse_kinematics(
    solver, target_pos, target_quat, ground_truth_angles, case_name
):
    print(f"\nInverse Kinematics Test: {case_name}")
    print(f"Target joint position: {ground_truth_angles}")

    # Compute with custom IK
    print("target pos: ", target_pos)
    print("target quat: ", target_quat)
    custom_angles = solver.compute_ik(target_pos, target_quat)
    if custom_angles is None:
        print("[ERROR] Custom IK failed to converge")
        assert False, "Custom IK failed"
    else:
        real_pose = solver.forward_kinematics_mujoco(custom_angles)
        print(f"Custom IK joint position: {custom_angles}")
        print(f"real pos: ", real_pose["position"])
        print(f"real quat: ", real_pose["orientation"])

        pos_error = np.linalg.norm(real_pose["position"] - target_pos)
        print(f"position error: {pos_error:.6f}")
        assert pos_error < TOLERANCE, f"position error exceeds tolerance: {pos_error}"

        quat_error = abs(np.dot(real_pose["orientation"], target_quat))
        print(f"orientation error: {1 - quat_error:.6f}")
        assert (
            quat_error > 1 - ROT_TOLERANCE
        ), f"orientation error exceeds tolerance: {1 - quat_error}"
