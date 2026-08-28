from __future__ import annotations

import math
import logging
import os
import time
from pathlib import Path
from typing import Any

import numpy as np

from rynnrcp.utils.redaction import describe_payload

logger = logging.getLogger(__name__)

JOINT_NUM = 23
JOINT_NAMES = [
    "head_yaw",
    "head_pitch",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_elbow_pitch",
    "left_elbow_yaw",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_elbow_pitch",
    "right_elbow_yaw",
    "waist",
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "left_knee_pitch",
    "left_crank_up",
    "left_crank_down",
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "right_knee_pitch",
    "right_crank_up",
    "right_crank_down",
]
DEFAULT_JOINT_POS = [
    0.0,
    0.0,
    0.20,
    -1.35,
    0.0,
    -0.50,
    0.20,
    1.35,
    0.0,
    0.50,
    0.0,
    -0.20,
    0.0,
    0.0,
    0.40,
    -0.25,
    0.0,
    -0.20,
    0.0,
    0.0,
    0.40,
    -0.25,
    0.0,
]
STIFFNESS = [
    20,
    20,
    20,
    20,
    20,
    20,
    20,
    20,
    20,
    20,
    200,
    200,
    200,
    200,
    200,
    50,
    50,
    200,
    200,
    200,
    200,
    50,
    50,
]
DAMPING = [
    0.2,
    0.2,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    0.5,
    5,
    5,
    5,
    5,
    5,
    3,
    3,
    5,
    5,
    5,
    5,
    3,
    3,
]
CONTROL_DT = 0.002
CONTROL_DECIMATION = 10
GAIT_FREQUENCY = 1.0
ACTION_SCALE = 1.0
ACTION_SIZE = 12
OBS_SIZE = 47
CLIP_ACTIONS = 1.0
GRAVITY_SCALE = 1.0
LIN_VEL_SCALE = 1.0
ANG_VEL_SCALE = 1.0
DOF_POS_SCALE = 1.0
DOF_VEL_SCALE = 0.1
SCALE_X = 1.0
SCALE_Y = 1.0
SCALE_Z = 1.0


class Policy:
    def load(self, policy_dir: str | Path) -> None:
        import onnxruntime as ort

        policy_dir = Path(policy_dir)
        model_path = Path(os.environ.get("BOOSTER_T1_POLICY_ONNX") or policy_dir / "model.onnx")
        if not model_path.exists():
            raise FileNotFoundError(f"Booster T1 policy ONNX model not found: {model_path}")

        self.joint_names = list(JOINT_NAMES)
        self.default_joint_pos = _array(DEFAULT_JOINT_POS)
        self.action_scale = ACTION_SCALE
        self.frame_rate = 1.0 / (CONTROL_DT * CONTROL_DECIMATION)
        self.action_size = ACTION_SIZE
        self.obs_size = OBS_SIZE
        self.clip_actions = CLIP_ACTIONS
        self.gravity_scale = GRAVITY_SCALE
        self.lin_vel_scale = LIN_VEL_SCALE
        self.ang_vel_scale = ANG_VEL_SCALE
        self.dof_pos_scale = DOF_POS_SCALE
        self.dof_vel_scale = DOF_VEL_SCALE
        self.scalex = SCALE_X
        self.scaley = SCALE_Y
        self.scalez = SCALE_Z

        self.session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.smoothed_cmd = np.zeros(3, dtype=np.float32)
        self.last_actions = np.zeros(self.action_size, dtype=np.float32)
        self.gait_frequency = GAIT_FREQUENCY
        logger.info(
            "[BoosterT1Policy][LOADED] model=%s frame_rate=%.1f "
            "obs_size=%d action_size=%d",
            model_path,
            self.frame_rate,
            self.obs_size,
            self.action_size,
        )

    def reset(self, runtime_inputs: dict[str, Any]) -> None:
        self.smoothed_cmd.fill(0.0)
        self.last_actions.fill(0.0)
        self.gait_frequency = GAIT_FREQUENCY
        logger.info(
            "[BoosterT1Policy][RESET] runtime_inputs=%s",
            describe_payload(runtime_inputs),
        )

    def step(self, obs: dict[str, Any]) -> dict[str, Any]:
        policy_obs = self._compute_observation(obs)
        output = self.session.run([self.output_name], {self.input_name: policy_obs.reshape(1, -1)})[0]
        actions = output.astype(np.float32).reshape(-1)
        if actions.size != self.action_size:
            raise RuntimeError(f"Booster T1 policy action size {actions.size} != expected {self.action_size}")
        self.last_actions = np.clip(actions, -self.clip_actions, self.clip_actions)
        target = self.default_joint_pos.copy()
        target[11:] += self.action_scale * self.last_actions
        return {
            "name": "action.robot.joint_position",
            "frame_rate": self.frame_rate,
            "frames": [{"joint_positions": target.astype(float).tolist()}],
        }

    def _compute_observation(self, obs: dict[str, Any]) -> np.ndarray:
        joint = obs["observation.robot.joint_state"]
        imu = obs["observation.robot.imu"]
        cmd = _cmd(obs.get("cmd_vel"), self.scalex, self.scaley, self.scalez)
        interval = 1.0 / self.frame_rate
        self.smoothed_cmd += np.clip(cmd - self.smoothed_cmd, -interval, interval)
        gait_phase = math.fmod(time.perf_counter() * self.gait_frequency, 1.0)
        moving = np.linalg.norm(self.smoothed_cmd) >= 1e-5
        self.gait_frequency = GAIT_FREQUENCY if moving else 0.0
        projected_gravity = _projected_gravity(imu)
        q = _required_vector(joint.get("joint_positions"), JOINT_NUM, "observation.robot.joint_state.joint_positions")
        dq = _required_vector(joint.get("joint_velocities"), JOINT_NUM, "observation.robot.joint_state.joint_velocities")
        frame = np.concatenate([
            projected_gravity * self.gravity_scale,
            _required_vector(imu.get("gyro"), 3, "observation.robot.imu.gyro") * self.ang_vel_scale,
            self.smoothed_cmd * np.array([self.lin_vel_scale, self.lin_vel_scale, self.ang_vel_scale], dtype=np.float32) * moving,
            np.array([math.cos(2 * math.pi * gait_phase), math.sin(2 * math.pi * gait_phase)], dtype=np.float32) * moving,
            (q[11:] - self.default_joint_pos[11:]) * self.dof_pos_scale,
            dq[11:] * self.dof_vel_scale,
            self.last_actions,
        ]).astype(np.float32)
        if frame.size != self.obs_size:
            raise RuntimeError(f"Booster T1 policy obs size {frame.size} != expected {self.obs_size}")
        return frame


def _array(value: Any) -> np.ndarray:
    arr = np.array(value, dtype=np.float32).reshape(-1)
    if arr.size != JOINT_NUM:
        raise ValueError(f"expected {JOINT_NUM} values, got {arr.size}")
    return arr


def _vector(value: Any, length: int, default: Any) -> np.ndarray:
    if value is None:
        return np.array(default, dtype=np.float32).reshape(length)
    arr = np.array(value, dtype=np.float32).reshape(-1)
    return arr if arr.size == length else np.array(default, dtype=np.float32).reshape(length)


def _required_vector(value: Any, length: int, name: str) -> np.ndarray:
    if value is None:
        raise ValueError(f"{name} is required")
    arr = np.array(value, dtype=np.float32).reshape(-1)
    if arr.size != length:
        raise ValueError(f"{name} must have {length} values, got {arr.size}")
    return arr


def _cmd(value: Any, scalex: float, scaley: float, scalez: float) -> np.ndarray:
    raw = _vector(value, 3, [0.0, 0.0, 0.0])
    return np.array([
        np.clip(raw[0] * scalex, -1.5, 1.5),
        np.clip(raw[1] * scaley, -1.0, 1.0),
        np.clip(raw[2] * scalez, -1.0, 1.0),
    ], dtype=np.float32)


def _projected_gravity(imu: dict[str, Any]) -> np.ndarray:
    if "orientation_quat_wxyz" in imu:
        w, x, y, z = _required_vector(
            imu.get("orientation_quat_wxyz"),
            4,
            "observation.robot.imu.orientation_quat_wxyz",
        )
        return np.array([
            2.0 * (w * y - x * z),
            -2.0 * (w * x + y * z),
            2.0 * (x * x + y * y) - 1.0,
        ], dtype=np.float32)
    rpy = _required_vector(imu.get("rpy"), 3, "observation.robot.imu.rpy")
    return _rotate_vector_inverse_rpy(rpy[0], rpy[1], rpy[2], np.array([0.0, 0.0, -1.0]))


def _rotate_vector_inverse_rpy(roll: float, pitch: float, yaw: float, vector: np.ndarray) -> np.ndarray:
    rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    return ((rz @ ry @ rx).T @ vector).astype(np.float32)
