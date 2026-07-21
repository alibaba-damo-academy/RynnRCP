from pathlib import Path

import numpy as np


JOINT_NUM = 23
HOME_POSITIONS = np.array([
    0.0, 0.0, -0.1, 0.3, -0.2, 0.0,
    0.0, 0.0, -0.1, 0.3, -0.2, 0.0, 0.0,
    0.18, 0.06, 0.0, 0.78, 0.0,
    0.18, -0.06, 0.0, 0.78, 0.0,
], dtype=np.float32)
USD2URDF = [0, 6, 12, 1, 7, 13, 18, 2, 8, 14, 19, 3, 9, 15, 20, 4, 10, 16, 21, 5, 11, 17, 22]


class Policy:
    def load(self, policy_dir):
        import onnxruntime as ort

        policy_dir = Path(policy_dir)
        self.session = ort.InferenceSession(str(policy_dir / "model.onnx"), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        motion = np.load(policy_dir / "motion.npz")
        self.motion_pos = motion["joint_pos"].astype(np.float32)
        self.motion_vel = motion["joint_vel"].astype(np.float32)
        self.last_action = np.zeros(JOINT_NUM, dtype=np.float32)
        self.frame = 0

    def reset(self, runtime_inputs):
        self.last_action.fill(0.0)
        self.frame = 0

    def step(self, obs):
        joint_state = obs["observation.robot.joint_state"]
        imu = obs.get("observation.robot.imu") or {}
        joint_q = _vector(joint_state.get("joint_positions"), JOINT_NUM, HOME_POSITIONS)
        joint_vel = _vector(joint_state.get("joint_velocities"), JOINT_NUM, np.zeros(JOINT_NUM, dtype=np.float32))
        motion_idx = min(self.frame, len(self.motion_pos) - 1)
        segments = [
            self.motion_pos[motion_idx],
            self.motion_vel[motion_idx],
            _vector(imu.get("gyro"), 3, np.zeros(3, dtype=np.float32)),
            _gravity_b(imu),
            _dof_pos(joint_q),
            _dof_vel(joint_vel),
            self.last_action,
        ]
        input_buffer = np.clip(np.concatenate(segments), -100.0, 100.0).astype(np.float32)
        output = self.session.run([self.output_name], {self.input_name: input_buffer.reshape(1, -1)})[0]
        self.last_action = np.clip(output.reshape(-1)[:JOINT_NUM], -100.0, 100.0).astype(np.float32)
        action = HOME_POSITIONS.copy()
        for i, idx in enumerate(USD2URDF):
            action[idx] = self.last_action[i] * 0.25 + HOME_POSITIONS[idx]
        self.frame += 1
        return {
            "name": "action.robot.joint_position",
            "frame_rate": 50,
            "frames": [{"joint_positions": action.astype(float).tolist()}],
        }


def _vector(value, length, default):
    if value is None:
        return np.array(default, dtype=np.float32)
    arr = np.array(value, dtype=np.float32).reshape(-1)
    if arr.size != length:
        return np.array(default, dtype=np.float32)
    return arr


def _gravity_b(imu):
    w, x, y, z = _vector(imu.get("orientation_quat_wxyz"), 4, np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32))
    return np.array([
        2.0 * (w * y - x * z),
        -2.0 * (w * x + y * z),
        2.0 * (x * x + y * y) - 1.0,
    ], dtype=np.float32)


def _dof_pos(joint_q):
    return np.array([joint_q[idx] - HOME_POSITIONS[idx] for idx in USD2URDF], dtype=np.float32)


def _dof_vel(joint_vel):
    return np.array([joint_vel[idx] for idx in USD2URDF], dtype=np.float32)
