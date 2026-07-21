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

        self.session = ort.InferenceSession(str(Path(policy_dir) / "model.onnx"), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.frame_stack = 10
        self.obs_layout = [
            ("ang_vel", 3),
            ("gravity_b", 3),
            ("cmd_vel", 3),
            ("dof_pos", 23),
            ("dof_vel", 23),
            ("last_action", 23),
            ("interrupt", 1),
        ]
        self.input_buffer = np.zeros(sum(size for _, size in self.obs_layout) * self.frame_stack, dtype=np.float32)
        self.last_action = np.zeros(JOINT_NUM, dtype=np.float32)
        self.first = True

    def reset(self, runtime_inputs):
        self.input_buffer.fill(0.0)
        self.last_action.fill(0.0)
        self.first = True

    def step(self, obs):
        joint_state = obs["observation.robot.joint_state"]
        imu = obs.get("observation.robot.imu") or {}
        joint_q = _vector(joint_state.get("joint_positions"), JOINT_NUM, HOME_POSITIONS)
        joint_vel = _vector(joint_state.get("joint_velocities"), JOINT_NUM, np.zeros(JOINT_NUM, dtype=np.float32))
        raw_cmd_vel = _vector(obs.get("cmd_vel"), 3, np.zeros(3, dtype=np.float32))
        cmd_vel = np.array([
            np.clip(raw_cmd_vel[0], -0.4, 0.6),
            np.clip(raw_cmd_vel[1], -0.4, 0.4),
            np.clip(raw_cmd_vel[2], -0.8, 0.8),
        ], dtype=np.float32)
        segments = [
            _vector(imu.get("gyro"), 3, np.zeros(3, dtype=np.float32)),
            _gravity_b(imu),
            cmd_vel,
            _dof_pos(joint_q),
            _dof_vel(joint_vel),
            self.last_action,
            np.array([0.0], dtype=np.float32),
        ]
        self._push_frame_major(segments)
        output = self.session.run([self.output_name], {self.input_name: self.input_buffer.reshape(1, -1)})[0]
        self.last_action = np.clip(output.reshape(-1)[:JOINT_NUM], -100.0, 100.0).astype(np.float32)
        action = HOME_POSITIONS.copy()
        for i, idx in enumerate(USD2URDF):
            action[idx] = self.last_action[i] * 0.25 + HOME_POSITIONS[idx]
        return {
            "name": "action.robot.joint_position",
            "frame_rate": 50,
            "frames": [{"joint_positions": action.astype(float).tolist()}],
        }

    def _push_frame_major(self, segments):
        obs = np.clip(np.concatenate(segments), -100.0, 100.0).astype(np.float32)
        obs_size = len(obs)
        if self.first:
            for frame in range(self.frame_stack):
                self.input_buffer[frame * obs_size:(frame + 1) * obs_size] = obs
            self.first = False
            return
        self.input_buffer[:-obs_size] = self.input_buffer[obs_size:]
        self.input_buffer[-obs_size:] = obs


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
