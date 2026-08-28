from __future__ import annotations

import math
import logging
import os
from pathlib import Path
from typing import Any
import time

import numpy as np
import yaml

from rynnrcp.utils.redaction import describe_payload

logger = logging.getLogger(__name__)

JOINT_NUM = 21


class Policy:
    def load(self, policy_dir: str | Path) -> None:
        import onnxruntime as ort

        policy_dir = Path(policy_dir)
        model_path = Path(os.environ.get("BUMI_POLICY_ONNX") or policy_dir / "model.onnx")
        if not model_path.exists():
            raise FileNotFoundError(f"Bumi policy ONNX not found: {model_path}")
        cfg = _load_cfg(policy_dir)
        self.joint_names = list(cfg["joint_names"])
        self.action_scale = _array(cfg["action_scale"])
        self.default_joint_pos = _array(cfg["default_joint_pos"])
        self.decimation = int(cfg.get("control", {}).get("decimation", 10))
        self.frame_rate = 1.0 / float(cfg.get("control", {}).get("cycle_time", 0.02))
        size = cfg.get("size", {})
        self.action_size = int(size.get("actions_size", JOINT_NUM))
        self.obs_size = int(size.get("observations_size", 71))
        self.stack_size = int(size.get("stack_size", 5))
        self.clip_obs = float(cfg.get("normalization", {}).get("clip_scales", {}).get("clip_observations", 18.0))
        self.clip_actions = float(cfg.get("normalization", {}).get("clip_scales", {}).get("clip_actions", 18.0))
        axes = cfg.get("axis_mappings", {})
        self.scalex = float(axes.get("scalex", 1.8))
        self.scaley = float(axes.get("scaley", 0.5))
        self.scalez = float(axes.get("scalez", 1.0))

        opts = ort.SessionOptions()
        opts.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(model_path), sess_options=opts, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.history = np.zeros(self.obs_size * self.stack_size, dtype=np.float32)
        self.last_actions = np.zeros(self.action_size, dtype=np.float32)
        self.first = True
        self._last_warn_at = 0.0
        self._last_status_at = 0.0
        self._step_index = 0
        self._infer_index = 0
        logger.info(
            "[BumiPolicy][LOADED] model=%s inference_hz=%.1f decimation=%d "
            "obs_size=%d stack_size=%d",
            model_path,
            self.frame_rate,
            self.decimation,
            self.obs_size,
            self.stack_size,
        )

    def reset(self, runtime_inputs: dict[str, Any]) -> None:
        self.history.fill(0.0)
        self.last_actions.fill(0.0)
        self.first = True
        self._last_status_at = 0.0
        self._step_index = 0
        self._infer_index = 0
        logger.info(
            "[BumiPolicy][RESET] runtime_inputs=%s",
            describe_payload(runtime_inputs),
        )

    def step(self, obs: dict[str, Any]) -> dict[str, Any]:
        self._step_index += 1
        policy_obs = self._compute_observation(obs)
        output = self.session.run(None, {self.input_name: policy_obs.reshape(1, -1)})[0]
        self.last_actions = np.clip(output.reshape(-1)[:self.action_size], -self.clip_actions, self.clip_actions).astype(np.float32)
        self._infer_index += 1
        self._print_infer_status(obs, policy_obs, output)
        self._warn_if_static()

        current = _vector(
            obs["observation.robot.joint_state"].get("joint_positions"),
            JOINT_NUM,
            self.default_joint_pos,
        )
        target = self.last_actions[:JOINT_NUM] * self.action_scale[:JOINT_NUM] + self.default_joint_pos[:JOINT_NUM]
        self._print_status(obs, target - current)
        return {
            "name": "action.robot.joint_position",
            "frame_rate": self.frame_rate,
            "frames": [{"joint_positions": target.astype(float).tolist()}],
        }

    def _compute_observation(self, obs: dict[str, Any]) -> np.ndarray:
        joint = obs["observation.robot.joint_state"]
        imu = obs["observation.robot.imu"]
        cmd = _cmd(obs.get("cmd_vel"), self.scalex, self.scaley, self.scalez)
        wxyz = _vector(imu.get("orientation_quat_wxyz"), 4, [1.0, 0.0, 0.0, 0.0])
        quat = np.array([wxyz[1], wxyz[2], wxyz[3], wxyz[0]], dtype=np.float32)
        euler = _quat_to_xyz(quat)
        frame = np.concatenate([
            cmd,
            _vector(imu.get("gyro"), 3, [0.0, 0.0, 0.0]),
            euler[:2],
            _vector(joint.get("joint_positions"), JOINT_NUM, self.default_joint_pos) - self.default_joint_pos[:JOINT_NUM],
            _vector(joint.get("joint_velocities"), JOINT_NUM, np.zeros(JOINT_NUM, dtype=np.float32)),
            self.last_actions[:JOINT_NUM],
        ]).astype(np.float32)
        if frame.size != self.obs_size:
            raise RuntimeError(f"Bumi policy obs size {frame.size} != expected {self.obs_size}")
        if self.first:
            for i in range(self.stack_size):
                self.history[i * self.obs_size:(i + 1) * self.obs_size] = frame
            self.first = False
        else:
            self.history[:-self.obs_size] = self.history[self.obs_size:]
            self.history[-self.obs_size:] = frame
        return np.clip(self.history, -self.clip_obs, self.clip_obs)

    def _warn_if_static(self) -> None:
        now = time.time()
        if np.max(np.abs(self.last_actions)) >= 1e-4 or now - self._last_warn_at < 2.0:
            return
        self._last_warn_at = now
        logger.warning(
            "[BumiPolicy][OUTPUT_NEAR_ZERO] inspect the ONNX model, cmd_vel, "
            "and observation inputs"
        )

    def _print_status(self, obs: dict[str, Any], delta: np.ndarray) -> None:
        now = time.time()
        if now - self._last_status_at < 1.0:
            return
        self._last_status_at = now
        cmd = [round(float(x), 3) for x in _vector(obs.get("cmd_vel"), 3, [0.0, 0.0, 0.0])]
        logger.debug(
            "[BumiPolicy][LOOP] cmd_vel=%s inference_hz=%.1f "
            "max_action=%.3f target_delta=%.3f",
            cmd,
            self.frame_rate,
            float(np.max(np.abs(self.last_actions))),
            float(np.max(np.abs(delta))),
        )

    def _print_infer_status(self, obs: dict[str, Any], policy_obs: np.ndarray, output: np.ndarray) -> None:
        if self._infer_index <= 3 or self._infer_index % max(1, int(self.frame_rate)) == 0:
            cmd_raw = [round(float(x), 3) for x in _vector(obs.get("cmd_vel"), 3, [0.0, 0.0, 0.0])]
            cmd_used = [round(float(x), 3) for x in _cmd(obs.get("cmd_vel"), self.scalex, self.scaley, self.scalez)]
            joint = obs["observation.robot.joint_state"]
            q = _vector(joint.get("joint_positions"), JOINT_NUM, self.default_joint_pos)
            dq = _vector(joint.get("joint_velocities"), JOINT_NUM, np.zeros(JOINT_NUM, dtype=np.float32))
            out = np.array(output, dtype=np.float32).reshape(-1)
            logger.debug(
                "[BumiPolicy][INFER] step=%d infer=%d cmd_raw=%s cmd_used=%s "
                "obs_abs_max=%.3f q_abs_max=%.3f dq_abs_max=%.3f onnx_abs_max=%.3f",
                self._step_index,
                self._infer_index,
                cmd_raw,
                cmd_used,
                float(np.max(np.abs(policy_obs))),
                float(np.max(np.abs(q))),
                float(np.max(np.abs(dq))),
                float(np.max(np.abs(out))),
            )


def _load_cfg(policy_dir: Path) -> dict[str, Any]:
    candidates = [policy_dir / "bumi_ac.yaml"]
    sdk_root = os.environ.get("BUMI_SDK_ROOT")
    if sdk_root:
        candidates.append(Path(sdk_root) / "config" / "bumi_ac.yaml")
    for path in candidates:
        if path.exists():
            data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            return data["run"]
    raise FileNotFoundError("Bumi policy config not found: put bumi_ac.yaml beside model.onnx or set BUMI_SDK_ROOT")


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


def _cmd(value: Any, scalex: float, scaley: float, scalez: float) -> np.ndarray:
    raw = _vector(value, 3, [0.0, 0.0, 0.0])
    x = raw[0] * scalex
    y = raw[1] * scaley
    yaw = raw[2] * scalez
    if x < -0.5:
        x = -0.5
    if 0.0 <= x < 0.3:
        x = 0.0
    if abs(yaw) < 0.3:
        yaw = 0.0
    return np.array([np.clip(x, -1.5, 1.5), np.clip(y, -1.0, 1.0), np.clip(yaw, -1.0, 1.0)], dtype=np.float32)


def _quat_to_xyz(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return np.array([roll, pitch, yaw], dtype=np.float32)
