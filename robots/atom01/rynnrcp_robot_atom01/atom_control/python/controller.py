#!/usr/bin/env python3
"""Thin Python wrapper for Atom01 low-rate robot control.

The C++ RobotInterface owns the high-rate motor loop. Python only exposes
low-rate robot commands.
"""

from __future__ import annotations

from pathlib import Path
import time
from typing import Any
import warnings

import yaml


class RobotController:
    def __init__(
        self,
        config_path: str,
        initial_positions: list[float] | None = None,
        *,
        require_calibration: bool = True,
    ):
        if require_calibration and not _calibration_confirmed(config_path):
            warnings.warn(
                "Atom01 zero calibration is not confirmed in robot.yaml. Confirm the robot was calibrated "
                "in the URDF zero pose before moving to policy/home positions.",
                RuntimeWarning,
                stacklevel=2,
            )
        import atom01_py
    
        self.robot = atom01_py.RobotInterface(config_path)
        self.robot.start_control_loop()
        self.joint_num = int(self.robot.get_joint_num())
        self.targets = [0.0] * self.joint_num
        self.robot.init_motors()
        # 等待控制循环（250Hz）执行至少一次 apply_damping() 以更新 joint_q_
        time.sleep(0.05)
        self.targets = self._read_joint_q(default=self.targets)
        self.robot.set_targets(self.targets)
        if initial_positions is not None:
            ok, msg = self.reset_to_default(initial_positions)
            if not ok:
                raise RuntimeError(msg)

    def init_motors(self) -> tuple[bool, str]:
        if self.robot.is_init:
            return True, "motors already enabled"
        self.robot.init_motors()
        time.sleep(0.05)  # 等待控制循环更新 joint_q_
        self.targets = self._read_joint_q(default=self.targets)
        self.robot.set_targets(self.targets)
        return True, "motors enabled in damping mode"

    def soft_deinit(self) -> tuple[bool, str]:
        self.robot.soft_deinit_motors(2.0, 3.0)
        return True, "motors disabled"

    def set_damping(self, kd: float = 2.0) -> tuple[bool, str]:
        self.robot.set_damping(float(kd))
        return True, "damping enabled"

    def clear_damping(self) -> tuple[bool, str]:
        if not self.robot.is_init:
            return False, "motors are not enabled"
        self.targets = self._read_joint_q(default=self.targets)
        self.robot.set_targets(self.targets)
        self.robot.clear_damping()
        return True, "damping cleared"

    def emergency_stop(self) -> tuple[bool, str]:
        return self.set_damping()

    def set_zeros(self) -> tuple[bool, str]:
        self.robot.set_zeros()
        return True, "zeros set"

    def set_zero_single(self, motor_index: int) -> tuple[bool, str]:
        self.robot.set_zero_single(int(motor_index))
        return True, f"zero set for motor {int(motor_index)}"

    def clear_errors(self) -> tuple[bool, str]:
        self.robot.clear_errors()
        return True, "errors cleared"

    def refresh_joints(self) -> tuple[bool, str]:
        self.robot.refresh_joints()
        self.targets = self._read_joint_q(default=self.targets)
        return True, "joints refreshed"

    def set_targets(self, targets: list[float]) -> tuple[bool, str]:
        targets = _float_list(targets, self.joint_num, "targets")
        if not self.robot.is_init:
            return False, "motors are not enabled"
        self.targets = targets
        self.robot.set_targets(targets)
        return True, "targets updated"

    def reset_to_default(
        self,
        joint_default_angle: list[float],
        duration_s: float = 1.0,
        rate_hz: float = 50.0,
    ) -> tuple[bool, str]:
        if not self.robot.is_init:
            return False, "motors are not enabled"
        target = _float_list(joint_default_angle, self.joint_num, "joint_default_angle")
        start = self._read_joint_q(default=self.targets)
        if self.robot.is_damping():
            self.robot.set_targets(start)
            self.robot.clear_damping()
        steps = max(1, int(float(duration_s) * float(rate_hz)))
        dt = 1.0 / max(1.0, float(rate_hz))
        for step in range(1, steps + 1):
            alpha = step / steps
            cmd = [a + (b - a) * alpha for a, b in zip(start, target)]
            self.robot.set_targets(cmd)
            time.sleep(dt)
        self.targets = target
        return True, "moved to default pose"

    def get_state(self) -> dict[str, Any]:
        motors_init = bool(self.robot.is_init)
        joint_q = self._read_joint_q(default=self.targets)
        joint_vel = self._read_vector(self.robot.get_joint_vel, self.joint_num)
        joint_tau = self._read_vector(self.robot.get_joint_tau, self.joint_num)
        return {
            "state": "damping" if self.robot.is_damping() else ("manual" if motors_init else "idle"),
            "motors_init": motors_init,
            "damping": bool(self.robot.is_damping()) if motors_init else False,
            "joint_q": joint_q,
            "joint_vel": joint_vel,
            "joint_tau": joint_tau,
            "imu_quat": self._read_vector(self.robot.get_quat, 4, [1.0, 0.0, 0.0, 0.0]),
            "imu_ang_vel": self._read_vector(self.robot.get_ang_vel, 3),
            "imu_accel": self._read_vector(self.robot.get_lin_acc, 3),
            "targets": list(self.targets),
        }

    def shutdown(self) -> None:
        try:
            if self.robot.is_init:
                self.robot.soft_deinit_motors(2.0, 3.0)
        finally:
            self.robot.stop_control_loop()

    def _read_joint_q(self, default: list[float]) -> list[float]:
        return self._read_vector(self.robot.get_joint_q, self.joint_num, default)

    def _read_vector(
        self,
        reader: Any,
        length: int,
        default: list[float] | None = None,
    ) -> list[float]:
        try:
            values = list(reader())
        except Exception:
            values = list(default) if default is not None else [0.0] * length
        if len(values) != length:
            values = (values + [0.0] * length)[:length]
        return [float(value) for value in values]


def _float_list(value: Any, length: int, name: str) -> list[float]:
    if not isinstance(value, (list, tuple)):
        raise TypeError(f"{name} must be a list")
    if len(value) != length:
        raise ValueError(f"{name} must contain {length} values, got {len(value)}")
    return [float(item) for item in value]


def _calibration_confirmed(config_path: str) -> bool:
    data = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}
    calibration = data.get("calibration") if isinstance(data, dict) else {}
    return bool(isinstance(calibration, dict) and calibration.get("confirmed"))
