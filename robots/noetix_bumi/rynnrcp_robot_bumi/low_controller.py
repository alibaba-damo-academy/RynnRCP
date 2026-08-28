"""Bumi low-level controller for RynnRCP."""

from __future__ import annotations

import ctypes
import importlib
import logging
import os
import sys
import time
from pathlib import Path
from typing import Any, Mapping

import yaml

from rynnrcp.robot.base_controller import BaseRobotController

logger = logging.getLogger(__name__)

JOINT_NAMES = [
    "leg_l1_joint", "leg_r1_joint", "waist_1_joint",
    "leg_l2_joint", "leg_r2_joint", "arm_l1_joint", "arm_r1_joint",
    "leg_l3_joint", "leg_r3_joint", "arm_l2_joint", "arm_r2_joint",
    "leg_l4_joint", "leg_r4_joint", "arm_l3_joint", "arm_r3_joint",
    "leg_l5_joint", "leg_r5_joint", "arm_l4_joint", "arm_r4_joint",
    "leg_l6_joint", "leg_r6_joint",
]


class BumiLowController(BaseRobotController):
    def __init__(
        self,
        robot_id: str = "bumi_low",
        sdk_root: str | None = None,
        default_kp: float = 10.0,
        default_kd: float = 0.5,
        damping_kd: float = 0.1,
        max_delta: float = 1.0,
    ) -> None:
        super().__init__()
        sdk_root = sdk_root or os.environ.get("BUMI_SDK_ROOT")
        if not sdk_root:
            raise ValueError("BumiLowController requires sdk_root or BUMI_SDK_ROOT")
        self.robot_id = str(robot_id)
        self.sdk_root = Path(sdk_root).expanduser().resolve()
        self.default_kp = float(default_kp)
        self.default_kd = float(default_kd)
        self.damping_kd = float(damping_kd)
        self.max_delta = abs(float(max_delta))
        self._kp = [self.default_kp] * len(JOINT_NAMES)
        self._kd = [self.default_kd] * len(JOINT_NAMES)
        self._ctrl: Any = None
        self._cmd_cls: Any = None
        self._last_command_log_at = 0.0
        self._command_count = 0

    def start(self) -> None:
        if self._ctrl is not None:
            return
        logger.info("[BumiLow][START] sdk_root=%s", self.sdk_root)
        _load_sdk(self.sdk_root)
        module = importlib.import_module("lowcontrol_py")
        self._cmd_cls = module.MotorCmd
        self._ctrl = module.LowController.instance()
        ok = self._ctrl.init()
        if ok is False:
            raise RuntimeError("Bumi low-level controller init failed")
        self._load_default_gains()
        logger.info(
            "[BumiLow][READY] kp0=%.3f kd0=%.3f max_delta=%.3f",
            self._kp[0],
            self._kd[0],
            self.max_delta,
        )

    def shutdown(self) -> None:
        if self._ctrl is not None:
            logger.info("[BumiLow][SHUTDOWN] applying damping before disconnect")
            self.damping()
        self._ctrl = None

    def get_health(self) -> dict[str, Any]:
        warnings = []
        if self._ctrl is None:
            warnings.append({"code": "bumi_low.not_started", "message": "Bumi low-level controller is not started", "source": "robot", "timestamp": time.time()})
        return {"errors": [], "warnings": warnings}

    def get_joint_positions(self) -> dict[str, Any]:
        self._ensure_started()
        states = self._ctrl.get_joint_state()
        ordered = [states[self._joint_index(name)] for name in JOINT_NAMES]
        return {
            "joint_positions": [float(j.pos) for j in ordered],
            "joint_velocities": [float(j.vel) for j in ordered],
        }

    def get_imu(self) -> dict[str, Any]:
        self._ensure_started()
        imu = self._ctrl.get_imu_data()
        q_xyzw = [float(x) for x in imu.ori]
        return {
            "accel": [float(x) for x in imu.linear_acc],
            "gyro": [float(x) for x in imu.angular_vel],
            "orientation_quat_wxyz": [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]],
        }

    def get_battery(self) -> dict[str, Any]:
        self._ensure_started()
        bms = self._ctrl.get_robot_bms_data()
        return {
            "battery_soc": int(bms.battery_soc),
            "battery_temp": float(bms.battery_temp),
            "battery_soh": int(bms.battery_soh),
            "battery_alarm": int(bms.battery_alarm),
        }

    def set_joint_positions(self, value: Mapping[str, Any]) -> dict[str, Any]:
        self._ensure_started()
        target = _vector(value.get("joint_positions"), len(JOINT_NAMES), "joint_positions")
        current = self.get_joint_positions()["joint_positions"]
        max_delta = max(abs(a - b) for a, b in zip(target, current))
        if self.max_delta > 0 and max_delta > self.max_delta:
            logger.warning(
                "[BumiLow][COMMAND_BLOCKED] max_delta=%.3f limit=%.3f; "
                "send a target closer to the current joint positions",
                max_delta,
                self.max_delta,
            )
            raise RuntimeError(f"joint target delta {max_delta:.3f} exceeds max_delta {self.max_delta:.3f}")

        cmds = self._empty_cmds()
        for i, name in enumerate(JOINT_NAMES):
            hw = self._joint_index(name)
            cmds[hw].pos = float(target[i])
            cmds[hw].vel = 0.0
            cmds[hw].kp = float(self._kp[i])
            cmds[hw].kd = float(self._kd[i])
            cmds[hw].tau = 0.0
            cmds[hw].motor_id = int(hw)
        self._ctrl.set_joint(cmds)
        self._log_joint_command(current, target, max_delta)
        return {"joints": len(target), "max_delta": max_delta}

    def _log_joint_command(self, current: list[float], target: list[float], max_delta: float) -> None:
        self._command_count += 1
        now = time.time()
        if self._command_count > 3 and now - self._last_command_log_at < 1.0:
            return
        self._last_command_log_at = now
        logger.debug(
            "[BumiLow][JOINT_COMMAND] count=%d max_delta=%.3f "
            "current_abs_max=%.3f target_abs_max=%.3f",
            self._command_count,
            max_delta,
            max(abs(x) for x in current),
            max(abs(x) for x in target),
        )

    def damping(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        self._ensure_started()
        logger.info("[BumiLow][DAMPING] kd=%.3f", self.damping_kd)
        cmds = self._empty_cmds()
        for cmd in cmds:
            cmd.kp = 0.0
            cmd.kd = self.damping_kd
        self._ctrl.set_joint(cmds)
        return {"mode": "damping", "kd": self.damping_kd}

    def _empty_cmds(self) -> list[Any]:
        cmds = [self._cmd_cls() for _ in range(21)]
        for i, cmd in enumerate(cmds):
            cmd.pos = 0.0
            cmd.vel = 0.0
            cmd.tau = 0.0
            cmd.kp = 0.0
            cmd.kd = 0.0
            cmd.motor_id = i
        return cmds

    def _joint_index(self, name: str) -> int:
        idx = int(self._ctrl.getJointsIndex(name))
        if idx < 0 or idx >= 21:
            raise RuntimeError(f"Bumi joint not found: {name}")
        return idx

    def _ensure_started(self) -> None:
        if self._ctrl is None:
            raise RuntimeError("Bumi low-level controller is not started")

    def _load_default_gains(self) -> None:
        path = self.sdk_root / "config" / "bumi_ac.yaml"
        if not path.exists():
            return
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        run = data.get("run") or {}
        kp = run.get("joint_stiffness")
        kd = run.get("joint_damping")
        if isinstance(kp, list) and len(kp) == len(JOINT_NAMES):
            self._kp = [float(x) for x in kp]
        if isinstance(kd, list) and len(kd) == len(JOINT_NAMES):
            self._kd = [float(x) for x in kd]


def _load_sdk(sdk_root: Path) -> None:
    if not sdk_root.exists():
        raise FileNotFoundError(f"Bumi SDK root does not exist: {sdk_root}")
    os.environ["CYCLONEDDS_URI"] = "file://" + str(sdk_root / "config" / "dds.xml")
    for arch in ("aarch64", "x86_64"):
        lib_dir = sdk_root / "lib" / arch
        if not lib_dir.exists():
            continue
        for lib in ("libcrypto.so.1.1", "libssl.so.1.1"):
            path = lib_dir / lib
            if path.exists():
                try:
                    ctypes.CDLL(str(path), mode=ctypes.RTLD_GLOBAL)
                except OSError as e:
                    raise RuntimeError(f"Failed to load library {lib}: {e}") from e
    build = str(sdk_root / "build")
    if build not in sys.path:
        sys.path.insert(0, build)


def _vector(value: Any, length: int, name: str, *, default: float | None = None) -> list[float]:
    if value is None and default is not None:
        return [float(default)] * length
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must be a list of {length} floats")
    return [float(x) for x in value]
