"""Atom01 body adapter for RynnRCP."""

from __future__ import annotations

import importlib.util
import logging
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

import yaml

from rynnrcp.robot.base_controller import BaseRobotController


JOINT_NAMES = (
    "left_thigh_yaw_joint",
    "left_thigh_roll_joint",
    "left_thigh_pitch_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_thigh_yaw_joint",
    "right_thigh_roll_joint",
    "right_thigh_pitch_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "torso_joint",
    "left_arm_pitch_joint",
    "left_arm_roll_joint",
    "left_arm_yaw_joint",
    "left_elbow_pitch_joint",
    "left_elbow_yaw_joint",
    "right_arm_pitch_joint",
    "right_arm_roll_joint",
    "right_arm_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
)
HOME_POSITIONS = (
    0.0,
    0.0,
    -0.1,
    0.3,
    -0.2,
    0.0,
    0.0,
    0.0,
    -0.1,
    0.3,
    -0.2,
    0.0,
    0.0,
    0.18,
    0.06,
    0.0,
    0.78,
    0.0,
    0.18,
    -0.06,
    0.0,
    0.78,
    0.0,
)

class Atom01Controller(BaseRobotController):
    """RCP controller wrapper around atom_control.python.controller.RobotController."""

    def __init__(
        self,
        robot_id: str = "atom01",
        role: str = "standalone",
        atom_root: str | None = None,
        config_path: str | None = None,
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__(logger=logger)
        self.robot_id = str(robot_id)
        self.role = str(role)
        self.atom_root = Path(atom_root).expanduser().resolve() if atom_root else _default_atom_root()
        self.config_path = Path(config_path).expanduser().resolve() if config_path else self.atom_root / "config" / "robot.yaml"
        self._ctrl: Any = None
        self._last_error: str | None = None

    def start(self) -> None:
        if self._ctrl is not None:
            return
        controller_cls = _load_atom_controller_class(self.atom_root)
        try:
            self._ctrl = controller_cls(str(self.config_path), list(HOME_POSITIONS))
        except ModuleNotFoundError as exc:
            if exc.name == "atom01_py":
                self._last_error = (
                    "atom01_py is not importable. Build atom_control on the robot and make sure "
                    f"{self.atom_root / 'build'} is usable by this Python."
                )
                raise ImportError(self._last_error) from exc
            raise
        self._last_error = None

    def shutdown(self) -> None:
        ctrl = self._ctrl
        self._ctrl = None
        if ctrl is not None:
            ctrl.shutdown()

    def _is_connected(self) -> bool:
        return self._ctrl is not None

    def get_joint_positions(self) -> Dict[str, List[float]]:
        state = self.get_status()
        result = {"joint_positions": _float_list(state.get("joint_q"), len(JOINT_NAMES), "joint_q")}
        try:
            result["joint_velocities"] = _float_list(state.get("joint_vel"), len(JOINT_NAMES), "joint_vel")
        except Exception:
            pass
        return result

    def get_imu(self) -> Dict[str, List[float]]:
        state = self.get_status()
        return {
            "accel": _float_list(state.get("imu_accel", [0.0, 0.0, 0.0]), 3, "imu_accel"),
            "gyro": _float_list(state.get("imu_ang_vel", [0.0, 0.0, 0.0]), 3, "imu_ang_vel"),
            "orientation_quat_wxyz": _float_list(state.get("imu_quat", [1.0, 0.0, 0.0, 0.0]), 4, "imu_quat"),
        }

    def set_joint_positions(self, value: Dict[str, Any]) -> Dict[str, Any]:
        positions = _joint_positions(value)
        return self._call_atom("set_targets", positions)

    def move_to_home(self, value: Dict[str, Any] | None = None) -> Dict[str, Any]:
        value = value or {}
        duration_s = float(value.get("duration_s", 1.0)) if isinstance(value, dict) else 1.0
        rate_hz = float(value.get("rate_hz", 50.0)) if isinstance(value, dict) else 50.0
        return self._call_atom("reset_to_default", [0.0] * len(JOINT_NAMES), duration_s, rate_hz)

    def toggle_damping(self, value: Dict[str, Any] | None = None) -> Dict[str, Any]:
        state = self.get_status()
        if bool(state.get("damping")):
            return self._call_atom("clear_damping")
        return self._call_atom("set_damping", 2.0)

    def get_health(self) -> Dict[str, Any]:
        warnings: list[Dict[str, Any]] = []
        if not self._is_connected():
            warnings.append(
                {
                    "code": "atom01.not_connected",
                    "message": "Atom01 controller is not connected",
                    "source": "robot",
                    "timestamp": time.time(),
                    "details": {"robot_id": self.robot_id, "atom_root": str(self.atom_root)},
                }
            )
        state = self.get_status()
        if self._last_error:
            warnings.append(
                {
                    "code": "atom01.last_error",
                    "message": self._last_error,
                    "source": "robot",
                    "timestamp": time.time(),
                    "details": {"robot_id": self.robot_id},
                }
            )
        if self._is_connected() and not state.get("motors_init"):
            warnings.append(
                {
                    "code": "atom01.motors_not_enabled",
                    "message": "Atom01 motors are not enabled",
                    "source": "robot",
                    "timestamp": time.time(),
                    "details": {"robot_id": self.robot_id},
                }
            )
        calibration = _calibration_status(self.config_path)
        if not calibration.get("confirmed"):
            warnings.append(
                {
                    "code": "atom01.zero_calibration_unconfirmed",
                    "message": "Atom01 zero calibration is not confirmed. Verify the URDF zero pose before moving the robot.",
                    "source": "robot",
                    "timestamp": time.time(),
                    "details": {
                        "robot_id": self.robot_id,
                        "config_path": str(self.config_path),
                        "reference_image": calibration.get("reference_image"),
                    },
                }
            )
        return {"errors": [], "warnings": warnings}

    def get_status(self) -> Dict[str, Any]:
        if self._ctrl is None:
            return {
                "connected": False,
                "state": "offline",
                "motors_init": False,
                "damping": False,
                "joint_q": list(HOME_POSITIONS),
                "joint_vel": [0.0] * len(JOINT_NAMES),
                "targets": list(HOME_POSITIONS),
                "last_error": self._last_error,
            }
        state = dict(self._ctrl.get_state())
        state["connected"] = True
        state["last_error"] = self._last_error
        return state

    def _ensure(self) -> Any:
        if self._ctrl is None:
            self.start()
        return self._ctrl

    def _call_atom(self, method_name: str, *args: Any) -> Dict[str, Any]:
        ctrl = self._ensure()
        ok, msg = getattr(ctrl, method_name)(*args)
        if not ok:
            self._last_error = str(msg)
            raise RuntimeError(str(msg))
        self._last_error = None
        return {"success": True, "message": str(msg)}


def _default_atom_root() -> Path:
    return Path(__file__).resolve().parent / "atom_control"


def _load_atom_controller_class(atom_root: Path) -> Any:
    atom_root = atom_root.resolve()
    for item in (atom_root / "python", atom_root / "build"):
        text = str(item)
        if text not in sys.path:
            sys.path.insert(0, text)
    controller_path = atom_root / "python" / "controller.py"
    spec = importlib.util.spec_from_file_location("_atom_control_controller", controller_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load atom_control controller: {controller_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.RobotController


def _calibration_status(config_path: Path) -> Dict[str, Any]:
    try:
        data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    except Exception:
        return {"confirmed": False}
    calibration = data.get("calibration") if isinstance(data, dict) else {}
    if not isinstance(calibration, dict):
        return {"confirmed": False}
    standard_pose = calibration.get("standard_pose") if isinstance(calibration.get("standard_pose"), dict) else {}
    return {
        "confirmed": bool(calibration.get("confirmed")),
        "confirmed_at": calibration.get("confirmed_at"),
        "reference_image": standard_pose.get("reference_image"),
    }


def _joint_positions(value: Dict[str, Any]) -> list[float]:
    if not isinstance(value, dict) or "joint_positions" not in value:
        raise TypeError("joint_position value requires joint_positions")
    return _float_list(value["joint_positions"], len(JOINT_NAMES), "joint_positions")


def _float_list(value: Any, length: int, name: str) -> list[float]:
    if not isinstance(value, (list, tuple)):
        raise TypeError(f"{name} must be a list")
    if len(value) != length:
        raise ValueError(f"{name} must contain {length} values, got {len(value)}")
    return [float(item) for item in value]
