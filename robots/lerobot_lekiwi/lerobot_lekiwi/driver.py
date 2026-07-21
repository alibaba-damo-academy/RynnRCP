"""Nine-motor LeKiwi hardware driver."""

from __future__ import annotations

import json
import logging
import os
from dataclasses import asdict
from pathlib import Path
from typing import Any

from .feetech_bus import FeetechBus, MotorCalibration, RESOLUTION
from .kinematics import WHEEL_NAMES, body_to_wheel_raw, wheel_raw_to_body


ARM_MOTORS = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
)
MOTORS = {name: index for index, name in enumerate(ARM_MOTORS + WHEEL_NAMES, start=1)}
LEADER_MOTORS = {name: index for index, name in enumerate(ARM_MOTORS, start=1)}


class LeKiwi:
    """Six-axis arm and three-omniwheel base on one Feetech bus."""

    def __init__(
        self,
        *,
        port: str,
        robot_id: str,
        wheel_radius_m: float = 0.05,
        base_radius_m: float = 0.125,
        max_wheel_raw: int = 3000,
        calibration_dir: str | Path | None = None,
        logger: logging.Logger | None = None,
    ) -> None:
        self.port = str(port)
        self.robot_id = str(robot_id)
        self.wheel_radius_m = float(wheel_radius_m)
        self.base_radius_m = float(base_radius_m)
        self.max_wheel_raw = int(max_wheel_raw)
        self.logger = logger or logging.getLogger(type(self).__name__)
        self.calibration_dir = Path(calibration_dir or _default_calibration_dir()).expanduser()
        self.calibration_path = self.calibration_dir / f"{self.robot_id}.json"
        self.calibration = self._load_calibration()
        self.bus = FeetechBus(self.port, MOTORS, self.calibration)

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, *, require_calibration: bool = True) -> None:
        self.bus.connect()
        try:
            missing = [name for name in ARM_MOTORS if name not in self.calibration]
            if require_calibration and missing:
                raise RuntimeError(
                    "LeKiwi arm calibration is missing for "
                    f"{', '.join(missing)}. Run rynnrcp-lekiwi-configure-web first."
                )
            self.bus.configure(ARM_MOTORS, WHEEL_NAMES)
        except Exception:
            self.bus.disconnect()
            raise
        self.logger.info("LeKiwi connected on %s with motor IDs 1-9", self.port)

    def disconnect(self) -> None:
        if not self.is_connected:
            return
        try:
            self.stop_base()
        finally:
            self.bus.disconnect(disable_torque=True)
        self.logger.info("LeKiwi disconnected from %s", self.port)

    def get_observation(self) -> dict[str, Any]:
        arm = self.bus.read_arm_degrees_and_percent(ARM_MOTORS)
        wheels = self.bus.sync_read("Present_Velocity", WHEEL_NAMES)
        return {
            **{f"{name}.pos": value for name, value in arm.items()},
            **wheel_raw_to_body(
                wheels,
                wheel_radius_m=self.wheel_radius_m,
                base_radius_m=self.base_radius_m,
            ),
        }

    def send_arm_action(self, action: dict[str, float]) -> dict[str, float]:
        values = {}
        for name in ARM_MOTORS:
            key = f"{name}.pos"
            if key not in action:
                raise ValueError(f"LeKiwi arm action is missing {key}")
            values[name] = float(action[key])
        self.bus.write_arm_degrees_and_percent(values)
        return {f"{name}.pos": value for name, value in values.items()}

    def send_base_velocity(self, x_m_s: float, y_m_s: float, yaw_rad_s: float) -> dict[str, Any]:
        wheel_raw = body_to_wheel_raw(
            x_m_s,
            y_m_s,
            yaw_rad_s,
            wheel_radius_m=self.wheel_radius_m,
            base_radius_m=self.base_radius_m,
            max_raw=self.max_wheel_raw,
        )
        self.bus.sync_write("Goal_Velocity", wheel_raw)
        return {
            "linear_vel": [float(x_m_s), float(y_m_s), 0.0],
            "angular_vel": [0.0, 0.0, float(yaw_rad_s)],
            "wheel_raw": wheel_raw,
        }

    def stop_base(self) -> None:
        self.bus.sync_write("Goal_Velocity", dict.fromkeys(WHEEL_NAMES, 0), retries=5)

    def calibrate(self) -> Path:
        """Interactively calibrate the six arm motors; wheels stay in raw velocity units."""
        if not self.is_connected:
            self.bus.connect()
        self.bus.disable_torque()
        for motor in ARM_MOTORS:
            self.bus.write("Operating_Mode", motor, 0)
        for motor in WHEEL_NAMES:
            self.bus.write("Operating_Mode", motor, 1)

        input("Place the LeKiwi arm near the middle of every joint range, then press ENTER...")
        homing_offsets = self.bus.set_half_turn_homings(ARM_MOTORS)
        homing_offsets.update(dict.fromkeys(WHEEL_NAMES, 0))

        full_turn = {"arm_wrist_flex", "arm_wrist_roll", *WHEEL_NAMES}
        measured = [name for name in MOTORS if name not in full_turn]
        range_mins, range_maxes = self.bus.record_ranges(measured)
        for name in full_turn:
            range_mins[name] = 0
            range_maxes[name] = RESOLUTION - 1

        self.calibration = {
            name: MotorCalibration(
                id=motor_id,
                drive_mode=0,
                homing_offset=homing_offsets[name],
                range_min=range_mins[name],
                range_max=range_maxes[name],
            )
            for name, motor_id in MOTORS.items()
        }
        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        self.logger.info("LeKiwi calibration saved to %s", self.calibration_path)
        return self.calibration_path

    def _load_calibration(self) -> dict[str, MotorCalibration]:
        if not self.calibration_path.is_file():
            return {}
        with self.calibration_path.open("r", encoding="utf-8") as handle:
            raw = json.load(handle)
        if not isinstance(raw, dict):
            raise ValueError(f"LeKiwi calibration must be an object: {self.calibration_path}")
        return {name: MotorCalibration(**item) for name, item in raw.items()}

    def _save_calibration(self) -> None:
        self.calibration_dir.mkdir(parents=True, exist_ok=True)
        with self.calibration_path.open("w", encoding="utf-8") as handle:
            json.dump({name: asdict(item) for name, item in self.calibration.items()}, handle, indent=2)


class LeKiwiLeader:
    """Read-only six-motor leader arm used by the LeKiwi teleoperator."""

    def __init__(
        self,
        *,
        port: str,
        robot_id: str,
        calibration_dir: str | Path | None = None,
        logger: logging.Logger | None = None,
    ) -> None:
        self.port = str(port)
        self.robot_id = str(robot_id)
        self.logger = logger or logging.getLogger(type(self).__name__)
        self.calibration_dir = Path(calibration_dir or _default_leader_calibration_dir()).expanduser()
        self.calibration_path = self.calibration_dir / f"{self.robot_id}.json"
        self.calibration = self._load_calibration()
        self.bus = FeetechBus(self.port, LEADER_MOTORS, self.calibration)

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, *, require_calibration: bool = True) -> None:
        self.bus.connect()
        try:
            missing = [name for name in ARM_MOTORS if name not in self.calibration]
            if require_calibration and missing:
                raise RuntimeError(
                    "LeKiwi leader calibration is missing for "
                    f"{', '.join(missing)}. Run rynnrcp-lekiwi-configure-web first."
                )
            self.bus.disable_torque(retries=5)
        except Exception:
            self.bus.disconnect()
            raise
        self.logger.info("LeKiwi leader connected read-only on %s with motor IDs 1-6", self.port)

    def disconnect(self) -> None:
        self.bus.disconnect(disable_torque=True)

    def get_action(self) -> dict[str, float]:
        arm = self.bus.read_arm_degrees_and_percent(ARM_MOTORS)
        return {f"{name}.pos": value for name, value in arm.items()}

    def calibrate(self) -> Path:
        if not self.is_connected:
            self.bus.connect()
        self.bus.disable_torque()
        for motor in ARM_MOTORS:
            self.bus.write("Operating_Mode", motor, 0)

        input("Place the leader arm near the middle of every joint range, then press ENTER...")
        homing_offsets = self.bus.set_half_turn_homings(ARM_MOTORS)
        full_turn = {"arm_wrist_flex", "arm_wrist_roll"}
        measured = [name for name in ARM_MOTORS if name not in full_turn]
        range_mins, range_maxes = self.bus.record_ranges(measured)
        for name in full_turn:
            range_mins[name] = 0
            range_maxes[name] = RESOLUTION - 1

        self.calibration = {
            name: MotorCalibration(
                id=motor_id,
                drive_mode=0,
                homing_offset=homing_offsets[name],
                range_min=range_mins[name],
                range_max=range_maxes[name],
            )
            for name, motor_id in LEADER_MOTORS.items()
        }
        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        return self.calibration_path

    def _load_calibration(self) -> dict[str, MotorCalibration]:
        if not self.calibration_path.is_file():
            return {}
        with self.calibration_path.open("r", encoding="utf-8") as handle:
            raw = json.load(handle)
        if not isinstance(raw, dict):
            raise ValueError(f"LeKiwi leader calibration must be an object: {self.calibration_path}")
        return {name: MotorCalibration(**item) for name, item in raw.items()}

    def _save_calibration(self) -> None:
        self.calibration_dir.mkdir(parents=True, exist_ok=True)
        with self.calibration_path.open("w", encoding="utf-8") as handle:
            json.dump({name: asdict(item) for name, item in self.calibration.items()}, handle, indent=2)


def _default_calibration_dir() -> Path:
    hf_home = Path(os.getenv("HF_HOME", Path.home() / ".cache" / "huggingface"))
    root = Path(os.getenv("HF_LEROBOT_CALIBRATION", hf_home / "lerobot" / "calibration"))
    return root / "robots" / "lekiwi"


def _default_leader_calibration_dir() -> Path:
    hf_home = Path(os.getenv("HF_HOME", Path.home() / ".cache" / "huggingface"))
    root = Path(os.getenv("HF_LEROBOT_CALIBRATION", hf_home / "lerobot" / "calibration"))
    return root / "teleoperators" / "lekiwi_leader"
