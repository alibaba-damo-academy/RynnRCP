"""Bumi high-level controller for RynnRCP."""

from __future__ import annotations

import ctypes
import importlib
import os
import sys
import time
from pathlib import Path
from typing import Any, Mapping

from rynnrcp.robot.base_controller import BaseRobotController

MODE_WALK = 2
MODE_NAMES = {
    0: "enabled",
    1: "ready",
    2: "walk",
    5: "dance",
    8: "swing",
    9: "shake",
    10: "cheer",
    11: "start_teach",
    12: "end_teach",
    14: "save_teach_1",
    23: "play_teach",
    26: "protect",
    27: "fall_to_stand",
    28: "stand_to_fall",
    29: "save_teach_2",
    30: "disabled",
    31: "dance1",
    32: "dance2",
    33: "tear",
}
MOTION_COMMANDS = {"WALK", "SWING", "CHEER", "SHAKE", "TEAR", "PLAYTEACH", "DEFAULT"}


class BumiHighController(BaseRobotController):
    def __init__(
        self,
        robot_id: str = "bumi_high",
        sdk_root: str | None = None,
        max_x: float = 1.0,
        max_yaw: float = 1.0,
        action_frames: int = 10,
        velocity_duration_s: float = 1.0,
        velocity_hz: float = 50.0,
    ) -> None:
        super().__init__()
        sdk_root = sdk_root or os.environ.get("BUMI_SDK_ROOT")
        if not sdk_root:
            raise ValueError("BumiHighController requires sdk_root or BUMI_SDK_ROOT")
        self.robot_id = str(robot_id)
        self.sdk_root = Path(sdk_root).expanduser().resolve()
        self.max_x = abs(float(max_x))
        self.max_yaw = abs(float(max_yaw))
        self.action_frames = max(1, int(action_frames))
        self.velocity_duration_s = max(0.1, float(velocity_duration_s))
        self.velocity_hz = max(10.0, float(velocity_hz))
        self._ctrl: Any = None
        self._cmd: Any = None

    def start(self) -> None:
        if self._ctrl is not None:
            return
        self._load_sdk()
        module = importlib.import_module("highcontrol_py")
        self._cmd = module.ControlCmd
        self._ctrl = module.HighController.instance()
        self._ctrl.init()

    def shutdown(self) -> None:
        if self._ctrl is not None and self._cmd is not None:
            self._publish(0.0, 0.0, self._cmd.DEFAULT, frames=50)
        self._ctrl = None

    def get_health(self) -> dict[str, Any]:
        warnings = []
        if self._ctrl is None:
            warnings.append(_warning("bumi_high.not_started", "Bumi high-level controller is not started"))
        return {"errors": [], "warnings": warnings}

    def get_joint_positions(self) -> dict[str, Any]:
        self._ensure_started()
        joints = self._ctrl.get_joint_state()
        return {
            "joint_positions": [float(j.pos) for j in joints],
            "joint_velocities": [float(j.vel) for j in joints],
        }

    def get_imu(self) -> dict[str, Any]:
        self._ensure_started()
        imu = self._ctrl.get_imu_data()
        return {
            "accel": [float(x) for x in imu.linear_acc],
            "gyro": [float(x) for x in imu.angular_vel],
            "orientation_quat_wxyz": [float(x) for x in imu.ori],
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

    def get_mode(self) -> dict[str, Any]:
        mode = self.mode()
        return {"mode": mode, "name": MODE_NAMES.get(mode, "unknown")}

    def start_mode(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "start")
        return self._run_action("START")

    def switch_mode(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "switch")
        return self._run_action("SWITCH")

    def walk_mode(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "walk")
        return self._run_action("WALK")

    def enter_walk(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "enter_walk")
        steps: list[dict[str, Any]] = []
        if self.mode() == 30:
            steps.append(self._run_action("START"))
            time.sleep(0.5)
        steps.append(self._run_action("SWITCH"))
        time.sleep(0.5)
        steps.append(self._run_action("WALK"))
        return {"command": "ENTER_WALK", "mode": self.mode(), "steps": steps}

    def swing(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "swing")
        return self._run_action("SWING")

    def shake(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "shake")
        return self._run_action("SHAKE")

    def cheer(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "cheer")
        return self._run_action("CHEER")

    def dance(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "dance")
        return self._run_action("DANCE")

    def dance1(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "dance1")
        return self._run_action("DANCE1")

    def dance2(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "dance2")
        return self._run_action("DANCE2")

    def tear(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "tear")
        return self._run_action("TEAR")

    def fall_to_stand(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "fall_to_stand")
        return self._run_action("FALLTOSTAND")

    def stand_to_fall(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "stand_to_fall")
        return self._run_action("STANDTOFALL")

    def start_teach(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "start_teach")
        return self._run_action("STARTTEACH")

    def save_teach(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        index = _index(value)
        return self._run_action("SAVETEACH", index=index)

    def play_teach(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        index = _index(value)
        return self._run_action("PLAYTEACH", index=index)

    def stop(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "stop")
        self._ensure_started()
        self._publish(0.0, 0.0, self._cmd.DEFAULT, frames=50)
        return {"command": "DEFAULT", "mode": self.mode(), "ver": 0.0, "hor": 0.0}

    def set_base_velocity(self, value: Mapping[str, Any]) -> dict[str, Any]:
        self._ensure_started()
        if self.mode() != MODE_WALK:
            self._publish(0.0, 0.0, self._cmd.DEFAULT, frames=5)
            raise RuntimeError("Bumi base_velocity requires WALK mode (mode 2); run action.robot.enter_walk first")
        linear = value.get("linear_vel") or [0.0, 0.0, 0.0]
        angular = value.get("angular_vel") or [0.0, 0.0, 0.0]
        ver = _clamp(float(linear[0]), self.max_x)
        hor = _clamp(float(angular[2]), self.max_yaw)
        frames = self._publish_ramped_velocity(ver, hor)
        return {"mode": self.mode(), "ver": ver, "hor": hor, "duration_s": self.velocity_duration_s, "frames": frames}

    def high_command(self, value: Mapping[str, Any]) -> dict[str, Any]:
        self._ensure_started()
        command = str(value.get("command") or "DEFAULT").upper()
        index = int(value.get("index") or 0)
        frames = int(value.get("frames") or 1)
        ver = _clamp(float(value.get("ver", value.get("x", 0.0)) or 0.0), self.max_x)
        hor = _clamp(float(value.get("hor", value.get("yaw", 0.0)) or 0.0), self.max_yaw)
        action = self._command(command)
        if command == "DEFAULT":
            ver = 0.0
            hor = 0.0
        if (abs(ver) > 0.0 or abs(hor) > 0.0) and command not in MOTION_COMMANDS:
            self._publish(0.0, 0.0, self._cmd.DEFAULT, frames=5)
            raise RuntimeError(f"{command} does not accept ver/hor")
        self._publish(ver, hor, action, index=index, frames=frames)
        if command != "DEFAULT" and frames == 1:
            self._publish(0.0, 0.0, self._cmd.DEFAULT, frames=5)
        return {"command": command, "mode": self.mode(), "ver": ver, "hor": hor, "index": index, "frames": frames}

    def mode(self) -> int:
        self._ensure_started()
        return int(self._ctrl.get_mode())

    def _run_action(self, name: str, *, index: int = 0) -> dict[str, Any]:
        self._ensure_started()
        self._publish(0.0, 0.0, self._command(name), index=index, frames=1)
        self._publish(0.0, 0.0, self._cmd.DEFAULT, frames=5)
        return {"command": name, "mode": self.mode(), "ver": 0.0, "hor": 0.0, "index": index, "frames": 1}

    def _command(self, name: str) -> Any:
        try:
            return getattr(self._cmd, name)
        except AttributeError as exc:
            raise ValueError(f"unsupported Bumi high command: {name}") from exc

    def _publish(self, ver: float, hor: float, action: Any, *, frames: int, index: int = 0) -> None:
        for _ in range(max(1, int(frames))):
            self._ctrl.publish_cmd(float(ver), float(hor), action, int(index))
            time.sleep(0.002)

    def _publish_ramped_velocity(self, ver: float, hor: float) -> int:
        frames = max(1, int(self.velocity_duration_s * self.velocity_hz))
        interval = 1.0 / self.velocity_hz
        for i in range(frames):
            scale = 1.0 - (i / frames)
            self._ctrl.publish_cmd(float(ver) * scale, float(hor) * scale, self._cmd.WALK, 0)
            time.sleep(interval)
        self._publish(0.0, 0.0, self._cmd.DEFAULT, frames=5)
        return frames

    def _ensure_started(self) -> None:
        if self._ctrl is None:
            raise RuntimeError("Bumi high-level controller is not started")

    def _load_sdk(self) -> None:
        if not self.sdk_root.exists():
            raise FileNotFoundError(f"Bumi SDK root does not exist: {self.sdk_root}")
        os.environ["CYCLONEDDS_URI"] = "file://" + str(self.sdk_root / "config" / "dds.xml")
        for arch in ("aarch64", "x86_64"):
            lib_dir = self.sdk_root / "lib" / arch
            if not lib_dir.exists():
                continue
            for lib in ("libcrypto.so.1.1", "libssl.so.1.1"):
                path = lib_dir / lib
                if path.exists():
                    ctypes.CDLL(str(path), mode=ctypes.RTLD_GLOBAL)
        build = str(self.sdk_root / "build")
        if build not in sys.path:
            sys.path.insert(0, build)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _require_empty(value: Mapping[str, Any] | None, name: str) -> None:
    if value:
        raise ValueError(f"{name} action value must be empty")


def _index(value: Mapping[str, Any] | None) -> int:
    if not value:
        return 0
    return int(value.get("index") or 0)


def _warning(code: str, message: str) -> dict[str, Any]:
    return {"code": code, "message": message, "source": "robot", "timestamp": time.time()}
