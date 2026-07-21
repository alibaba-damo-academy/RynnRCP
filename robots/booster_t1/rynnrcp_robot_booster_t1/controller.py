"""Booster T1 high-level controller for RynnRCP."""

from __future__ import annotations

import time
import threading
from typing import Any, Mapping

from rynnrcp.robot.base_controller import BaseRobotController

MODE_NAMES = {
    "kUnknown": "unknown",
    "kDamping": "damping",
    "kPrepare": "prepare",
    "kWalking": "walking",
    "kCustom": "custom",
}


class BoosterT1HighController(BaseRobotController):
    def __init__(
        self,
        robot_id: str = "booster_t1",
        net: str = "127.0.0.1",
        domain_id: int = 0,
        max_x: float = 0.5,
        max_y: float = 0.3,
        max_yaw: float = 0.5,
        velocity_duration_s: float = 0.5,
        velocity_hz: float = 20.0,
    ) -> None:
        super().__init__()
        self.robot_id = str(robot_id)
        self.net = str(net)
        self.domain_id = int(domain_id)
        self.max_x = abs(float(max_x))
        self.max_y = abs(float(max_y))
        self.max_yaw = abs(float(max_yaw))
        self.velocity_duration_s = max(0.1, float(velocity_duration_s))
        self.velocity_hz = max(5.0, float(velocity_hz))
        self._client: Any = None
        self._sdk: Any = None
        self._velocity_stop = threading.Event()

    def start(self) -> None:
        if self._client is not None:
            return
        try:
            import booster_robotics_sdk_python as sdk  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "booster_robotics_sdk_python is not installed; install Booster Robotics SDK Python binding on the robot"
            ) from exc
        self._sdk = sdk
        sdk.ChannelFactory.Instance().Init(self.domain_id, self.net)
        self._client = sdk.B1LocoClient()
        self._client.Init()

    def shutdown(self) -> None:
        self._velocity_stop.set()
        if self._client is not None:
            try:
                self._stop_if_walking()
                self._client.ChangeMode(self._sdk.RobotMode.kPrepare)
                time.sleep(0.2)
            except Exception:
                pass
        self._client = None

    def get_health(self) -> dict[str, Any]:
        warnings = []
        if self._client is None:
            warnings.append(_warning("booster_t1.not_started", "Booster T1 high-level controller is not started"))
        return {"errors": [], "warnings": warnings}

    def get_mode(self) -> dict[str, Any]:
        self._ensure_started()
        resp = self._client.GetMode()
        mode = _mode_key(resp.mode)
        return {"mode": mode, "name": MODE_NAMES.get(mode, "unknown")}

    def prepare(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "prepare")
        return self._change_mode(self._sdk.RobotMode.kPrepare, "prepare")

    def walking(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "walking")
        return self._change_mode(self._sdk.RobotMode.kWalking, "walking")

    def damping(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "damping")
        self._velocity_stop.set()
        return self._change_mode(self._sdk.RobotMode.kDamping, "damping")

    def enter_walk(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "enter_walk")
        if self.get_mode()["name"] == "walking":
            return {"command": "enter_walk", "mode": self.get_mode()}
        steps = [self.prepare()]
        time.sleep(0.5)
        steps.append(self.walking())
        return {"command": "enter_walk", "mode": self.get_mode(), "steps": steps}

    def stop(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "stop")
        self._velocity_stop.set()
        stopped = self._stop_if_walking()
        return {"command": "stop", "sent_move": stopped, "vx": 0.0, "vy": 0.0, "vyaw": 0.0}

    def set_base_velocity(self, value: Mapping[str, Any]) -> dict[str, Any]:
        self._ensure_started()
        if self.get_mode()["name"] != "walking":
            raise RuntimeError("Booster T1 base_velocity requires walking mode; run action.robot.enter_walk first")
        self._velocity_stop.clear()
        linear = value.get("linear_vel") or [0.0, 0.0, 0.0]
        angular = value.get("angular_vel") or [0.0, 0.0, 0.0]
        vx = _clamp(float(linear[0]), self.max_x)
        vy = _clamp(float(linear[1]), self.max_y)
        vyaw = _clamp(float(angular[2]), self.max_yaw)
        frames = self._pulse_velocity(vx, vy, vyaw)
        return {"mode": self.get_mode(), "vx": vx, "vy": vy, "vyaw": vyaw, "duration_s": self.velocity_duration_s, "frames": frames}

    def get_up(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "get_up")
        self._ensure_started()
        self._check(self._client.GetUp(), "GetUp")
        return {"command": "get_up", "mode": self.get_mode()}

    def _change_mode(self, mode: Any, name: str) -> dict[str, Any]:
        self._ensure_started()
        self._check(self._client.ChangeMode(mode), f"ChangeMode({name})")
        return {"command": name, "mode": self.get_mode()}

    def _move(self, vx: float, vy: float, vyaw: float) -> None:
        self._ensure_started()
        self._check(self._client.Move(float(vx), float(vy), float(vyaw)), "Move")

    def _pulse_velocity(self, vx: float, vy: float, vyaw: float) -> int:
        frames = max(1, int(self.velocity_duration_s * self.velocity_hz))
        interval = 1.0 / self.velocity_hz
        sent = 0
        for _ in range(frames):
            if self._velocity_stop.is_set():
                break
            self._move(vx, vy, vyaw)
            sent += 1
            time.sleep(interval)
        self._move(0.0, 0.0, 0.0)
        return sent

    def _stop_if_walking(self) -> bool:
        if self.get_mode()["name"] != "walking":
            return False
        self._move(0.0, 0.0, 0.0)
        return True

    def _ensure_started(self) -> None:
        if self._client is None or self._sdk is None:
            raise RuntimeError("Booster T1 high-level controller is not started")

    @staticmethod
    def _check(ret: int | None, command: str) -> None:
        if ret is not None and int(ret) != 0:
            raise RuntimeError(f"{command} failed: error={ret}")


def _mode_key(mode: Any) -> str:
    return str(mode).rsplit(".", 1)[-1]


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _require_empty(value: Mapping[str, Any] | None, name: str) -> None:
    if value:
        raise ValueError(f"{name} action value must be empty")


def _warning(code: str, message: str) -> dict[str, Any]:
    return {"code": code, "message": message, "source": "robot", "timestamp": time.time()}
