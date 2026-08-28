"""LeKiwi leader-arm and keyboard teleoperation controller."""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any

from rynnrcp.robot.base_controller import BaseRobotController

from lerobot_lekiwi import ARM_MOTORS, LeKiwiLeader


ARM_KEYS = tuple(f"{name}.pos" for name in ARM_MOTORS)


class KeyboardBaseController:
    """Track held keys and expose the latest RCP base velocity."""

    def __init__(
        self,
        *,
        slow_xy_m_s: float = 0.1,
        medium_xy_m_s: float = 0.2,
        fast_xy_m_s: float = 0.3,
        slow_yaw_rad_s: float = math.pi / 6,
        medium_yaw_rad_s: float = math.pi / 3,
        fast_yaw_rad_s: float = math.pi / 2,
    ) -> None:
        self._levels = (
            (float(slow_xy_m_s), float(slow_yaw_rad_s)),
            (float(medium_xy_m_s), float(medium_yaw_rad_s)),
            (float(fast_xy_m_s), float(fast_yaw_rad_s)),
        )
        if any(xy <= 0.0 or yaw <= 0.0 for xy, yaw in self._levels):
            raise ValueError("Keyboard speed levels must be positive")
        self._speed_index = 0
        self._pressed: set[str] = set()
        self._lock = threading.RLock()
        self._listener: Any = None

    def start(self) -> None:
        if self._listener is not None:
            return
        try:
            from pynput import keyboard

            listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
            listener.start()
        except Exception as exc:
            raise RuntimeError(f"Could not start keyboard listener: {exc}") from exc
        self._listener = listener

    def stop(self) -> None:
        listener = self._listener
        self._listener = None
        with self._lock:
            self._pressed.clear()
        if listener is not None:
            listener.stop()

    def velocity(self) -> tuple[float, float, float]:
        with self._lock:
            keys = set(self._pressed)
            xy, yaw = self._levels[self._speed_index]
        if "space" in keys:
            return (0.0, 0.0, 0.0)
        return (
            xy * (int("w" in keys) - int("s" in keys)),
            xy * (int("a" in keys) - int("d" in keys)),
            yaw * (int("q" in keys) - int("e" in keys)),
        )

    @property
    def speed_level(self) -> int:
        with self._lock:
            return self._speed_index + 1

    def _on_press(self, key: Any) -> None:
        name = _key_name(key)
        if not name:
            return
        with self._lock:
            if name in self._pressed:
                return
            self._pressed.add(name)
            if name == "]":
                self._speed_index = min(2, self._speed_index + 1)
            elif name == "[":
                self._speed_index = max(0, self._speed_index - 1)

    def _on_release(self, key: Any) -> None:
        name = _key_name(key)
        if name:
            with self._lock:
                self._pressed.discard(name)


class LeKiwiLeaderController(BaseRobotController):
    """Combine a read-only leader arm and keyboard state into one 9D value."""

    def __init__(
        self,
        port: str,
        robot_id: str,
        logger: logging.Logger | None = None,
        calibration_dir: str | None = None,
        state_read_hz: float = 60.0,
        slow_xy_m_s: float = 0.1,
        medium_xy_m_s: float = 0.2,
        fast_xy_m_s: float = 0.3,
        slow_yaw_rad_s: float = math.pi / 6,
        medium_yaw_rad_s: float = math.pi / 3,
        fast_yaw_rad_s: float = math.pi / 2,
    ) -> None:
        super().__init__(logger=logger)
        self.port = str(port)
        self.robot_id = str(robot_id)
        self.calibration_dir = calibration_dir
        self.state_read_hz = float(state_read_hz)
        if not math.isfinite(self.state_read_hz) or self.state_read_hz <= 0.0:
            raise ValueError("state_read_hz must be positive")
        self.keyboard = KeyboardBaseController(
            slow_xy_m_s=slow_xy_m_s,
            medium_xy_m_s=medium_xy_m_s,
            fast_xy_m_s=fast_xy_m_s,
            slow_yaw_rad_s=slow_yaw_rad_s,
            medium_yaw_rad_s=medium_yaw_rad_s,
            fast_yaw_rad_s=fast_yaw_rad_s,
        )
        self._leader: LeKiwiLeader | None = None
        self._arm_positions: list[float] | None = None
        self._state_lock = threading.RLock()
        self._io_lock = threading.RLock()
        self._stop = threading.Event()
        self._worker: threading.Thread | None = None
        self._last_error = ""

    def start(self) -> None:
        if self._leader is None:
            self._leader = LeKiwiLeader(
                port=self.port,
                robot_id=self.robot_id,
                calibration_dir=self.calibration_dir,
                logger=self.logger,
            )
        if not self._leader.is_connected:
            self._leader.connect(require_calibration=True)
        try:
            self._read_arm()
            self.keyboard.start()
        except Exception:
            self._leader.disconnect()
            raise
        self._stop.clear()
        self._worker = threading.Thread(target=self._worker_loop, name=f"lekiwi-leader-{self.robot_id}", daemon=True)
        self._worker.start()

    def shutdown(self) -> None:
        self.keyboard.stop()
        self._stop.set()
        thread = self._worker
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._worker = None
        if self._leader is not None:
            with self._io_lock:
                self._leader.disconnect()

    def get_joint_positions(self) -> dict[str, list[float]]:
        with self._state_lock:
            if self._arm_positions is None:
                raise RuntimeError("LeKiwi leader has no arm state yet")
            arm = list(self._arm_positions)
        return {"joint_positions": [*arm, *self.keyboard.velocity()]}

    def get_health(self) -> dict[str, Any]:
        warnings = []
        if self._leader is None or not self._leader.is_connected:
            warnings.append(self._warning("lekiwi_leader.not_connected", "LeKiwi leader is not connected"))
        if self._last_error:
            warnings.append(self._warning("lekiwi_leader.read_failed", self._last_error))
        return {"errors": [], "warnings": warnings}

    def units(self) -> dict[str, str]:
        return {
            "arm": "radians + gripper ratio_0_1",
            "base_linear": "m/s",
            "base_angular": "rad/s",
        }

    def _worker_loop(self) -> None:
        period = 1.0 / self.state_read_hz
        while not self._stop.wait(period):
            try:
                self._read_arm()
                self._last_error = ""
            except Exception as exc:
                message = f"{type(exc).__name__}: {exc}"
                if message != self._last_error:
                    self.logger.warning("LeKiwi leader state read failed: %s", message)
                self._last_error = message

    def _read_arm(self) -> None:
        with self._io_lock:
            assert self._leader is not None
            state = self._leader.get_action()
        arm = [math.radians(float(state[key])) for key in ARM_KEYS[:-1]]
        arm.append(max(0.0, min(1.0, float(state[ARM_KEYS[-1]]) / 100.0)))
        with self._state_lock:
            self._arm_positions = arm

    def _warning(self, code: str, message: str) -> dict[str, Any]:
        return {
            "code": code,
            "message": message,
            "source": "robot",
            "timestamp": time.time(),
            "details": {"robot_id": self.robot_id, "port": self.port},
        }


def _key_name(key: Any) -> str:
    char = getattr(key, "char", None)
    if isinstance(char, str):
        return char.lower()
    return "space" if str(key) == "Key.space" else ""
