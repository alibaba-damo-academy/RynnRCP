"""RynnRCP controller for the standalone LeKiwi driver."""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Mapping, Sequence
from typing import Any

from rynnrcp.robot.base_controller import BaseRobotController

from lerobot_lekiwi import ARM_MOTORS, LeKiwi


ARM_KEYS = tuple(f"{name}.pos" for name in ARM_MOTORS)
ARM_DOF = len(ARM_KEYS)
TELEOP_DOF = ARM_DOF + 3
HOME_POSITIONS = [0.0, -1.564, 1.564, 1.0, 0.0, 0.0]


class LeKiwiController(BaseRobotController):
    """LeKiwi controller with one serialized bus worker.

    Public RCP values use radians, a gripper ratio in [0, 1], m/s, and rad/s.
    The controller converts arm values to the driver's degrees and percent at
    the hardware boundary.
    """

    def __init__(
        self,
        port: str,
        robot_id: str,
        logger: logging.Logger | None = None,
        wheel_radius_m: float = 0.05,
        base_radius_m: float = 0.125,
        max_wheel_raw: int = 3000,
        max_x_m_s: float = 0.3,
        max_y_m_s: float = 0.3,
        max_yaw_rad_s: float = math.pi / 2.0,
        watchdog_timeout_s: float = 0.5,
        control_loop_hz: float = 60.0,
        state_read_hz: float = 30.0,
        max_joint_velocity_rad_s: float = 3.0,
        max_gripper_velocity_per_s: float = 3.0,
        target_timeout_s: float = 0.25,
        calibration_dir: str | None = None,
        auto_start_worker: bool = True,
        home_on_shutdown: bool = True,
    ) -> None:
        super().__init__(logger=logger)
        self.port = str(port)
        self.robot_id = str(robot_id)
        self.wheel_radius_m = _positive(wheel_radius_m, "wheel_radius_m")
        self.base_radius_m = _positive(base_radius_m, "base_radius_m")
        self.max_wheel_raw = int(max_wheel_raw)
        if not 1 <= self.max_wheel_raw <= 0x7FFF:
            raise ValueError("max_wheel_raw must be in [1, 32767]")
        self.max_x_m_s = _positive(max_x_m_s, "max_x_m_s")
        self.max_y_m_s = _positive(max_y_m_s, "max_y_m_s")
        self.max_yaw_rad_s = _positive(max_yaw_rad_s, "max_yaw_rad_s")
        self.watchdog_timeout_s = max(0.0, float(watchdog_timeout_s))
        self.control_loop_hz = _positive(control_loop_hz, "control_loop_hz")
        self.state_read_hz = _positive(state_read_hz, "state_read_hz")
        self.max_joint_velocity_rad_s = _positive(max_joint_velocity_rad_s, "max_joint_velocity_rad_s")
        self.max_gripper_velocity_per_s = _positive(
            max_gripper_velocity_per_s, "max_gripper_velocity_per_s"
        )
        self.target_timeout_s = max(0.0, float(target_timeout_s))
        self.calibration_dir = calibration_dir
        self.auto_start_worker = bool(auto_start_worker)
        self.home_on_shutdown = bool(home_on_shutdown)

        self._robot: LeKiwi | None = None
        self._io_lock = threading.RLock()
        self._state_lock = threading.RLock()
        self._action_lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        self._worker_stop = threading.Event()
        self._worker: threading.Thread | None = None
        self._arm_positions: list[float] | None = None
        self._base_velocity: dict[str, list[float]] | None = None
        self._last_arm_command: list[float] | None = None
        self._arm_target: list[float] | None = None
        self._arm_target_at: float | None = None
        self._pending_arm_action: dict[str, float] | None = None
        self._last_arm_signature: tuple[float, ...] | None = None
        self._pending_base_velocity: tuple[float, float, float] | None = None
        self._last_base_signature: tuple[float, float, float] | None = None
        self._last_base_command_at: float | None = None
        self._base_active = False
        self._motion_generation = 0
        self._worker_errors: dict[str, dict[str, Any]] = {}

    def start(self) -> None:
        with self._lifecycle_lock:
            if self._robot is None:
                self._robot = self._create_robot()
            if not self._robot.is_connected:
                self._robot.connect(require_calibration=True)
            self._cache_observation(self._robot.get_observation())
            if self.auto_start_worker:
                self._start_worker()

    def shutdown(self) -> None:
        with self._lifecycle_lock:
            if self._robot is None or not self._robot.is_connected:
                return
            if self.home_on_shutdown:
                try:
                    self.go_home(wait=True)
                except Exception as exc:
                    self.logger.warning("LeKiwi home during shutdown failed: %s", exc)
            self._stop_worker()
            with self._io_lock:
                self._robot.disconnect()

    def get_joint_positions(self) -> dict[str, list[float]]:
        with self._state_lock:
            if self._arm_positions is None or self._base_velocity is None:
                raise RuntimeError("LeKiwi has no combined state yet")
            linear = self._base_velocity["linear_vel"]
            angular = self._base_velocity["angular_vel"]
            return {
                "joint_positions": [
                    *self._arm_positions,
                    linear[0],
                    linear[1],
                    angular[2],
                ]
            }

    def set_joint_positions(self, value: Mapping[str, Any]) -> None:
        target = _joint_positions(value, TELEOP_DOF)
        arm_target = target[:ARM_DOF]
        base_command = self._bounded_base_command(*target[ARM_DOF:])
        with self._action_lock:
            self._motion_generation += 1
            self._pending_arm_action = None
            self._arm_target = arm_target
            self._arm_target_at = time.monotonic()
            self._pending_base_velocity = base_command
            self._last_base_command_at = time.monotonic()
            self._base_active = any(abs(item) > 1e-9 for item in base_command)

    def get_base_velocity(self) -> dict[str, list[float]]:
        with self._state_lock:
            if self._base_velocity is None:
                raise RuntimeError("LeKiwi has no base state yet")
            return {key: list(value) for key, value in self._base_velocity.items()}

    def set_base_velocity(self, value: Mapping[str, Any]) -> dict[str, list[float]]:
        linear = _vector(value.get("linear_vel"), "linear_vel")
        angular = _vector(value.get("angular_vel"), "angular_vel")
        command = self._bounded_base_command(linear[0], linear[1], angular[2])
        with self._action_lock:
            self._pending_base_velocity = command
            self._last_base_command_at = time.monotonic()
            self._base_active = any(abs(item) > 1e-9 for item in command)
        return _base_value(command)

    def stop_base(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "stop_base")
        with self._action_lock:
            self._pending_base_velocity = (0.0, 0.0, 0.0)
            self._last_base_command_at = time.monotonic()
            self._base_active = False
        return {"stopped": True}

    def go_home(
        self,
        value: Mapping[str, Any] | None = None,
        *,
        steps: int = 60,
        fps: int = 30,
        home_positions: Sequence[float] | None = None,
        wait: bool = True,
    ) -> dict[str, Any]:
        _require_empty(value, "home")
        target = _position_list(HOME_POSITIONS if home_positions is None else home_positions)
        start = self._get_arm_positions()
        generation = self._begin_motion()
        result: dict[str, Any] = {"start": start, "home": target, "frames_sent": 0, "fps": fps}

        def produce() -> None:
            interval = 1.0 / max(1, int(fps))
            for index in range(1, max(1, int(steps)) + 1):
                ratio = index / max(1, int(steps))
                arm = [a + (b - a) * ratio for a, b in zip(start[:-1], target[:-1])]
                half = max(1, int(steps) // 2)
                if index <= half:
                    grip = start[-1] + (1.0 - start[-1]) * index / half
                else:
                    grip = 1.0 + (target[-1] - 1.0) * (index - half) / max(1, int(steps) - half)
                if not self._queue_motion(arm + [grip], generation):
                    break
                result["frames_sent"] += 1
                if index < steps:
                    time.sleep(interval)
            result["completed"] = result["frames_sent"] == max(1, int(steps))

        thread = threading.Thread(target=produce, name=f"lekiwi-home-{self.robot_id}", daemon=True)
        thread.start()
        result["thread"] = thread.name
        if wait:
            thread.join()
        return result

    def move_to(self, value: Mapping[str, Any]) -> dict[str, Any]:
        target = _joint_positions(value, ARM_DOF)
        return self._interpolate(target, duration_s=2.0, fps=30)

    def stop_motion(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "stop_motion")
        with self._action_lock:
            self._motion_generation += 1
            self._pending_arm_action = None
            self._arm_target = None
            self._arm_target_at = None
        return {"stopped": True}

    def get_health(self) -> dict[str, Any]:
        warnings = []
        if self._robot is None or not self._robot.is_connected:
            warnings.append(self._health_warning("lekiwi.not_connected", "LeKiwi is not connected"))
        with self._state_lock:
            if self._arm_positions is None or self._base_velocity is None:
                warnings.append(self._health_warning("lekiwi.no_state", "LeKiwi has no cached state"))
        for scope, state in list(self._worker_errors.items()):
            warnings.append(
                self._health_warning(
                    f"lekiwi.{scope}",
                    str(state.get("message") or "LeKiwi worker error"),
                    error=str(state.get("error") or ""),
                )
            )
        return {"errors": [], "warnings": warnings}

    def units(self) -> dict[str, str]:
        return {
            "arm": "radians + gripper ratio_0_1",
            "base_linear": "m/s",
            "base_angular": "rad/s",
        }

    def _create_robot(self) -> LeKiwi:
        return LeKiwi(
            port=self.port,
            robot_id=self.robot_id,
            wheel_radius_m=self.wheel_radius_m,
            base_radius_m=self.base_radius_m,
            max_wheel_raw=self.max_wheel_raw,
            calibration_dir=self.calibration_dir,
            logger=self.logger,
        )

    def _start_worker(self) -> None:
        if self._worker is not None and self._worker.is_alive():
            return
        self._worker_stop.clear()
        self._worker = threading.Thread(target=self._worker_loop, name=f"lekiwi-bus-{self.robot_id}", daemon=True)
        self._worker.start()

    def _stop_worker(self) -> None:
        thread = self._worker
        if thread is None:
            return
        self._worker_stop.set()
        if thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._worker = None

    def _worker_loop(self) -> None:
        period = 1.0 / self.control_loop_hz
        state_period = 1.0 / self.state_read_hz
        next_tick = time.monotonic()
        next_read = 0.0
        while not self._worker_stop.is_set():
            next_tick += period
            now = time.monotonic()
            if now >= next_read:
                try:
                    with self._io_lock:
                        assert self._robot is not None
                        observation = self._robot.get_observation()
                    self._cache_observation(observation)
                    self._worker_errors.pop("state_read", None)
                except Exception as exc:
                    self._record_worker_error("state_read", "LeKiwi state read failed", exc)
                next_read = now + state_period

            arm_action = self._next_arm_action(period, now)
            if arm_action is not None:
                signature = tuple(round(arm_action[key], 6) for key in ARM_KEYS)
                if signature != self._last_arm_signature:
                    try:
                        with self._io_lock:
                            assert self._robot is not None
                            self._robot.send_arm_action(arm_action)
                        self._last_arm_signature = signature
                        self._worker_errors.pop("arm_write", None)
                    except Exception as exc:
                        with self._action_lock:
                            if self._pending_arm_action is None:
                                self._pending_arm_action = arm_action
                        self._record_worker_error("arm_write", "LeKiwi arm write failed", exc)

            base_command = self._next_base_command(now)
            if base_command is not None and base_command != self._last_base_signature:
                try:
                    with self._io_lock:
                        assert self._robot is not None
                        self._robot.send_base_velocity(*base_command)
                    self._last_base_signature = base_command
                    self._worker_errors.pop("base_write", None)
                except Exception as exc:
                    with self._action_lock:
                        if self._pending_base_velocity is None:
                            self._pending_base_velocity = base_command
                    self._record_worker_error("base_write", "LeKiwi base write failed", exc)

            sleep_s = max(0.0, next_tick - time.monotonic())
            if self._worker_stop.wait(sleep_s):
                break
            if sleep_s == 0.0:
                next_tick = time.monotonic()

    def _next_arm_action(self, period: float, now: float) -> dict[str, float] | None:
        with self._action_lock:
            pending = self._pending_arm_action
            self._pending_arm_action = None
            target = list(self._arm_target) if self._arm_target is not None else None
            target_at = self._arm_target_at
        if pending is not None:
            return pending
        if target is None:
            return None
        if self.target_timeout_s and target_at is not None and now - target_at > self.target_timeout_s:
            with self._action_lock:
                self._arm_target = None
                self._arm_target_at = None
            return None
        current = self._last_arm_command or self._get_arm_positions()
        result = []
        reached = True
        for index, (source, destination) in enumerate(zip(current, target)):
            limit = (self.max_gripper_velocity_per_s if index == 5 else self.max_joint_velocity_rad_s) * period
            delta = destination - source
            if abs(delta) <= limit:
                result.append(destination)
            else:
                result.append(source + math.copysign(limit, delta))
                reached = False
        result[-1] = max(0.0, min(1.0, result[-1]))
        self._last_arm_command = result
        if reached:
            with self._action_lock:
                self._arm_target = None
                self._arm_target_at = None
        return _positions_to_action(result)

    def _next_base_command(self, now: float) -> tuple[float, float, float] | None:
        with self._action_lock:
            pending = self._pending_base_velocity
            self._pending_base_velocity = None
            if pending is not None:
                return pending
            if (
                self._base_active
                and self.watchdog_timeout_s > 0.0
                and self._last_base_command_at is not None
                and now - self._last_base_command_at > self.watchdog_timeout_s
            ):
                self._base_active = False
                return (0.0, 0.0, 0.0)
        return None

    def _cache_observation(self, observation: Mapping[str, Any]) -> None:
        arm = [math.radians(float(observation[key])) for key in ARM_KEYS[:-1]]
        arm.append(max(0.0, min(1.0, float(observation[ARM_KEYS[-1]]) / 100.0)))
        base = {
            "linear_vel": [float(item) for item in observation["linear_vel"]],
            "angular_vel": [float(item) for item in observation["angular_vel"]],
        }
        with self._state_lock:
            self._arm_positions = arm
            self._base_velocity = base
        if self._last_arm_command is None:
            self._last_arm_command = list(arm)

    def _get_arm_positions(self) -> list[float]:
        with self._state_lock:
            if self._arm_positions is None:
                raise RuntimeError("LeKiwi has no arm state yet")
            return list(self._arm_positions)

    def _bounded_base_command(
        self,
        x_m_s: float,
        y_m_s: float,
        yaw_rad_s: float,
    ) -> tuple[float, float, float]:
        return (
            _clamp(x_m_s, self.max_x_m_s),
            _clamp(y_m_s, self.max_y_m_s),
            _clamp(yaw_rad_s, self.max_yaw_rad_s),
        )

    def _begin_motion(self) -> int:
        with self._action_lock:
            self._motion_generation += 1
            self._arm_target = None
            self._arm_target_at = None
            return self._motion_generation

    def _queue_motion(self, positions: Sequence[float], generation: int) -> bool:
        values = _position_list(positions)
        with self._action_lock:
            if generation != self._motion_generation:
                return False
            self._pending_arm_action = _positions_to_action(values)
            self._last_arm_command = values
            return True

    def _interpolate(self, target: Sequence[float], *, duration_s: float, fps: int) -> dict[str, Any]:
        start = self._get_arm_positions()
        generation = self._begin_motion()
        steps = max(1, int(duration_s * fps))
        sent = 0
        for index in range(1, steps + 1):
            ratio = index / steps
            values = [source + (destination - source) * ratio for source, destination in zip(start, target)]
            if not self._queue_motion(values, generation):
                break
            sent += 1
            if index < steps:
                time.sleep(1.0 / fps)
        return {"start": start, "target": list(target), "frames_sent": sent, "fps": fps}

    def _record_worker_error(self, scope: str, message: str, exc: Exception) -> None:
        previous = self._worker_errors.get(scope)
        error = f"{type(exc).__name__}: {exc}"
        self._worker_errors[scope] = {"message": message, "error": error, "timestamp": time.time()}
        if previous is None or previous.get("error") != error:
            self.logger.warning("%s: %s", message, error)

    def _health_warning(self, code: str, message: str, **details: Any) -> dict[str, Any]:
        return {
            "code": code,
            "message": message,
            "source": "robot",
            "timestamp": time.time(),
            "details": {"robot_id": self.robot_id, "port": self.port, **details},
        }


def _positions_to_action(positions: Sequence[float]) -> dict[str, float]:
    values = _position_list(positions)
    action = {key: math.degrees(value) for key, value in zip(ARM_KEYS[:-1], values[:-1])}
    action[ARM_KEYS[-1]] = values[-1] * 100.0
    return action


def _joint_positions(value: Mapping[str, Any], size: int) -> list[float]:
    if not isinstance(value, Mapping) or "joint_positions" not in value:
        raise ValueError("joint_position value requires joint_positions")
    return _position_list(value["joint_positions"], size)


def _position_list(value: Any, size: int = ARM_DOF) -> list[float]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError(f"LeKiwi positions must be a list of {size} values")
    result = [float(item) for item in value]
    if len(result) != size or not all(math.isfinite(item) for item in result):
        raise ValueError(f"LeKiwi positions must contain {size} finite values")
    result[ARM_DOF - 1] = max(0.0, min(1.0, result[ARM_DOF - 1]))
    return result


def _vector(value: Any, field: str) -> list[float]:
    if value is None:
        return [0.0, 0.0, 0.0]
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{field} must contain 3 values")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{field} must contain finite values")
    return result


def _base_value(command: tuple[float, float, float]) -> dict[str, list[float]]:
    return {"linear_vel": [command[0], command[1], 0.0], "angular_vel": [0.0, 0.0, command[2]]}


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, float(value)))


def _positive(value: float, field: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{field} must be positive")
    return result


def _require_empty(value: Mapping[str, Any] | None, action: str) -> None:
    if value is not None and (not isinstance(value, Mapping) or value):
        raise ValueError(f"{action} value must be an empty object")
