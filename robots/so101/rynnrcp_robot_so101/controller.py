#!/usr/bin/env python3
"""SO101 controller."""

from __future__ import annotations

import os
import logging
import math
import threading
import time
from collections.abc import Mapping, Sequence
from typing import Any, Callable, Dict, List

from lerobot_so101.position_trace import PositionTrace
from rynnrcp.robot.base_controller import BaseRobotController


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
LEROBOT_KEYS = [f"{name}.pos" for name in JOINT_NAMES]
ARM_KEYS = LEROBOT_KEYS[:5]
GRIPPER_KEY = LEROBOT_KEYS[-1]
HOME_POSITIONS = [
    -0.14,
    -1.49,
    1.64,
    0.80,
    0.31,
    0.1,
]
HOME_OPEN_GRIPPER = 1.0
HOME_STEPS = 60
HOME_FPS = 30

_DEFAULT_CONNECT_TIMEOUT_S = 20.0
_DEFAULT_DISCONNECT_COOLDOWN_S = 1.0
_WORKER_JOIN_TIMEOUT_S = 2.0
_DEFAULT_CONTROL_LOOP_HZ = 60.0
_DEFAULT_FOLLOWER_STATE_READ_HZ = 60.0
_DEFAULT_MAX_JOINT_VELOCITY_RAD_S = 30.0
_DEFAULT_MAX_GRIPPER_VELOCITY_PER_S = 30.0
_DEFAULT_TARGET_TOLERANCE = 1e-4
_DEFAULT_TARGET_TIMEOUT_S = 0.25
_WORKER_ERROR_LOG_INTERVAL_S = 5.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _env_bool(name: str) -> bool:
    return str(os.environ.get(name) or "").strip().lower() in {"1", "true", "yes", "on"}


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value in (None, ""):
        return float(default)
    return float(value)


class SO101Controller(BaseRobotController):
    """Controller for an SO101 follower arm.

    Public controller units:
    - first five joints: radians
    - final gripper value: normalized ratio in [0, 1]

    SO101 driver units:
    - first five joints: degrees
    - final gripper value: percent in [0, 100]
    """

    def __init__(
        self,
        port: str,
        robot_id: str,
        role: str = "follower",
        logger: logging.Logger | None = None,
        connect_timeout_s: float = _DEFAULT_CONNECT_TIMEOUT_S,
        disconnect_cooldown_s: float = _DEFAULT_DISCONNECT_COOLDOWN_S,
        auto_start_worker: bool = True,
        control_loop_hz: float = _DEFAULT_CONTROL_LOOP_HZ,
        state_read_hz: float | None = None,
        max_joint_velocity_rad_s: float = _DEFAULT_MAX_JOINT_VELOCITY_RAD_S,
        max_gripper_velocity_per_s: float = _DEFAULT_MAX_GRIPPER_VELOCITY_PER_S,
        target_tolerance: float = _DEFAULT_TARGET_TOLERANCE,
        target_timeout_s: float = _DEFAULT_TARGET_TIMEOUT_S,
    ) -> None:
        super().__init__(logger=logger)
        self.port = str(port)
        self.robot_id = str(robot_id)
        self.role = _normalize_role(role)
        self.connect_timeout_s = float(connect_timeout_s)
        self.disconnect_cooldown_s = float(disconnect_cooldown_s)
        self.auto_start_worker = bool(auto_start_worker)
        self.is_leader = self.role == "leader"
        self.control_loop_hz = max(1.0, _env_float("RYNNRCP_SO101_CONTROL_LOOP_HZ", control_loop_hz))
        default_state_read_hz = self.control_loop_hz if self.is_leader else _DEFAULT_FOLLOWER_STATE_READ_HZ
        self.state_read_hz = max(
            1.0,
            _env_float("RYNNRCP_SO101_STATE_READ_HZ", state_read_hz if state_read_hz is not None else default_state_read_hz),
        )
        self.max_joint_velocity_rad_s = max(
            1e-6,
            _env_float("RYNNRCP_SO101_MAX_JOINT_VELOCITY_RAD_S", max_joint_velocity_rad_s),
        )
        self.max_gripper_velocity_per_s = max(
            1e-6,
            _env_float("RYNNRCP_SO101_MAX_GRIPPER_VELOCITY_PER_S", max_gripper_velocity_per_s),
        )
        self.target_tolerance = max(0.0, float(target_tolerance))
        self.target_timeout_s = max(0.0, float(target_timeout_s))
        self.torque_on_connect = not self.is_leader
        self.go_home_on_disconnect = not self.is_leader
        self._robot: Any = None
        self._lock = threading.RLock()
        self._worker_lock = threading.RLock()
        self._worker_stop = threading.Event()
        self._worker_thread: threading.Thread | None = None
        self._state_lock = threading.RLock()
        self._latest_positions: List[float] | None = None
        self._action_lock = threading.RLock()
        self._pending_action: Dict[str, float] | None = None
        self._target_positions: List[float] | None = None
        self._target_updated_at: float | None = None
        self._last_commanded_positions: List[float] | None = None
        self._last_executed_action_signature: Any = None
        self._motion_generation = 0
        self._worker_error_logs: Dict[str, Dict[str, Any]] = {}
        self._trace = PositionTrace(
            enabled=_env_bool("RYNNRCP_SO101_TRACE_POSITIONS"),
            trace_dir=os.environ.get("RYNNRCP_SO101_TRACE_DIR") or ".",
            robot_id=self.robot_id,
            role=self.role,
            joint_names=JOINT_NAMES,
        )

    def start(self) -> None:
        with self._lock:
            loading_driver = self._robot is None
            if loading_driver:
                started = time.monotonic()
                print(
                    "[SO101 Controller] Loading lightweight SO101 drivers. Please WAIT.",
                    flush=True,
                )
            self._ensure_robot_instance()
            if loading_driver:
                print(
                    f"[SO101 Controller] SO101 drivers ready "
                    f"(load took {time.monotonic() - started:.2f}s).",
                    flush=True,
                )
            if not self._is_connected():
                self._connect_driver()
            positions = self._read_joint_positions_sync()
            self._set_cached_positions(positions)
            self._set_last_commanded_positions(positions)
        if self._is_connected() and self.auto_start_worker:
            self._start_bus_worker()

    def shutdown(self) -> None:
        """Apply configured shutdown policy and disconnect."""
        if self.go_home_on_disconnect:
            self.go_home()
        self._stop_bus_worker()
        with self._lock:
            if self._robot is not None and self._is_connected():
                self._disconnect_driver_locked()
        self._trace.flush()

    def _is_connected(self) -> bool:
        return bool(self._robot is not None and getattr(self._robot, "is_connected", False))

    def get_joint_positions(self) -> Dict[str, List[float]]:
        cached = self._cached_positions()
        if cached is not None:
            return {"joint_positions": cached}
        raise RuntimeError("SO101Controller has no cached joint state yet")

    def set_joint_positions(self, value: Dict[str, Any]) -> None:
        if not self.torque_on_connect:
            raise RuntimeError("SO101Controller is configured as read-only; torque is disabled")
        if not isinstance(value, dict) or "joint_positions" not in value:
            raise ValueError("joint_position action value requires joint_positions")
        values = [float(v) for v in value["joint_positions"]]
        if len(values) != len(JOINT_NAMES):
            raise ValueError(f"SO101 expects {len(JOINT_NAMES)} values, got {len(values)}")

        values[-1] = clamp(values[-1], 0.0, 1.0)
        self._trace.record_position("action", values)
        self._cancel_motion_sequence()
        self._set_target_positions(values)

    def get_health(self) -> Dict[str, Any]:
        warnings: List[Dict[str, Any]] = []
        if not self._is_connected():
            warnings.append(
                {
                    "code": "so101.not_connected",
                    "message": "SO101 controller is not connected",
                    "source": "robot",
                    "timestamp": time.time(),
                    "details": {
                        "robot_id": self.robot_id,
                        "role": self.role,
                        "port": self.port,
                    },
                }
            )
        with self._state_lock:
            has_state = self._latest_positions is not None
        if not has_state:
            warnings.append(
                {
                    "code": "so101.no_joint_state",
                    "message": "SO101 controller has no cached joint state",
                    "source": "robot",
                    "timestamp": time.time(),
                    "details": {
                        "robot_id": self.robot_id,
                        "role": self.role,
                        "port": self.port,
                    },
                }
            )
        for scope, state in list(self._worker_error_logs.items()):
            warnings.append(
                {
                    "code": f"so101.{scope}",
                    "message": str(state.get("message") or "SO101 controller worker error"),
                    "source": "robot",
                    "timestamp": float(state.get("wall_at") or time.time()),
                    "details": {
                        "robot_id": self.robot_id,
                        "role": self.role,
                        "port": self.port,
                        "scope": scope,
                        "error": str(state.get("error") or ""),
                        "suppressed": int(state.get("suppressed") or 0),
                    },
                }
            )
        return {"errors": [], "warnings": warnings}

    def go_home(
        self,
        value: Dict[str, Any] | None = None,
        *,
        steps: int = HOME_STEPS,
        fps: int = HOME_FPS,
        home_positions: Sequence[float] | None = None,
        wait: bool = True,
    ) -> Dict[str, Any]:
        """Move arm joints to home while the gripper opens and then closes."""
        if value is not None and value:
            raise ValueError("prearranged home action value must be an empty object")
        if not self.torque_on_connect:
            return {
                "skipped": True,
                "reason": "SO101Controller is configured as read-only; torque is disabled",
            }

        target = _joint_position_list(home_positions or HOME_POSITIONS, "home_positions")
        target[-1] = clamp(target[-1], 0.0, 1.0)
        interval_s = 1.0 / float(fps)
        start = self._cached_positions()
        if start is None:
            raise RuntimeError("SO101Controller has no cached joint state yet")

        result: Dict[str, Any] = {
            "start": start,
            "home": target,
            "frames_sent": 0,
            "fps": fps,
        }

        if self.auto_start_worker:
            self._start_bus_worker()

        motion_generation = self._begin_motion_sequence()

        def _producer() -> None:
            result["frames_sent"] = self._queue_home_motion(
                start,
                target,
                steps,
                interval_s,
                motion_generation=motion_generation,
            )
            result["completed"] = self._is_motion_current(motion_generation)
            result["cancelled"] = not result["completed"]

        producer = threading.Thread(
            target=_producer,
            name=f"so101-go-home-{self.role}-{self.port}",
            daemon=True,
        )
        producer.start()
        result["thread"] = producer.name

        if wait:
            producer.join()

        return result

    def move_to(self, value: Mapping[str, Any]) -> Dict[str, Any]:
        if not self.torque_on_connect:
            return {
                "skipped": True,
                "reason": "SO101Controller is configured as read-only; torque is disabled",
            }
        if not isinstance(value, Mapping):
            raise ValueError("move_to action value must be an object")
        target = _joint_position_list(value.get("joint_positions"), "joint_positions")
        interval_s = 1.0 / float(HOME_FPS)
        start = self._cached_positions()
        if start is None:
            raise RuntimeError("SO101Controller has no cached joint state yet")

        result: Dict[str, Any] = {
            "start": start,
            "target": target,
            "frames_sent": 0,
            "fps": HOME_FPS,
        }

        if self.auto_start_worker:
            self._start_bus_worker()

        motion_generation = self._begin_motion_sequence()

        def _producer() -> None:
            result["frames_sent"] = self._queue_interpolated_motion(
                start,
                target,
                HOME_STEPS,
                interval_s,
                motion_generation=motion_generation,
            )
            result["completed"] = self._is_motion_current(motion_generation)
            result["cancelled"] = not result["completed"]

        producer = threading.Thread(
            target=_producer,
            name=f"so101-move-to-{self.role}-{self.port}",
            daemon=True,
        )
        producer.start()
        result["thread"] = producer.name
        producer.join()
        return result

    def units(self) -> Dict[str, str]:
        return {
            "shoulder_pan": "rad",
            "shoulder_lift": "rad",
            "elbow_flex": "rad",
            "wrist_flex": "rad",
            "wrist_roll": "rad",
            "gripper": "ratio_0_1",
        }

    def disable_torque(self) -> None:
        self._stop_bus_worker()
        with self._lock:
            self._robot.bus.disable_torque(num_retry=5)

    def enable_torque(self) -> None:
        with self._lock:
            self._robot.bus.enable_torque(num_retry=5)
        if self.auto_start_worker:
            self._start_bus_worker()

    def _ensure_robot_instance(self) -> None:
        if self._robot is not None:
            return
        self.logger.info(
            "SO101 controller preparing driver instance: robot_id=%s role=%s port=%s",
            self.robot_id,
            self.role,
            self.port,
        )
        self._robot = self._create_robot()
        self.logger.info(
            "SO101 driver instance ready, will now open serial port and handshake with motors: "
            "robot_id=%s role=%s port=%s",
            self.robot_id,
            self.role,
            self.port,
        )

    def _connect_driver(self) -> None:
        self.logger.info(
            "Connecting SO101 controller (opening %s @ 1Mbps and pinging 6 sts3215 motors, "
            "usually <1s; first attempt after USB enumeration may briefly retry): "
            "robot_id=%s role=%s",
            self.port,
            self.robot_id,
            self.role,
        )
        started_at = time.monotonic()
        self._run_connect_with_timeout(lambda: self._robot.connect(calibrate=False))
        self.logger.info(
            "SO101 controller connected and ready: robot_id=%s role=%s port=%s duration=%.3fs",
            self.robot_id,
            self.role,
            self.port,
            time.monotonic() - started_at,
        )

    def _disconnect_driver_locked(self) -> None:
        if self._robot is None or not self._is_connected():
            return
        self._robot.disconnect()
        if self.disconnect_cooldown_s > 0:
            self.logger.info(
                "SO101 serial cooldown after disconnect: robot_id=%s role=%s port=%s duration=%.3fs",
                self.robot_id,
                self.role,
                self.port,
                self.disconnect_cooldown_s,
            )
            time.sleep(self.disconnect_cooldown_s)

    def _run_connect_with_timeout(self, fn: Callable[[], None]) -> None:
        if self.connect_timeout_s <= 0:
            fn()
            return

        result: Dict[str, Any] = {}

        def _target() -> None:
            try:
                fn()
            except BaseException as exc:
                result["error"] = exc

        worker = threading.Thread(
            target=_target,
            name=f"so101-connect-{self.role}-{self.port}",
            daemon=True,
        )
        worker.start()
        worker.join(timeout=self.connect_timeout_s)
        if worker.is_alive():
            message = (
                f"Timed out connecting SO101 {self.role} arm on {self.port} "
                f"after {self.connect_timeout_s:.1f}s. SO101 driver connect did not finish "
                "while reading calibration/configuring motors. Check calibration cache, motor "
                "firmware responses, USB stability, and stale MCP/Teleop/calibration processes."
            )
            self.logger.error(
                "%s robot_id=%s role=%s port=%s",
                message,
                self.robot_id,
                self.role,
                self.port,
            )
            raise TimeoutError(message)

        error = result.get("error")
        if error is not None:
            raise error

    def _create_robot(self) -> Any:
        if self.is_leader:
            from lerobot_so101 import SO101Leader, SO101LeaderConfig

            driver_config = SO101LeaderConfig(
                port=self.port,
                id=self.robot_id,
                use_degrees=True,
            )
            return SO101Leader(driver_config)

        from lerobot_so101 import SO101Follower, SO101FollowerConfig

        driver_config = SO101FollowerConfig(
            port=self.port,
            id=self.robot_id,
            use_degrees=True,
            disable_torque_on_disconnect=True,
        )
        return SO101Follower(driver_config)

    def _obs_to_positions(self, obs: Dict[str, Any]) -> List[float]:
        arm_values = [float(obs[key]) for key in ARM_KEYS]
        arm_positions = [math.radians(v) for v in arm_values]

        gripper_percent = float(obs[GRIPPER_KEY])
        gripper_ratio = clamp(gripper_percent / 100.0, 0.0, 1.0)
        return arm_positions + [gripper_ratio]

    def _positions_to_action(self, positions: Sequence[float]) -> Dict[str, float]:
        arm_rad = [float(v) for v in positions[:5]]
        arm_targets = [math.degrees(v) for v in arm_rad]

        gripper_percent = clamp(float(positions[-1]), 0.0, 1.0) * 100.0
        action = {key: value for key, value in zip(ARM_KEYS, arm_targets)}
        action[GRIPPER_KEY] = gripper_percent
        return action

    def _action_to_positions(self, action: Mapping[str, Any]) -> List[float]:
        arm_positions = [math.radians(float(action[key])) for key in ARM_KEYS]
        gripper_ratio = clamp(float(action[GRIPPER_KEY]) / 100.0, 0.0, 1.0)
        return arm_positions + [gripper_ratio]

    def _cached_positions(self) -> List[float] | None:
        with self._state_lock:
            if self._latest_positions is None:
                return None
            return list(self._latest_positions)

    def _set_cached_positions(self, positions: Sequence[float]) -> None:
        values = [float(v) for v in positions]
        with self._state_lock:
            self._latest_positions = list(values)
        self._trace.record_position("state", values)

    def _read_joint_positions_sync(self) -> List[float]:
        started = time.perf_counter()
        if self.is_leader:
            obs = self._robot.get_action()
        else:
            obs = self._robot.get_observation()
        self._trace.record_timing("read_state", (time.perf_counter() - started) * 1000.0)
        return self._obs_to_positions(obs)

    def _send_action_sync(self, action: Dict[str, float]) -> None:
        self._trace.record_position("sent", self._action_to_positions(action))
        started = time.perf_counter()
        self._robot.send_action(action)
        self._trace.record_timing("send_action", (time.perf_counter() - started) * 1000.0)

    def _start_bus_worker(self) -> None:
        with self._worker_lock:
            if self._worker_thread is not None and self._worker_thread.is_alive():
                return
            if not self._is_connected():
                return
            self._worker_stop.clear()
            self._worker_thread = threading.Thread(
                target=self._bus_worker_loop,
                name=f"so101-bus-{self.role}-{self.port}",
                daemon=True,
            )
            self._worker_thread.start()
            self.logger.info(
                "SO101 controller bus worker started: robot_id=%s role=%s port=%s "
                "control_loop_hz=%.1f state_read_hz=%.1f",
                self.robot_id,
                self.role,
                self.port,
                self.control_loop_hz,
                self.state_read_hz,
            )

    def _stop_bus_worker(self) -> bool:
        with self._worker_lock:
            thread = self._worker_thread
            if thread is None:
                return False
            self._worker_stop.set()
        if thread is not threading.current_thread():
            thread.join(timeout=_WORKER_JOIN_TIMEOUT_S)
            if thread.is_alive():
                self.logger.warning(
                    "SO101 controller bus worker did not stop within %.1fs: robot_id=%s role=%s port=%s",
                    _WORKER_JOIN_TIMEOUT_S,
                    self.robot_id,
                    self.role,
                    self.port,
                )
        stopped = thread is threading.current_thread() or not thread.is_alive()
        with self._worker_lock:
            if self._worker_thread is thread and stopped:
                self._worker_thread = None
                self._worker_stop.clear()
        return stopped

    def _take_pending_action(self) -> Dict[str, float] | None:
        with self._action_lock:
            action = self._pending_action
            self._pending_action = None
            return dict(action) if action is not None else None

    def _queue_action(self, action: Dict[str, float]) -> None:
        with self._action_lock:
            self._pending_action = dict(action)

    def _queue_motion_action_if_current(
        self,
        action: Dict[str, float],
        motion_generation: int,
    ) -> bool:
        with self._action_lock:
            if motion_generation != self._motion_generation:
                return False
            self._pending_action = dict(action)
            return True

    def _begin_motion_sequence(self) -> int:
        with self._action_lock:
            self._motion_generation += 1
            self._pending_action = None
            self._target_positions = None
            self._target_updated_at = None
            return self._motion_generation

    def _cancel_motion_sequence(self) -> None:
        with self._action_lock:
            self._motion_generation += 1
            self._pending_action = None
            self._target_positions = None
            self._target_updated_at = None

    def _is_motion_current(self, motion_generation: int) -> bool:
        with self._action_lock:
            return motion_generation == self._motion_generation

    def _set_target_positions(self, positions: Sequence[float]) -> None:
        with self._action_lock:
            self._target_positions = [float(v) for v in positions]
            self._target_updated_at = time.monotonic()

    def _set_last_commanded_positions(self, positions: Sequence[float]) -> None:
        with self._action_lock:
            self._last_commanded_positions = [float(v) for v in positions]

    def _current_tracking_positions(self) -> List[float] | None:
        with self._action_lock:
            if self._last_commanded_positions is not None:
                return list(self._last_commanded_positions)
        return self._cached_positions()

    def _next_target_tracking_action(
        self,
        period_s: float,
        now_mono: float | None = None,
    ) -> tuple[Dict[str, float], List[float], List[float], bool] | None:
        with self._action_lock:
            target = list(self._target_positions) if self._target_positions is not None else None
            target_updated_at = self._target_updated_at
        if target is None:
            return None
        if self._target_expired(target_updated_at, now_mono):
            self._clear_target_positions()
            return None

        current = self._current_tracking_positions()
        if current is None:
            return None

        next_positions, reached = self._step_toward_target(current, target, period_s)
        if reached and _positions_close(current, target, self.target_tolerance):
            with self._action_lock:
                if _positions_close(self._target_positions, target, self.target_tolerance):
                    self._target_positions = None
                    self._target_updated_at = None
            return None
        return self._positions_to_action(next_positions), next_positions, target, reached

    def _target_expired(self, target_updated_at: float | None, now_mono: float | None = None) -> bool:
        if self.target_timeout_s <= 0.0 or target_updated_at is None:
            return False
        now = time.monotonic() if now_mono is None else float(now_mono)
        return now - target_updated_at > self.target_timeout_s

    def _clear_target_positions(self) -> None:
        with self._action_lock:
            self._target_positions = None
            self._target_updated_at = None

    def _mark_tracking_action_sent(
        self,
        positions: Sequence[float],
        target: Sequence[float],
        reached: bool,
    ) -> None:
        self._set_last_commanded_positions(positions)
        if reached:
            with self._action_lock:
                if _positions_close(self._target_positions, target, self.target_tolerance):
                    self._target_positions = None
                    self._target_updated_at = None

    def _step_toward_target(
        self,
        current: Sequence[float],
        target: Sequence[float],
        period_s: float,
    ) -> tuple[List[float], bool]:
        interval_s = max(0.0, float(period_s))
        arm_step = self.max_joint_velocity_rad_s * interval_s
        gripper_step = self.max_gripper_velocity_per_s * interval_s
        next_positions: List[float] = []
        reached = True

        for index, (current_value, target_value) in enumerate(zip(current, target)):
            max_step = gripper_step if index == len(JOINT_NAMES) - 1 else arm_step
            delta = float(target_value) - float(current_value)
            if abs(delta) <= max(max_step, self.target_tolerance):
                next_positions.append(float(target_value))
                continue
            reached = False
            next_positions.append(float(current_value) + math.copysign(max_step, delta))

        next_positions[-1] = clamp(next_positions[-1], 0.0, 1.0)
        return next_positions, reached

    def _bus_worker_loop(self) -> None:
        period_s = 1.0 / self.control_loop_hz
        state_period_s = 1.0 / self.state_read_hz
        next_tick = time.monotonic()
        next_state_read = 0.0
        while not self._worker_stop.is_set():
            loop_started = time.perf_counter()
            try:
                next_tick += period_s
                now_mono = time.monotonic()

                should_read_state = self._latest_positions is None or now_mono >= next_state_read
                if should_read_state:
                    try:
                        self._set_cached_positions(self._read_joint_positions_sync())
                        self._clear_worker_error("state_read")
                    except Exception as exc:
                        self._log_worker_error("state_read", "SO101 controller state read failed", exc)
                    next_state_read = max(now_mono + state_period_s, next_state_read + state_period_s)

                tracking_action: tuple[Dict[str, float], List[float], List[float], bool] | None = None
                action = self._take_pending_action()
                if action is None:
                    tracking_action = self._next_target_tracking_action(period_s, now_mono=now_mono)
                    if tracking_action is not None:
                        action = tracking_action[0]
                action_changed = action is not None and self._is_new_action(action)
                if action_changed:
                    try:
                        self._send_action_sync(action)
                        self._last_executed_action_signature = _action_signature(action)
                        if tracking_action is not None:
                            self._mark_tracking_action_sent(
                                tracking_action[1],
                                tracking_action[2],
                                tracking_action[3],
                            )
                        self._clear_worker_error("send_action")
                    except Exception as exc:
                        self._log_worker_error("send_action", "SO101 controller action send failed", exc)
                elif tracking_action is not None and tracking_action[3]:
                    self._mark_tracking_action_sent(
                        tracking_action[1],
                        tracking_action[2],
                        tracking_action[3],
                    )

                sleep_s = max(0.0, next_tick - time.monotonic())
                if self._worker_stop.wait(timeout=sleep_s):
                    break
                if sleep_s <= 0.0:
                    next_tick = time.monotonic()
            except Exception as exc:
                if not self._worker_stop.is_set():
                    self._log_worker_error("worker", "SO101 controller bus worker error", exc)
                self._worker_stop.wait(0.05)
            finally:
                self._trace.record_timing("worker_tick", (time.perf_counter() - loop_started) * 1000.0)

    def _log_worker_error(self, scope: str, message: str, exc: Exception) -> None:
        now = time.monotonic()
        error = str(exc)
        key = f"{type(exc).__name__}:{error}"
        state = self._worker_error_logs.get(scope)
        if state is None or state.get("key") != key:
            self._worker_error_logs[scope] = {
                "key": key,
                "last_at": now,
                "wall_at": time.time(),
                "suppressed": 0,
                "message": message,
                "error": error,
            }
            self.logger.warning(
                "%s: robot_id=%s role=%s port=%s error=%s",
                message,
                self.robot_id,
                self.role,
                self.port,
                error,
            )
            return

        suppressed = int(state.get("suppressed") or 0) + 1
        if now - float(state.get("last_at") or 0.0) >= _WORKER_ERROR_LOG_INTERVAL_S:
            self.logger.warning(
                "%s: robot_id=%s role=%s port=%s error=%s (suppressed %d repeated errors)",
                message,
                self.robot_id,
                self.role,
                self.port,
                error,
                suppressed,
            )
            state["last_at"] = now
            state["suppressed"] = 0
        else:
            state["suppressed"] = suppressed

    def _clear_worker_error(self, scope: str) -> None:
        self._worker_error_logs.pop(scope, None)

    def _is_new_action(self, action: Dict[str, float]) -> bool:
        return _action_signature(action) != self._last_executed_action_signature

    def _queue_interpolated_motion(
        self,
        start: Sequence[float],
        target: Sequence[float],
        steps: int,
        interval_s: float,
        *,
        motion_generation: int | None = None,
    ) -> int:
        frames_sent = 0
        start_values = [float(v) for v in start]
        target_values = [float(v) for v in target]
        for index in range(1, steps + 1):
            ratio = index / float(steps)
            frame = [
                s + (t - s) * ratio
                for s, t in zip(start_values, target_values)
            ]
            action = self._positions_to_action(frame)
            if motion_generation is None:
                self._queue_action(action)
            elif not self._queue_motion_action_if_current(action, motion_generation):
                return frames_sent
            self._set_last_commanded_positions(frame)
            frames_sent += 1
            if index < steps:
                time.sleep(interval_s)
        return frames_sent

    def _queue_home_motion(
        self,
        start: Sequence[float],
        target: Sequence[float],
        steps: int,
        interval_s: float,
        *,
        motion_generation: int,
    ) -> int:
        frames_sent = 0
        start_values = [float(v) for v in start]
        target_values = [float(v) for v in target]
        open_value = clamp(HOME_OPEN_GRIPPER, 0.0, 1.0)
        close_value = clamp(target_values[-1], 0.0, 1.0)
        open_steps = max(1, steps // 2)
        close_steps = max(1, steps - open_steps)

        for index in range(1, steps + 1):
            arm_ratio = index / float(steps)
            frame = [
                s + (t - s) * arm_ratio
                for s, t in zip(start_values[:-1], target_values[:-1])
            ]
            if index <= open_steps:
                gripper_ratio = index / float(open_steps)
                frame.append(start_values[-1] + (open_value - start_values[-1]) * gripper_ratio)
            else:
                gripper_ratio = (index - open_steps) / float(close_steps)
                frame.append(open_value + (close_value - open_value) * gripper_ratio)
            action = self._positions_to_action(frame)
            if not self._queue_motion_action_if_current(action, motion_generation):
                return frames_sent
            self._set_last_commanded_positions(frame)
            frames_sent += 1
            if index < steps:
                time.sleep(interval_s)
        return frames_sent


def _normalize_role(role: str) -> str:
    normalized = str(role).strip().lower()
    if normalized in {"leader", "follower"}:
        return normalized
    raise ValueError("SO101 role must be 'leader' or 'follower'")


def _joint_position_list(value: Any, key: str) -> List[float]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError(f"{key} must be a list of {len(JOINT_NAMES)} joint values")
    result = [float(item) for item in value]
    if len(result) != len(JOINT_NAMES):
        raise ValueError(f"{key} must contain {len(JOINT_NAMES)} values")
    return result


def _positions_close(
    left: Sequence[float] | None,
    right: Sequence[float] | None,
    tolerance: float,
) -> bool:
    if left is None or right is None:
        return False
    if len(left) != len(right):
        return False
    return all(abs(float(a) - float(b)) <= tolerance for a, b in zip(left, right))


def _action_signature(action: Dict[str, float]) -> Any:
    return (
        "dict",
        tuple(
            sorted(
                (str(key), _value_signature(value))
                for key, value in action.items()
            )
        ),
    )


def _value_signature(value: Any) -> Any:
    if isinstance(value, dict):
        return (
            "dict",
            tuple(sorted((str(key), _value_signature(val)) for key, val in value.items())),
        )
    if isinstance(value, (list, tuple)):
        return ("seq", tuple(_value_signature(item) for item in value))
    if isinstance(value, bool):
        return ("bool", value)
    if isinstance(value, (int, float)):
        number = float(value)
        if math.isfinite(number):
            return ("num", round(number, 6))
        return ("num", str(number))
    if value is None:
        return ("none", None)
    return ("value", str(value))
