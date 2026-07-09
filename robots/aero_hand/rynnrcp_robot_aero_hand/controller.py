"""Aero Hand controller for RynnRCP."""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Mapping
from typing import Any, Dict, List

from rynnrcp.robot.base_controller import BaseRobotController


JOINT_NAMES = (
    "thumb_cmc_abd",
    "thumb_cmc_flex",
    "thumb_mcp_ip",
    "index_finger",
    "middle_finger",
    "ring_finger",
    "pinky_finger",
)
JOINT_LIMITS_DEG = (
    (0.0, 100.0),
    (0.0, 55.0),
    (0.0, 90.0),
    (0.0, 90.0),
    (0.0, 90.0),
    (0.0, 90.0),
    (0.0, 90.0),
)
DUAL_HAND_ORDER = ("left", "right")
_CONTROL_LOOP_HZ = 60.0
_MAX_JOINT_VELOCITY_RAD_S = 5.0
_TARGET_SMOOTHING = 0.18
_TARGET_TOLERANCE_RAD = 1e-3
_WORKER_JOIN_TIMEOUT_S = 2.0
_WORKER_ERROR_LOG_INTERVAL_S = 5.0


class AeroHandController(BaseRobotController):
    """Map one Aero Hand robot profile to RCP protocol values.

    RCP joint values are radians. The Aero Hand SDK uses degrees, so conversion
    stays here at the hardware boundary. A cloud profile is either one 7-DoF
    hand or one 14-DoF dual-hand robot; the RCP surface remains one robot vector.
    """

    def __init__(
        self,
        robot_id: str,
        mode: str = "single",
        port: str | None = None,
        left_port: str | None = None,
        right_port: str | None = None,
        baudrate: int = 921600,
        homing_on_connect: bool = False,
        homing_timeout_s: float = 175.0,
        homing_settle_s: float = 8.0,
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__(logger=logger)
        self.robot_id = str(robot_id)
        self.mode = _validate_mode(mode)
        self.port = port
        self.left_port = left_port
        self.right_port = right_port
        self.baudrate = int(baudrate)
        self.homing_on_connect = bool(homing_on_connect)
        self.homing_timeout_s = float(homing_timeout_s)
        self.homing_settle_s = max(0.0, float(homing_settle_s))
        self._hands: Dict[str, Any] = {}
        self._lock = threading.RLock()
        self._target_lock = threading.RLock()
        self._worker_lock = threading.RLock()
        self._worker_stop = threading.Event()
        self._worker_thread: threading.Thread | None = None
        self._latest_positions: List[float] | None = None
        self._last_commanded_positions: List[float] | None = None
        self._target_positions: List[float] | None = None
        self._last_worker_error: dict[str, Any] | None = None

    def start(self) -> None:
        with self._lock:
            if self._is_connected():
                return
            from .aero_open_sdk.aero_hand import AeroHand

            try:
                for hand_side in self._active_hands():
                    self._hands[hand_side] = AeroHand(port=self._port_for(hand_side), baudrate=self.baudrate)
                if self.homing_on_connect:
                    for hand in self._hands.values():
                        hand.send_homing(timeout_s=self.homing_timeout_s)
                    if self.homing_settle_s:
                        time.sleep(self.homing_settle_s)
                positions = self._read_joint_positions_locked()
                self._last_commanded_positions = list(positions)
            except Exception:
                self.shutdown()
                raise
        self._start_worker()

    def shutdown(self) -> None:
        self._stop_worker()
        with self._lock:
            hands = list(self._hands.values())
            self._hands.clear()
            for hand in hands:
                try:
                    hand.close()
                except Exception as exc:
                    self.logger.warning("Aero Hand close failed: %s", exc)

    def _is_connected(self) -> bool:
        active = self._active_hands()
        if set(self._hands) != set(active):
            return False
        return all(getattr(getattr(hand, "ser", None), "is_open", True) for hand in self._hands.values())

    def get_joint_positions(self) -> Dict[str, List[float]]:
        with self._lock:
            if self._is_connected():
                try:
                    return {"joint_positions": self._read_joint_positions_locked()}
                except Exception as exc:
                    self.logger.warning("Aero Hand state read failed: %s", exc)
            if self._latest_positions is None:
                raise RuntimeError("AeroHandController has no cached joint state yet")
            return {"joint_positions": list(self._latest_positions)}

    def set_joint_positions(self, value: Dict[str, Any]) -> Dict[str, List[float]]:
        if not isinstance(value, Mapping):
            raise TypeError("joint_position action value must be an object")
        positions = _joint_position_list(value.get("joint_positions"))
        if len(positions) != self._expected_dof():
            raise ValueError(f"Aero Hand {self.mode} mode expects {self._expected_dof()} joints, got {len(positions)}")
        with self._lock:
            if not self._is_connected():
                raise RuntimeError("AeroHandController is not connected")
            with self._target_lock:
                self._target_positions = list(positions)
        return {"joint_positions": positions}

    def get_health(self) -> Dict[str, Any]:
        warnings: List[Dict[str, Any]] = []
        if not self._is_connected():
            warnings.append(
                {
                    "code": "aero_hand.not_connected",
                    "message": "Aero Hand controller is not connected",
                    "source": "robot",
                    "timestamp": time.time(),
                    "details": {
                        "robot_id": self.robot_id,
                        "mode": self.mode,
                        "port": self.port,
                        "left_port": self.left_port,
                        "right_port": self.right_port,
                    },
                }
            )
        if self._latest_positions is None:
            warnings.append(
                {
                    "code": "aero_hand.no_joint_state",
                    "message": "Aero Hand controller has no cached joint state",
                    "source": "robot",
                    "timestamp": time.time(),
                    "details": {"robot_id": self.robot_id, "mode": self.mode},
                }
            )
        if self._last_worker_error is not None:
            warnings.append(
                {
                    "code": "aero_hand.worker_error",
                    "message": "Aero Hand background command worker error",
                    "source": "robot",
                    "timestamp": float(self._last_worker_error.get("timestamp") or time.time()),
                    "details": dict(self._last_worker_error),
                }
            )
        return {"errors": [], "warnings": warnings}

    def _start_worker(self) -> None:
        with self._worker_lock:
            if self._worker_thread is not None and self._worker_thread.is_alive():
                return
            self._worker_stop.clear()
            self._worker_thread = threading.Thread(
                target=self._worker_loop,
                name=f"aero-hand-command-{self.robot_id}",
                daemon=True,
            )
            self._worker_thread.start()

    def _stop_worker(self) -> None:
        with self._worker_lock:
            thread = self._worker_thread
            if thread is None:
                return
            self._worker_stop.set()
        if thread is not threading.current_thread():
            thread.join(timeout=_WORKER_JOIN_TIMEOUT_S)
        with self._worker_lock:
            if self._worker_thread is thread and not thread.is_alive():
                self._worker_thread = None
                self._worker_stop.clear()

    def _worker_loop(self) -> None:
        period_s = 1.0 / _CONTROL_LOOP_HZ
        next_tick = time.monotonic()
        last_error_log = 0.0
        while not self._worker_stop.is_set():
            try:
                next_tick += period_s
                self._send_interpolated_target(period_s)
                self._last_worker_error = None
                sleep_s = max(0.0, next_tick - time.monotonic())
                if self._worker_stop.wait(sleep_s):
                    break
                if sleep_s <= 0.0:
                    next_tick = time.monotonic()
            except Exception as exc:
                now = time.monotonic()
                self._last_worker_error = {
                    "error": str(exc),
                    "error_type": type(exc).__name__,
                    "timestamp": time.time(),
                }
                if now - last_error_log >= _WORKER_ERROR_LOG_INTERVAL_S:
                    self.logger.warning("Aero Hand command worker failed: %s", exc)
                    last_error_log = now
                self._worker_stop.wait(0.05)

    def _send_interpolated_target(self, period_s: float) -> None:
        with self._target_lock:
            target = list(self._target_positions) if self._target_positions is not None else None
        if target is None:
            return

        current = self._last_commanded_positions or self._latest_positions
        if current is None:
            current = target
        next_positions, reached = _step_toward(current, target, _MAX_JOINT_VELOCITY_RAD_S * period_s)

        with self._lock:
            if not self._is_connected():
                return
            for hand_side, hand_positions in self._split_positions(next_positions).items():
                self._hands[hand_side].set_joint_positions([math.degrees(v) for v in hand_positions])
            self._last_commanded_positions = list(next_positions)
            self._latest_positions = list(next_positions)

        if reached:
            with self._target_lock:
                if _positions_close(self._target_positions, target):
                    self._target_positions = None

    def _read_joint_positions_locked(self) -> List[float]:
        positions: List[float] = []
        for hand_side in self._active_hands():
            positions.extend(self._read_hand_positions(self._hands[hand_side]))
        self._latest_positions = positions
        return list(positions)

    def _read_hand_positions(self, hand: Any) -> List[float]:
        positions_deg = None
        for _ in range(5):
            positions_deg = hand.get_joint_positions_compact()
            if positions_deg is not None:
                break
            time.sleep(0.05)
        if positions_deg is None:
            raise RuntimeError("Aero Hand returned no joint positions")
        if len(positions_deg) != len(JOINT_NAMES):
            raise ValueError(f"Aero Hand expects {len(JOINT_NAMES)} joints, got {len(positions_deg)}")
        return [math.radians(float(v)) for v in positions_deg]

    def _active_hands(self) -> tuple[str, ...]:
        if self.mode == "dual":
            return DUAL_HAND_ORDER
        return ("hand",)

    def _port_for(self, hand_side: str) -> str | None:
        if hand_side == "hand":
            return self.port or self.right_port or self.left_port
        return self.left_port if hand_side == "left" else self.right_port

    def _expected_dof(self) -> int:
        return len(JOINT_NAMES) * len(self._active_hands())

    def _split_positions(self, positions: List[float]) -> Dict[str, List[float]]:
        active = self._active_hands()
        return {
            hand_side: positions[index * len(JOINT_NAMES) : (index + 1) * len(JOINT_NAMES)]
            for index, hand_side in enumerate(active)
        }

    def _joint_specs(self) -> list[tuple[str, str, tuple[float, float]]]:
        return [
            (hand_side, name, limits)
            for hand_side in self._active_hands()
            for name, limits in zip(JOINT_NAMES, JOINT_LIMITS_DEG)
        ]

    def _joint_name(self, hand_side: str, name: str) -> str:
        if self.mode == "dual":
            return f"{hand_side}_{name}"
        return name


def _joint_position_list(value: Any) -> List[float]:
    if not isinstance(value, list):
        raise TypeError("joint_positions must be a list")
    return [float(v) for v in value]


def _validate_mode(mode: str) -> str:
    mode = str(mode or "single").strip().lower()
    if mode not in ("single", "dual"):
        raise ValueError("Aero Hand mode must be 'single' or 'dual'")
    return mode


def _step_toward(current: List[float], target: List[float], max_step: float) -> tuple[List[float], bool]:
    step = max(0.0, float(max_step))
    result: List[float] = []
    reached = True
    for current_value, target_value in zip(current, target):
        delta = float(target_value) - float(current_value)
        smooth_step = min(step, abs(delta) * _TARGET_SMOOTHING)
        if abs(delta) <= max(smooth_step, _TARGET_TOLERANCE_RAD):
            result.append(float(target_value))
            continue
        reached = False
        result.append(float(current_value) + math.copysign(smooth_step, delta))
    return result, reached


def _positions_close(left: List[float] | None, right: List[float], tolerance: float = _TARGET_TOLERANCE_RAD) -> bool:
    return left is not None and len(left) == len(right) and all(abs(a - b) <= tolerance for a, b in zip(left, right))
