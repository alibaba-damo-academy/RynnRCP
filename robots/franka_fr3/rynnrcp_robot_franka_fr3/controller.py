"""RynnRCP lifecycle wrapper for the native libfranka driver."""

from __future__ import annotations

import importlib
import logging
import math
import threading
import time
from collections.abc import Mapping, Sequence
from typing import Any

from rynnrcp.robot.base_controller import BaseRobotController


ARM_DOF = 7
DEFAULT_LOWER_LIMITS = (-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159)
DEFAULT_UPPER_LIMITS = (2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159)
DEFAULT_HOME = (0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785)
DEFAULT_TORQUE_THRESHOLDS = (20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0)
DEFAULT_FORCE_THRESHOLDS = (20.0, 20.0, 20.0, 25.0, 25.0, 25.0)


class FrankaController(BaseRobotController):
    """Expose FR3 joint, end-effector pose, and Franka Hand controls."""

    n_dof = ARM_DOF + 1

    def __init__(
        self,
        robot_ip: str,
        robot_id: str = "franka",
        *,
        with_gripper: bool = True,
        realtime_enforce: bool = True,
        max_joint_velocity_rad_s: float = 0.25,
        max_joint_acceleration_rad_s2: float = 0.5,
        max_joint_jerk_rad_s3: float = 2.5,
        max_cartesian_velocity_m_s: float = 0.05,
        max_cartesian_rotation_rad_s: float = 0.25,
        target_timeout_s: float = 0.5,
        gripper_max_width_m: float = 0.08,
        gripper_speed_m_s: float = 0.05,
        gripper_command_deadband: float = 0.01,
        gripper_grasp_threshold: float = 0.1,
        gripper_grasp_force_n: float = 20.0,
        home_joint_positions: Sequence[float] = DEFAULT_HOME,
        joint_lower_limits: Sequence[float] = DEFAULT_LOWER_LIMITS,
        joint_upper_limits: Sequence[float] = DEFAULT_UPPER_LIMITS,
        collision_torque_thresholds: Sequence[float] = DEFAULT_TORQUE_THRESHOLDS,
        collision_force_thresholds: Sequence[float] = DEFAULT_FORCE_THRESHOLDS,
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__(logger=logger)
        self.robot_ip = str(robot_ip)
        self.robot_id = str(robot_id)
        self.with_gripper = bool(with_gripper)
        self.realtime_enforce = bool(realtime_enforce)
        self.max_joint_velocity_rad_s = _positive(
            max_joint_velocity_rad_s, "max_joint_velocity_rad_s"
        )
        self.max_joint_acceleration_rad_s2 = _positive(
            max_joint_acceleration_rad_s2, "max_joint_acceleration_rad_s2"
        )
        self.max_joint_jerk_rad_s3 = _positive(
            max_joint_jerk_rad_s3, "max_joint_jerk_rad_s3"
        )
        self.max_cartesian_velocity_m_s = _positive(
            max_cartesian_velocity_m_s, "max_cartesian_velocity_m_s"
        )
        self.max_cartesian_rotation_rad_s = _positive(
            max_cartesian_rotation_rad_s, "max_cartesian_rotation_rad_s"
        )
        self.target_timeout_s = _positive(target_timeout_s, "target_timeout_s")
        self.gripper_max_width_m = _positive(gripper_max_width_m, "gripper_max_width_m")
        self.gripper_speed_m_s = _positive(gripper_speed_m_s, "gripper_speed_m_s")
        self.gripper_command_deadband = _bounded(
            gripper_command_deadband, "gripper_command_deadband", 0.0, 1.0
        )
        self.gripper_grasp_threshold = _bounded(
            gripper_grasp_threshold, "gripper_grasp_threshold", 0.0, 1.0
        )
        self.gripper_grasp_force_n = _positive(
            gripper_grasp_force_n, "gripper_grasp_force_n"
        )
        self.home_joint_positions = _vector(
            home_joint_positions, ARM_DOF, "home_joint_positions"
        )
        self.joint_lower_limits = _vector(
            joint_lower_limits, ARM_DOF, "joint_lower_limits"
        )
        self.joint_upper_limits = _vector(
            joint_upper_limits, ARM_DOF, "joint_upper_limits"
        )
        if any(
            low >= high
            for low, high in zip(self.joint_lower_limits, self.joint_upper_limits)
        ):
            raise ValueError(
                "each Franka lower joint limit must be below its upper limit"
            )
        self._validate_joint_target(self.home_joint_positions)
        self.collision_torque_thresholds = _vector(
            collision_torque_thresholds, ARM_DOF, "collision_torque_thresholds"
        )
        self.collision_force_thresholds = _vector(
            collision_force_thresholds, 6, "collision_force_thresholds"
        )

        self._native: Any = None
        self._effective_gripper_width_m = self.gripper_max_width_m
        self._gripper_condition = threading.Condition()
        self._gripper_thread: threading.Thread | None = None
        self._gripper_target: float | None = None
        self._gripper_requested: float | None = None
        self._gripper_state: dict[str, Any] | None = None
        self._gripper_error: str | None = None
        self._stopping = False

    def start(self) -> None:
        """Connect and start the single native state/control loop."""
        if self._native is not None:
            return
        module = importlib.import_module("rynnrcp_robot_franka_fr3._franka_native")
        native = module.FrankaHardware(
            self.robot_ip,
            self.with_gripper,
            self.realtime_enforce,
            self.max_joint_velocity_rad_s,
            self.max_joint_acceleration_rad_s2,
            self.max_joint_jerk_rad_s3,
            self.max_cartesian_velocity_m_s,
            self.max_cartesian_rotation_rad_s,
            self.target_timeout_s,
            self.joint_lower_limits,
            self.joint_upper_limits,
            self.collision_torque_thresholds,
            self.collision_force_thresholds,
        )
        try:
            native.connect()
            if self.with_gripper:
                gripper_state = native.get_gripper_state()
                measured_max = float(gripper_state["max_width"])
                if math.isfinite(measured_max) and measured_max > 0.0:
                    self._effective_gripper_width_m = min(
                        self.gripper_max_width_m, measured_max
                    )
                self._gripper_requested = _clamp(
                    float(gripper_state["width"]) / self._effective_gripper_width_m,
                    0.0,
                    1.0,
                )
                self._update_gripper_state(gripper_state)
            native.start_control()
        except Exception:
            native.disconnect()
            raise
        self._native = native
        self._stopping = False
        self._gripper_error = None
        if self.with_gripper:
            self._gripper_thread = threading.Thread(
                target=self._gripper_worker,
                name=f"franka-gripper-{self.robot_id}",
                daemon=True,
            )
            self._gripper_thread.start()

    def shutdown(self) -> None:
        native = self._native
        with self._gripper_condition:
            self._stopping = True
            self._gripper_target = None
            self._gripper_condition.notify_all()
        thread, self._gripper_thread = self._gripper_thread, None
        if thread is not None:
            thread.join()
        self._native = None
        if native is not None:
            native.disconnect()

    def set_joint_positions(self, value: Mapping[str, Any]) -> dict[str, list[float]]:
        """Apply a seven-joint target."""
        if not isinstance(value, Mapping):
            raise ValueError("joint_position action value must be an object")
        positions = _vector(value.get("joint_positions"), ARM_DOF, "joint_positions")
        self._validate_joint_target(positions)
        native = self._ensure_control()
        native.set_joint_target(positions)
        return {"joint_positions": positions}

    def set_control_positions(
        self, value: Mapping[str, Any]
    ) -> dict[str, list[float]]:
        """Apply seven arm joints followed by one normalized gripper position."""
        if not isinstance(value, Mapping):
            raise ValueError("joint_position action value must be an object")
        positions = _vector(
            value.get("joint_positions"),
            ARM_DOF + 1,
            "joint_positions",
        )
        arm_positions = positions[:ARM_DOF]
        gripper_position = _bounded(
            positions[ARM_DOF],
            "gripper position",
            0.0,
            1.0,
        )
        self._validate_joint_target(arm_positions)
        native = self._ensure_control()
        native.set_joint_target(arm_positions)
        self._queue_gripper(gripper_position)
        return {"joint_positions": positions}

    def home(self, value: Mapping[str, Any] | None = None) -> dict[str, list[float]]:
        """Send the configured seven-joint home target."""
        _require_empty_action(value, "home")
        return self.set_joint_positions(
            {"joint_positions": list(self.home_joint_positions)}
        )

    def get_joint_positions(self) -> dict[str, list[float]]:
        """Return the seven arm joints."""
        state = self._require_native().get_state()
        return {
            "joint_positions": [float(item) for item in state["q"]],
            "joint_velocities": [float(item) for item in state["dq"]],
        }

    def get_control_positions(self) -> dict[str, list[float]]:
        """Return seven measured arm joints followed by normalized gripper opening."""
        joints = self.get_joint_positions()
        gripper = self.get_gripper_state()
        return {
            "joint_positions": [
                *joints["joint_positions"],
                float(gripper["position"]),
            ],
            "joint_velocities": [
                *joints["joint_velocities"],
                0.0,
            ],
        }

    def get_ee_pose(self) -> dict[str, list[float]]:
        """Return the measured end-effector pose in the Franka base frame."""
        transform = _vector(
            self._require_native().get_state()["O_T_EE"], 16, "O_T_EE"
        )
        return {
            "position": [transform[12], transform[13], transform[14]],
            "orientation_quat_xyzw": _rotation_matrix_to_quaternion(transform),
        }

    def set_ee_pose(self, value: Mapping[str, Any]) -> dict[str, list[float]]:
        """Track an absolute end-effector pose in the Franka base frame."""
        if not isinstance(value, Mapping):
            raise ValueError("ee_pose action value must be an object")
        position = _vector(value.get("position"), 3, "position")
        quaternion = _normalized_quaternion(value.get("orientation_quat_xyzw"))
        self._ensure_control().set_ee_target(position, quaternion)
        return {
            "position": position,
            "orientation_quat_xyzw": quaternion,
        }

    def set_gripper(self, value: Mapping[str, Any]) -> dict[str, float]:
        """Apply a normalized Franka Hand position target."""
        if not isinstance(value, Mapping) or "position" not in value:
            raise ValueError("gripper action value requires position")
        position = _bounded(value["position"], "position", 0.0, 1.0)
        self._queue_gripper(position)
        return {"position": position}

    def get_gripper_state(self) -> dict[str, Any]:
        """Return the latest Franka Hand state without blocking observation."""
        self._require_native()
        with self._gripper_condition:
            if self._gripper_state is None:
                raise RuntimeError("Franka Hand has no cached state")
            return dict(self._gripper_state)

    def automatic_error_recovery(
        self, value: Mapping[str, Any] | None = None
    ) -> dict[str, bool]:
        """Recover from a Franka error after control has stopped."""
        _require_empty_action(value, "automatic_error_recovery")
        native = self._require_native()
        native.stop_control()
        native.automatic_error_recovery()
        native.start_control()
        return {"recovered": True}

    def get_health(self) -> dict[str, list[dict[str, Any]]]:
        errors: list[dict[str, Any]] = []
        warnings: list[dict[str, Any]] = []
        now = time.time()
        if self._native is None:
            warnings.append(
                self._health_item(
                    "franka.not_connected", "Franka is not connected", now
                )
            )
            return {"errors": errors, "warnings": warnings}

        status = self._native.status()
        if not status["control_enabled"]:
            warnings.append(
                self._health_item(
                    "franka.control_idle",
                    "Franka is connected and waiting for an arm target",
                    now,
                )
            )
        if self.with_gripper and not status["gripper_connected"]:
            warnings.append(
                self._health_item(
                    "franka.gripper_not_connected", "Franka Hand is not connected", now
                )
            )
        if not status["realtime_enforce"]:
            warnings.append(
                self._health_item(
                    "franka.realtime_not_enforced",
                    "Real-time scheduling enforcement is disabled",
                    now,
                )
            )
        if status["watchdog_triggered"]:
            warnings.append(
                self._health_item(
                    "franka.target_timeout",
                    "Target stream timed out; the arm is holding its measured pose",
                    now,
                )
            )
        if status["control_error"]:
            errors.append(
                self._health_item(
                    "franka.control_error", str(status["control_error"]), now
                )
            )
        if self._gripper_error:
            errors.append(
                self._health_item("franka.gripper_error", self._gripper_error, now)
            )
        return {"errors": errors, "warnings": warnings}

    def _queue_gripper(self, normalized_position: float) -> None:
        if not self.with_gripper:
            return
        with self._gripper_condition:
            if (
                self._gripper_requested is not None
                and abs(normalized_position - self._gripper_requested)
                <= self.gripper_command_deadband
            ):
                return
            self._gripper_requested = normalized_position
            self._gripper_target = normalized_position
            self._gripper_condition.notify()

    def _gripper_worker(self) -> None:
        while True:
            with self._gripper_condition:
                self._gripper_condition.wait_for(
                    lambda: self._stopping or self._gripper_target is not None,
                    timeout=0.05,
                )
                if self._stopping:
                    return
                normalized_position = self._gripper_target
                self._gripper_target = None
            native = self._require_native()
            if normalized_position is not None:
                try:
                    width = normalized_position * self._effective_gripper_width_m
                    if normalized_position <= self.gripper_grasp_threshold:
                        native.gripper_grasp(
                            width,
                            self.gripper_speed_m_s,
                            self.gripper_grasp_force_n,
                            0.01,
                            0.07,
                        )
                    else:
                        native.gripper_move(width, self.gripper_speed_m_s)
                    self._gripper_error = None
                except Exception as exc:
                    self._gripper_error = str(exc)
                    self.logger.exception("Franka Hand command failed")
            try:
                self._update_gripper_state(native.get_gripper_state())
                if normalized_position is None:
                    self._gripper_error = None
            except Exception as exc:
                self._gripper_error = str(exc)

    def _update_gripper_state(self, state: Mapping[str, Any]) -> None:
        normalized = {
            "position": _clamp(
                float(state["width"]) / self._effective_gripper_width_m,
                0.0,
                1.0,
            ),
            "width": float(state["width"]),
            "max_width": float(state["max_width"]),
            "is_grasped": bool(state["is_grasped"]),
            "temperature": int(state["temperature"]),
        }
        with self._gripper_condition:
            self._gripper_state = normalized

    def _validate_joint_target(self, positions: Sequence[float]) -> None:
        for index, (position, low, high) in enumerate(
            zip(positions, self.joint_lower_limits, self.joint_upper_limits),
            start=1,
        ):
            if not low <= position <= high:
                raise ValueError(
                    f"Franka joint {index} target {position:.6f} is outside "
                    f"[{low:.6f}, {high:.6f}] rad"
                )

    def _ensure_control(self) -> Any:
        native = self._require_native()
        native_status = native.status()
        if not native_status["control_enabled"]:
            previous_error = str(native_status.get("control_error", "")).strip()
            if previous_error:
                raise RuntimeError(
                    "Franka previous control session failed: "
                    f"{previous_error}. Inspect Franka Desk, recover the robot, "
                    "then reconnect before sending another target."
                )
            raise RuntimeError("Franka state/control loop is not running")
        return native

    def _require_native(self) -> Any:
        if self._native is None:
            raise RuntimeError("Franka is not connected")
        return self._native

    def _health_item(self, code: str, message: str, timestamp: float) -> dict[str, Any]:
        return {
            "code": code,
            "message": message,
            "source": "robot",
            "timestamp": timestamp,
            "details": {"robot_id": self.robot_id, "robot_ip": self.robot_ip},
        }


def _vector(value: Any, size: int, name: str) -> list[float]:
    if value is None or isinstance(value, (str, bytes, Mapping)):
        raise ValueError(f"{name} must contain {size} numeric values")
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain {size} numeric values") from exc
    if len(result) != size or not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain {size} finite numeric values")
    return result


def _positive(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return result


def _bounded(value: Any, name: str, low: float, high: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a number") from exc
    if not math.isfinite(result) or not low <= result <= high:
        raise ValueError(f"{name} must be within [{low}, {high}]")
    return result


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _require_empty_action(value: Mapping[str, Any] | None, action: str) -> None:
    if value is not None and (not isinstance(value, Mapping) or value):
        raise ValueError(f"{action} action value must be an empty object")


def _normalized_quaternion(value: Any) -> list[float]:
    quaternion = _vector(value, 4, "orientation_quat_xyzw")
    norm = math.sqrt(sum(item * item for item in quaternion))
    if norm < 1e-12:
        raise ValueError("orientation_quat_xyzw must have a non-zero norm")
    return [item / norm for item in quaternion]


def _rotation_matrix_to_quaternion(transform: Sequence[float]) -> list[float]:
    r00, r01, r02 = transform[0], transform[4], transform[8]
    r10, r11, r12 = transform[1], transform[5], transform[9]
    r20, r21, r22 = transform[2], transform[6], transform[10]
    trace = r00 + r11 + r22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = [
            (r21 - r12) / scale,
            (r02 - r20) / scale,
            (r10 - r01) / scale,
            0.25 * scale,
        ]
    elif r00 > r11 and r00 > r22:
        scale = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        quaternion = [
            0.25 * scale,
            (r01 + r10) / scale,
            (r02 + r20) / scale,
            (r21 - r12) / scale,
        ]
    elif r11 > r22:
        scale = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        quaternion = [
            (r01 + r10) / scale,
            0.25 * scale,
            (r12 + r21) / scale,
            (r02 - r20) / scale,
        ]
    else:
        scale = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        quaternion = [
            (r02 + r20) / scale,
            (r12 + r21) / scale,
            0.25 * scale,
            (r10 - r01) / scale,
        ]
    quaternion = _normalized_quaternion(quaternion)
    if quaternion[3] < 0.0:
        quaternion = [-item for item in quaternion]
    return quaternion
