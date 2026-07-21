"""Booster T1 low-level controller for RynnRCP."""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Mapping

from rynnrcp.robot.base_controller import BaseRobotController

JOINT_NAMES = [
    "head_yaw",
    "head_pitch",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_elbow_pitch",
    "left_elbow_yaw",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_elbow_pitch",
    "right_elbow_yaw",
    "waist",
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "left_knee_pitch",
    "left_crank_up",
    "left_crank_down",
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "right_knee_pitch",
    "right_crank_up",
    "right_crank_down",
]

JOINT_ENUM_NAMES = [
    "kHeadYaw",
    "kHeadPitch",
    "kLeftShoulderPitch",
    "kLeftShoulderRoll",
    "kLeftElbowPitch",
    "kLeftElbowYaw",
    "kRightShoulderPitch",
    "kRightShoulderRoll",
    "kRightElbowPitch",
    "kRightElbowYaw",
    "kWaist",
    "kLeftHipPitch",
    "kLeftHipRoll",
    "kLeftHipYaw",
    "kLeftKneePitch",
    "kCrankUpLeft",
    "kCrankDownLeft",
    "kRightHipPitch",
    "kRightHipRoll",
    "kRightHipYaw",
    "kRightKneePitch",
    "kCrankUpRight",
    "kCrankDownRight",
]

DEFAULT_KP = [
    5.0,
    5.0,
    40.0,
    50.0,
    20.0,
    10.0,
    40.0,
    50.0,
    20.0,
    10.0,
    100.0,
    350.0,
    350.0,
    180.0,
    350.0,
    550.0,
    550.0,
    350.0,
    350.0,
    180.0,
    350.0,
    550.0,
    550.0,
]

DEFAULT_KD = [
    0.1,
    0.1,
    0.5,
    1.5,
    0.2,
    0.2,
    0.5,
    1.5,
    0.2,
    0.2,
    5.0,
    7.5,
    7.5,
    3.0,
    5.5,
    1.5,
    1.5,
    7.5,
    7.5,
    3.0,
    5.5,
    1.5,
    1.5,
]

PREPARE_KP = [
    5.0,
    5.0,
    40.0,
    50.0,
    20.0,
    10.0,
    40.0,
    50.0,
    20.0,
    10.0,
    100.0,
    350.0,
    350.0,
    180.0,
    350.0,
    450.0,
    450.0,
    350.0,
    350.0,
    180.0,
    350.0,
    450.0,
    450.0,
]

PREPARE_KD = [
    0.1,
    0.1,
    0.5,
    1.5,
    0.2,
    0.2,
    0.5,
    1.5,
    0.2,
    0.2,
    5.0,
    7.5,
    7.5,
    3.0,
    5.5,
    0.5,
    0.5,
    7.5,
    7.5,
    3.0,
    5.5,
    0.5,
    0.5,
]

PREPARE_JOINT_POS = [
    0.0,
    0.0,
    0.25,
    -1.4,
    0.0,
    -0.5,
    0.25,
    1.4,
    0.0,
    0.5,
    0.0,
    -0.1,
    0.0,
    0.0,
    0.2,
    -0.1,
    0.0,
    -0.1,
    0.0,
    0.0,
    0.2,
    -0.1,
    0.0,
]

TORQUE_LIMIT = [
    7.0,
    7.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    30.0,
    60.0,
    25.0,
    30.0,
    60.0,
    24.0,
    15.0,
    60.0,
    25.0,
    30.0,
    60.0,
    24.0,
    15.0,
]

PARALLEL_MECH_INDEXES = {15, 16, 21, 22}
LOW_CMD_DT = 0.02
TARGET_BLEND = 1.0 - 0.8**10

# The robot firmware publishes and accepts the 29-motor layout even when this
# policy uses the compact 23-joint T1 observation/action layout.
POLICY_TO_MOTOR_INDEX = [
    0, 1,
    2, 3, 4, 5,
    9, 10, 11, 12,
    16,
    17, 18, 19, 20, 21, 22,
    23, 24, 25, 26, 27, 28,
]
EXTRA_MOTOR_GAINS = {
    6: (40.0, 0.65),
    7: (100.0, 1.5),
    8: (40.0, 0.65),
    13: (40.0, 0.65),
    14: (100.0, 1.5),
    15: (40.0, 0.65),
}


class BoosterT1LowController(BaseRobotController):
    def __init__(
        self,
        robot_id: str = "booster_t1_low",
        net: str = "",
        domain_id: int = 0,
        default_kp: float | list[float] | None = None,
        default_kd: float | list[float] | None = None,
        damping_kd: float = 0.1,
    ) -> None:
        super().__init__()
        self.robot_id = str(robot_id)
        self.net = str(net)
        self.domain_id = int(domain_id)
        self.damping_kd = float(damping_kd)
        self._kp = _gains(default_kp, DEFAULT_KP, "default_kp")
        self._kd = _gains(default_kd, DEFAULT_KD, "default_kd")
        self._sdk: Any = None
        self._publisher: Any = None
        self._subscriber: Any = None
        self._loco: Any = None
        self._low_command: Any = None
        self._motor_commands: list[Any] | None = None
        self._latest_joint_positions: list[float] | None = None
        self._latest_joint_velocities: list[float] | None = None
        self._latest_motor_positions: list[float] | None = None
        self._latest_imu: dict[str, list[float]] | None = None
        self._state_error: str | None = None
        self._lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._target_lock = threading.Lock()
        self._desired_joint_positions: list[float] | None = None
        self._filtered_joint_positions: list[float] | None = None
        self._prepare_control = True
        self._last_target_update = 0.0
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread: threading.Thread | None = None
        self._heartbeat_error: Exception | None = None

    def start(self) -> None:
        if self._publisher is not None:
            return
        try:
            import booster_robotics_sdk_python as sdk  # type: ignore
        except ImportError as exc:
            raise RuntimeError("booster_robotics_sdk_python is not installed on the robot") from exc

        self._sdk = sdk
        sdk.ChannelFactory.Instance().Init(self.domain_id, self.net)

        self._publisher = sdk.B1LowCmdPublisher()
        self._publisher.InitChannel()

        self._low_command = sdk.LowCmd()
        self._low_command.cmd_type = sdk.LowCmdType.SERIAL
        joint_count = int(sdk.kJointCnt7DofArm)
        self._motor_commands = [sdk.MotorCmd() for _ in range(joint_count)]

        self._subscriber = sdk.B1LowStateSubscriber(self._on_low_state)
        self._subscriber.InitChannel()

        self._loco = sdk.B1LocoClient()
        self._loco.Init()

    def shutdown(self, prepare: bool = True) -> None:
        if prepare and self._loco is not None:
            try:
                self._loco.ChangeMode(self._sdk.RobotMode.kPrepare)
                time.sleep(0.2)
            except Exception:
                pass
        self._stop_heartbeat()
        if self._subscriber is not None:
            try:
                self._subscriber.CloseChannel()
            except Exception:
                pass
        if self._publisher is not None:
            try:
                self._publisher.CloseChannel()
            except Exception:
                pass
        self._publisher = None
        self._subscriber = None
        self._loco = None
        self._low_command = None
        self._motor_commands = None
        self._heartbeat_thread = None

    def get_health(self) -> dict[str, Any]:
        warnings = []
        if self._publisher is None:
            warnings.append(_warning("booster_t1_low.not_started", "Booster T1 low-level controller is not started"))
        if self._latest_joint_positions is None:
            warnings.append(_warning("booster_t1_low.no_state", "No LowState has been received yet"))
        if self._latest_imu is None:
            warnings.append(_warning("booster_t1_low.no_imu", "No LowState IMU has been received yet"))
        if self._publisher is not None:
            if self._heartbeat_thread is None or not self._heartbeat_thread.is_alive():
                warnings.append(_warning(
                    "booster_t1_low.enter_low_required",
                    "Run action.robot.enter_low before starting Booster T1 policy inference",
                ))
            else:
                try:
                    status = self._loco.GetStatus()
                    if status.current_mode != self._sdk.RobotMode.kCustom:
                        warnings.append(_warning(
                            "booster_t1_low.enter_low_required",
                            "Robot is not in Custom low-level mode; run action.robot.enter_low before policy inference",
                        ))
                except Exception as exc:
                    warnings.append(_warning("booster_t1_low.status_failed", f"Could not read loco status: {exc}"))
        return {"errors": [], "warnings": warnings}

    def prepare_high(self, duration_s: float = 5.0) -> dict[str, Any]:
        self._ensure_started()
        self._loco.ChangeMode(self._sdk.RobotMode.kPrepare)
        time.sleep(max(0.0, float(duration_s)))
        return {"command": "prepare_high", **self.get_loco_status()}

    def enter_low(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        self._ensure_started()
        prepare_target = self.get_joint_positions()["joint_positions"]
        if value and "prepare_joint_positions" in value:
            prepare_target = _vector(value["prepare_joint_positions"], len(JOINT_NAMES), "prepare_joint_positions")
        self.set_prepare_positions(prepare_target)
        self._write_position_cmd(prepare_target, PREPARE_KP, PREPARE_KD, use_parallel_conversion=False)
        self._start_heartbeat()
        time.sleep(0.3)
        self._loco.ChangeMode(self._sdk.RobotMode.kCustom)
        deadline = time.perf_counter() + 2.0
        while time.perf_counter() < deadline:
            self._raise_heartbeat_error()
            status = self._loco.GetStatus()
            if status.current_mode == self._sdk.RobotMode.kCustom:
                if value and "initial_joint_positions" in value:
                    self.set_joint_positions({"joint_positions": value["initial_joint_positions"]})
                return {
                    "command": "enter_low",
                    "current_mode": str(status.current_mode),
                    "current_body_control": str(status.current_body_control),
                    "current_actions": [str(x) for x in status.current_actions],
                }
            time.sleep(0.05)
        raise RuntimeError(f"ChangeMode(kCustom) did not take effect: {self.get_loco_status()}")

    def get_loco_status(self) -> dict[str, Any]:
        self._ensure_started()
        status = self._loco.GetStatus()
        return {
            "current_mode": str(status.current_mode),
            "current_body_control": str(status.current_body_control),
            "current_actions": [str(x) for x in status.current_actions],
        }

    def get_joint_positions(self) -> dict[str, Any]:
        self._ensure_started()
        with self._lock:
            state_error = self._state_error
            q = self._latest_joint_positions
            dq = self._latest_joint_velocities
        if state_error is not None:
            raise RuntimeError(state_error)
        if q is None or dq is None:
            raise RuntimeError("No LowState received yet")
        return {"joint_positions": list(q), "joint_velocities": list(dq)}

    def get_motor_count(self) -> int:
        with self._lock:
            motors = self._latest_motor_positions
        return 0 if motors is None else len(motors)

    def get_imu(self) -> dict[str, Any]:
        self._ensure_started()
        with self._lock:
            state_error = self._state_error
            imu = self._latest_imu
        if state_error is not None:
            raise RuntimeError(state_error)
        if imu is None:
            raise RuntimeError("No LowState received yet")
        return {key: list(value) for key, value in imu.items()}

    def set_joint_positions(self, value: Mapping[str, Any]) -> dict[str, Any]:
        self._ensure_started()
        self._raise_heartbeat_error()
        if self._heartbeat_thread is None or not self._heartbeat_thread.is_alive():
            raise RuntimeError("LowCmd heartbeat is not running; run action.robot.enter_low before policy inference")
        target = _vector(value.get("joint_positions"), len(JOINT_NAMES), "joint_positions")
        with self._target_lock:
            if self._filtered_joint_positions is None:
                self._filtered_joint_positions = list(target)
            self._desired_joint_positions = list(target)
            self._prepare_control = False
            self._last_target_update = time.perf_counter()
        return {"joints": len(target)}

    def set_prepare_positions(self, joint_positions: list[float]) -> None:
        self._raise_heartbeat_error()
        target = _vector(joint_positions, len(JOINT_NAMES), "joint_positions")
        with self._target_lock:
            self._desired_joint_positions = list(target)
            self._filtered_joint_positions = list(target)
            self._prepare_control = True
            self._last_target_update = time.perf_counter()

    def _start_heartbeat(self) -> None:
        if self._heartbeat_thread is not None and self._heartbeat_thread.is_alive():
            return
        self._heartbeat_error = None
        self._heartbeat_stop.clear()
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, name="booster-t1-lowcmd", daemon=True)
        self._heartbeat_thread.start()

    def _stop_heartbeat(self) -> None:
        self._heartbeat_stop.set()
        if self._heartbeat_thread is not None and self._heartbeat_thread is not threading.current_thread():
            self._heartbeat_thread.join(timeout=0.5)
        self._heartbeat_thread = None

    def _heartbeat_loop(self) -> None:
        next_time = time.perf_counter()
        while not self._heartbeat_stop.is_set():
            try:
                with self._target_lock:
                    desired = None if self._desired_joint_positions is None else list(self._desired_joint_positions)
                    filtered = None if self._filtered_joint_positions is None else list(self._filtered_joint_positions)
                    prepare_control = self._prepare_control
                    last_update = self._last_target_update
                if desired is not None:
                    if not prepare_control and time.perf_counter() - last_update > 0.5:
                        self._return_to_prepare()
                        continue
                    if prepare_control:
                        command = desired
                    else:
                        command = [
                            (1.0 - TARGET_BLEND) * old + TARGET_BLEND * goal
                            for old, goal in zip(filtered or desired, desired)
                        ]
                        with self._target_lock:
                            self._filtered_joint_positions = list(command)
                    self._write_position_cmd(
                        command,
                        PREPARE_KP if prepare_control else self._kp,
                        PREPARE_KD if prepare_control else self._kd,
                        use_parallel_conversion=not prepare_control,
                    )
            except Exception as exc:
                self._heartbeat_error = exc
                self._heartbeat_stop.set()
                return
            next_time += LOW_CMD_DT
            time.sleep(max(0.0, next_time - time.perf_counter()))

    def _return_to_prepare(self) -> None:
        current = self.get_joint_positions()["joint_positions"]
        with self._target_lock:
            self._desired_joint_positions = list(current)
            self._filtered_joint_positions = list(current)
            self._prepare_control = True
        self._loco.ChangeMode(self._sdk.RobotMode.kPrepare)
        self._heartbeat_stop.set()

    def _raise_heartbeat_error(self) -> None:
        if self._heartbeat_error is not None:
            raise RuntimeError(f"LowCmd heartbeat failed: {self._heartbeat_error!r}") from self._heartbeat_error

    def _write_position_cmd(
        self,
        target: list[float],
        kp: list[float],
        kd: list[float],
        *,
        use_parallel_conversion: bool,
    ) -> None:
        with self._lock:
            current_motors = None if self._latest_motor_positions is None else list(self._latest_motor_positions)
        if current_motors is None:
            raise RuntimeError("No LowState received yet")

        with self._command_lock:
            motors = self._motor_commands
            if motors is None:
                raise RuntimeError("LowCmd buffer is not initialized")
            for motor_index, motor in enumerate(motors):
                motor.q = current_motors[motor_index]
                motor.dq = 0.0
                motor.kp, motor.kd = EXTRA_MOTOR_GAINS.get(motor_index, (0.0, 0.0))
                motor.tau = 0.0
                motor.weight = 0.0
            for policy_index, motor_index in enumerate(POLICY_TO_MOTOR_INDEX):
                motor = motors[motor_index]
                motor.q = target[policy_index]
                motor.kp = kp[policy_index]
                motor.kd = kd[policy_index]
                if use_parallel_conversion and policy_index in PARALLEL_MECH_INDEXES:
                    motor.q = current_motors[motor_index]
                    motor.tau = _clip(
                        (target[policy_index] - current_motors[motor_index]) * kp[policy_index],
                        -TORQUE_LIMIT[policy_index],
                        TORQUE_LIMIT[policy_index],
                    )
                    motor.kp = 0.0
            self._write_buffered_command()

    def damping(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "damping")
        self._ensure_started()
        self._stop_heartbeat()
        with self._command_lock:
            motors = self._motor_commands
            if motors is None:
                raise RuntimeError("LowCmd buffer is not initialized")
            for motor in motors:
                motor.q = 0.0
                motor.dq = 0.0
                motor.kp = 0.0
                motor.kd = self.damping_kd
                motor.tau = 0.0
                motor.weight = 0.0
            self._write_buffered_command()
        self._loco.ChangeMode(self._sdk.RobotMode.kDamping)
        return {"mode": "damping", "kd": self.damping_kd}

    def _write_buffered_command(self) -> None:
        if self._low_command is None or self._motor_commands is None:
            raise RuntimeError("LowCmd buffer is not initialized")
        self._low_command.motor_cmd = self._motor_commands
        self._write(self._low_command)

    def _write(self, cmd: Any) -> None:
        if not self._publisher.Write(cmd):
            raise RuntimeError("B1LowCmdPublisher.Write returned False")

    def _on_low_state(self, msg: Any) -> None:
        try:
            motors = msg.motor_state_serial
            if len(motors) < 29:
                raise RuntimeError(f"LowState motor_state_serial has {len(motors)} motors, expected 29")
            imu = msg.imu_state
            motor_positions = [float(motors[i].q) for i in range(29)]
            joint_positions = [float(motors[i].q) for i in POLICY_TO_MOTOR_INDEX]
            joint_velocities = [float(motors[i].dq) for i in POLICY_TO_MOTOR_INDEX]
            rpy = _required_vector_attr(imu, "rpy", 3)
            imu_state = {
                "accel": _required_vector_attr(imu, "acc", 3),
                "gyro": _required_vector_attr(imu, "gyro", 3),
                "rpy": rpy,
                "orientation_quat_wxyz": _quat_wxyz_from_rpy(rpy),
            }
        except Exception as exc:
            with self._lock:
                self._state_error = f"Invalid LowState from Booster SDK: {exc}"
            return
        with self._lock:
            self._state_error = None
            self._latest_joint_positions = joint_positions
            self._latest_joint_velocities = joint_velocities
            self._latest_motor_positions = motor_positions
            self._latest_imu = imu_state

    def _ensure_started(self) -> None:
        if self._publisher is None or self._sdk is None:
            raise RuntimeError("Booster T1 low-level controller is not started")


def _vector(value: Any, length: int, name: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must be a list of {length} floats")
    return [float(x) for x in value]


def _gains(value: Any, default: list[float], name: str) -> list[float]:
    if value is None:
        return list(default)
    if isinstance(value, (int, float)):
        return [float(value)] * len(default)
    return _vector(value, len(default), name)


def _required_vector_attr(obj: Any, name: str, length: int) -> list[float]:
    try:
        value = getattr(obj, name)
    except AttributeError as exc:
        raise RuntimeError(f"ImuState missing required field {name!r}") from exc
    data = [float(x) for x in value]
    if len(data) != length:
        raise RuntimeError(f"ImuState field {name!r} has {len(data)} values, expected {length}")
    return data


def _quat_wxyz_from_rpy(rpy: list[float]) -> list[float]:
    roll, pitch, yaw = rpy
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def _require_empty(value: Mapping[str, Any] | None, name: str) -> None:
    if value:
        raise ValueError(f"{name} action value must be empty")


def _clip(value: float, lower: float, upper: float) -> float:
    return min(max(float(value), float(lower)), float(upper))


def _warning(code: str, message: str) -> dict[str, Any]:
    return {"code": code, "message": message, "source": "robot", "timestamp": time.time()}
