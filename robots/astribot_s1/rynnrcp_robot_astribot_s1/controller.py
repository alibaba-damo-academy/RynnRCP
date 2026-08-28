"""RynnRCP adapter for the official Astribot S1 Python SDK."""

from __future__ import annotations

import builtins
import importlib
import math
import os
import sys
import threading
import time
from collections.abc import Mapping, Sequence
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from rynnrcp.robot.base_controller import BaseRobotController


PART_NAMES = (
    "astribot_torso",
    "astribot_arm_left",
    "astribot_gripper_left",
    "astribot_arm_right",
    "astribot_gripper_right",
    "astribot_head",
)
PART_DOFS = (4, 7, 1, 7, 1, 2)
JOINT_COUNT = sum(PART_DOFS)
CHASSIS_NAME = "astribot_chassis"
CHASSIS_DOF = 3
LEFT_GRIPPER_NAME = "astribot_gripper_left"
RIGHT_GRIPPER_NAME = "astribot_gripper_right"
GRIPPER_NAMES = frozenset((LEFT_GRIPPER_NAME, RIGHT_GRIPPER_NAME))
SDK_WHOLE_BODY_NAMES = (
    CHASSIS_NAME,
    "astribot_torso",
    "astribot_arm_left",
    LEFT_GRIPPER_NAME,
    "astribot_arm_right",
    RIGHT_GRIPPER_NAME,
    "astribot_head",
)
SDK_WHOLE_BODY_DOFS = (CHASSIS_DOF, 4, 7, 1, 7, 1, 2)
SDK_PART_DOFS = dict(zip(SDK_WHOLE_BODY_NAMES, SDK_WHOLE_BODY_DOFS))
GRIPPER_JOINT_INDICES = (11, 19)
CONTROL_RIGHTS_MODES = {"prompt", "read_only", "force"}
_SDK_INPUT_LOCK = threading.Lock()
DEFAULT_SDK_ROOT = "/home/astribot/astribot_sdk_aarch64"
JOINT_POSITION_RATE_PER_S = 0.5
GRIPPER_POSITION_RATE_PER_S = 50.0


@dataclass(slots=True)
class _StreamingCommand:
    names: list[str]
    current: list[list[float]]
    target: list[list[float]]
    kwargs: dict[str, Any]

    def matches(self, names: Sequence[str], kwargs: Mapping[str, Any]) -> bool:
        return self.names == list(names) and self.kwargs == kwargs

    def advance(self, frequency_hz: float) -> list[list[float]]:
        self.current = [
            [
                _move_towards(
                    current_value,
                    target_value,
                    _position_rate(name) / frequency_hz,
                )
                for current_value, target_value in zip(current, target)
            ]
            for name, current, target in zip(
                self.names,
                self.current,
                self.target,
            )
        ]
        return self.current


class AstribotS1Controller(BaseRobotController):
    """Expose Astribot S1 upper-body and chassis controls through RynnRCP."""

    def __init__(
        self,
        robot_id: str = "astribot_s1",
        sdk_root: str | None = None,
        frequency_hz: float = 100.0,
        high_control_rights: bool = False,
        control_rights_mode: str = "prompt",
        node_name: str = "rynnrcp_astribot_s1",
        control_way: str = "filter",
        use_wbc: bool = False,
        activate_cameras: bool = True,
        base_command_dt_s: float = 0.02,
        max_base_x_m_s: float = 0.3,
        max_base_y_m_s: float = 0.3,
        max_base_yaw_rad_s: float = 0.5,
    ) -> None:
        super().__init__()
        sdk_root = (
            sdk_root
            or os.environ.get("ASTRIBOT_SDK_ROOT")
            or DEFAULT_SDK_ROOT
        )
        if control_way not in {"filter", "direct"}:
            raise ValueError("control_way must be 'filter' or 'direct'")
        self.robot_id = str(robot_id)
        self.sdk_root = Path(sdk_root).expanduser().resolve()
        self.frequency_hz = _positive(frequency_hz, "frequency_hz")
        self.high_control_rights = bool(high_control_rights)
        if control_rights_mode not in CONTROL_RIGHTS_MODES:
            raise ValueError(
                "control_rights_mode must be 'prompt', 'read_only', or 'force'"
            )
        self.control_rights_mode = control_rights_mode
        self.node_name = str(node_name)
        self.control_way = control_way
        self.use_wbc = bool(use_wbc)
        self.activate_cameras = bool(activate_cameras)
        self.base_command_dt_s = _positive(base_command_dt_s, "base_command_dt_s")
        self.max_base_x_m_s = _positive(max_base_x_m_s, "max_base_x_m_s")
        self.max_base_y_m_s = _positive(max_base_y_m_s, "max_base_y_m_s")
        self.max_base_yaw_rad_s = _positive(max_base_yaw_rad_s, "max_base_yaw_rad_s")
        self._client: Any = None
        self._lock = threading.RLock()
        self._last_error: tuple[str, float] | None = None
        self._streaming_command: _StreamingCommand | None = None
        self._command_stop = threading.Event()
        self._command_thread: threading.Thread | None = None
        self._last_command_error_log = 0.0

    def start(self) -> None:
        with self._lock:
            if self._client is not None:
                return
            if not self.sdk_root.is_dir():
                raise RuntimeError(f"Astribot SDK root does not exist: {self.sdk_root}")
            os.environ.setdefault("ROBOT_TYPE", "S1")
            if str(self.sdk_root) not in sys.path:
                sys.path.insert(0, str(self.sdk_root))
            client: Any = None
            try:
                module = importlib.import_module(
                    "astribot_sdk.core.astribot_api.astribot_client"
                )
                with _sdk_control_rights_input(self.control_rights_mode):
                    client = module.Astribot(
                        freq=self.frequency_hz,
                        high_control_rights=(
                            self.high_control_rights
                            or self.control_rights_mode == "force"
                        ),
                        node_name=self.node_name,
                    )
                self._validate_layout(client)
                if self.activate_cameras:
                    client.activate_camera()
            except Exception as exc:
                interface = getattr(client, "astribot_interface", None)
                shutdown = getattr(interface, "shutdown", None)
                if callable(shutdown):
                    try:
                        shutdown()
                    except Exception:
                        pass
                self._record_error(exc)
                raise RuntimeError(
                    "Astribot SDK startup failed. Source the SDK env.sh before starting RynnRCP "
                    f"and verify the robot is active: {exc}"
                ) from exc
            self._client = client
            self._last_error = None
            self._command_stop.clear()
            self._command_thread = threading.Thread(
                target=self._command_loop,
                name=f"{self.node_name}_commands",
                daemon=True,
            )
            self._command_thread.start()

    def shutdown(self) -> None:
        self._command_stop.set()
        with self._lock:
            command_thread, self._command_thread = self._command_thread, None
            self._streaming_command = None
        if (
            command_thread is not None
            and command_thread is not threading.current_thread()
        ):
            command_thread.join(timeout=1.0)
        with self._lock:
            client, self._client = self._client, None
            if client is None:
                return
            interface = getattr(client, "astribot_interface", None)
            shutdown = getattr(interface, "shutdown", None)
            if callable(shutdown):
                try:
                    shutdown()
                except Exception as exc:
                    self._record_error(exc)
                    self.logger.warning("Astribot SDK shutdown failed: %s", exc)

    def get_joint_positions(self) -> dict[str, list[float]]:
        with self._sdk_call() as client:
            positions = _normalize_gripper_slots(
                _flatten_parts(
                    client.get_current_joints_position(list(PART_NAMES)),
                    "joint positions",
                )
            )
            velocities = _normalize_gripper_slots(
                _flatten_parts(
                    client.get_current_joints_velocity(list(PART_NAMES)),
                    "joint velocities",
                )
            )
        return {"joint_positions": positions, "joint_velocities": velocities}

    def get_joint_limits(self) -> dict[str, list[float]]:
        with self._sdk_call() as client:
            lower, upper = client.get_joints_position_limit(list(PART_NAMES))
        return {
            "lower": _normalize_gripper_slots(
                _flatten_parts(lower, "joint lower limits")
            ),
            "upper": _normalize_gripper_slots(
                _flatten_parts(upper, "joint upper limits")
            ),
        }

    def get_chassis_state(self) -> dict[str, list[float]]:
        with self._sdk_call() as client:
            position = _nested_vector(
                client.get_current_joints_position([CHASSIS_NAME]), CHASSIS_DOF, "chassis position"
            )[0]
            velocity = _nested_vector(
                client.get_current_joints_velocity([CHASSIS_NAME]), CHASSIS_DOF, "chassis velocity"
            )[0]
        return {"position_xy_yaw": position, "velocity_xy_yaw": velocity}

    def get_left_gripper_state(self) -> dict[str, float]:
        return self._get_gripper_state(LEFT_GRIPPER_NAME)

    def get_right_gripper_state(self) -> dict[str, float]:
        return self._get_gripper_state(RIGHT_GRIPPER_NAME)

    def set_joint_positions(self, value: Mapping[str, Any]) -> dict[str, list[float]]:
        positions = _vector_from_action(value, "joint_positions", JOINT_COUNT)
        sdk_positions = list(positions)
        for index in GRIPPER_JOINT_INDICES:
            if not 0.0 <= positions[index] <= 1.0:
                raise ValueError(
                    f"joint_positions[{index}] gripper target must be in [0, 1]"
                )
            sdk_positions[index] *= 100.0
        commands = _split_parts(sdk_positions)
        self._set_streaming_position(
            list(PART_NAMES),
            commands,
            use_wbc=self.use_wbc,
        )
        return {"joint_positions": positions}

    def set_joint_velocities(self, value: Mapping[str, Any]) -> dict[str, list[float]]:
        velocities = _vector_from_action(value, "joint_velocities", JOINT_COUNT)
        sdk_velocities = list(velocities)
        for index in GRIPPER_JOINT_INDICES:
            sdk_velocities[index] *= 100.0
        with self._sdk_call() as client:
            self._streaming_command = None
            client.set_joints_velocity(
                list(PART_NAMES), _split_parts(sdk_velocities)
            )
        return {"joint_velocities": velocities}

    def set_single_joint_position(self, index: int, value: float) -> dict[str, Any]:
        name, local_index, dof = _part_for_joint_index(index)
        if name in {LEFT_GRIPPER_NAME, RIGHT_GRIPPER_NAME}:
            raise ValueError("use the dedicated normalized gripper action")
        target_value = float(value)
        if not math.isfinite(target_value):
            raise ValueError("joint target must be finite")
        with self._sdk_call() as client:
            previous = self._streaming_command
            if previous is not None and previous.matches(
                [name],
                _position_kwargs("direct", self.use_wbc),
            ):
                target = list(previous.target[0])
            else:
                target = _nested_vector(
                    client.get_current_joints_position([name]),
                    dof,
                    f"{name} position",
                )[0]
            target[local_index] = target_value
        self._set_streaming_position(
            [name],
            [target],
            use_wbc=self.use_wbc,
            control_way="direct",
        )
        return {
            "index": int(index),
            "part": name,
            "part_index": local_index,
            "position": target_value,
        }

    def set_base_velocity(self, value: Mapping[str, Any]) -> dict[str, list[float]]:
        if not isinstance(value, Mapping):
            raise ValueError("base_velocity action value must be an object")
        linear = _optional_xyz(value.get("linear_vel"), "linear_vel")
        angular = _optional_xyz(value.get("angular_vel"), "angular_vel")
        if "linear_vel" not in value and "angular_vel" not in value:
            raise ValueError("base_velocity requires linear_vel or angular_vel")
        velocity = [
            _clamp(linear[0], self.max_base_x_m_s),
            _clamp(linear[1], self.max_base_y_m_s),
            _clamp(angular[2], self.max_base_yaw_rad_s),
        ]
        with self._sdk_call() as client:
            desired = _nested_vector(
                client.get_desired_joints_position([CHASSIS_NAME]), CHASSIS_DOF, "desired chassis position"
            )[0]
            target = [
                desired[index] + velocity[index] * self.base_command_dt_s
                for index in range(CHASSIS_DOF)
            ]
        self._set_streaming_position([CHASSIS_NAME], [target], use_wbc=False)
        return {
            "linear_vel": [velocity[0], velocity[1], 0.0],
            "angular_vel": [0.0, 0.0, velocity[2]],
        }

    def set_left_gripper(self, value: Mapping[str, Any]) -> dict[str, float]:
        return self._set_gripper(LEFT_GRIPPER_NAME, value)

    def set_right_gripper(self, value: Mapping[str, Any]) -> dict[str, float]:
        return self._set_gripper(RIGHT_GRIPPER_NAME, value)

    def home(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "home")
        with self._sdk_call() as client:
            self._streaming_command = None
            result = client.move_to_home()
        return {"command": "home", "result": result}

    def stop(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "stop")
        with self._sdk_call() as client:
            self._streaming_command = None
            result = client.stop_robot()
        return {"command": "stop", "result": result}

    def restart(self, value: Mapping[str, Any] | None = None) -> dict[str, Any]:
        _require_empty(value, "restart")
        with self._sdk_call() as client:
            self._streaming_command = None
            result = client.restart_robot()
        return {"command": "restart", "result": result}

    def get_health(self) -> dict[str, list[dict[str, Any]]]:
        warnings: list[dict[str, Any]] = []
        errors: list[dict[str, Any]] = []
        now = time.time()
        with self._lock:
            client = self._client
            if client is None:
                warnings.append(self._health_item("astribot_s1.not_started", "Astribot S1 is not connected", now))
            elif not bool(getattr(client, "is_alive", False)):
                errors.append(self._health_item("astribot_s1.not_alive", "Astribot S1 interface is not alive", now))
            else:
                try:
                    has_control = bool(client.get_control_rights_status())
                except Exception as exc:
                    errors.append(
                        self._health_item(
                            "astribot_s1.health_failed",
                            f"Astribot S1 control-rights check failed: {exc}",
                            now,
                        )
                    )
                else:
                    if not has_control:
                        warnings.append(
                            self._health_item(
                                "astribot_s1.no_control_rights",
                                "Astribot S1 is connected without motion control rights",
                                now,
                            )
                        )
        if self._last_error is not None:
            message, timestamp = self._last_error
            errors.append(self._health_item("astribot_s1.sdk_error", message, timestamp))
        return {"errors": errors, "warnings": warnings}

    def has_control_rights(self) -> bool:
        with self._sdk_call() as client:
            return bool(client.get_control_rights_status())

    def clear_command(self) -> None:
        with self._lock:
            self._streaming_command = None

    def _set_gripper(self, name: str, value: Mapping[str, Any]) -> dict[str, float]:
        if not isinstance(value, Mapping) or "position" not in value:
            raise ValueError("gripper action requires position in [0, 1]")
        position = float(value["position"])
        if not math.isfinite(position) or position < 0.0 or position > 1.0:
            raise ValueError("gripper position must be finite and in [0, 1]")
        with self._sdk_call() as client:
            if "force" in value:
                force = float(value["force"])
                if not math.isfinite(force) or force <= 0.0:
                    raise ValueError("gripper force must be a positive finite value")
                client.set_effector_max_force([name], [force])
        self._set_streaming_position(
            [name],
            [[position * 100.0]],
            use_wbc=False,
            control_way="direct",
        )
        result = {"position": position}
        if "force" in value:
            result["force"] = float(value["force"])
        return result

    def _get_gripper_state(self, name: str) -> dict[str, float]:
        with self._sdk_call() as client:
            position = _nested_vector(
                client.get_current_joints_position([name]), 1, f"{name} position"
            )[0][0]
            velocity = _nested_vector(
                client.get_current_joints_velocity([name]), 1, f"{name} velocity"
            )[0][0]
        return {
            "position": position / 100.0,
            "velocity": velocity / 100.0,
            "sdk_joint_position": position,
            "sdk_joint_velocity": velocity,
        }

    def _validate_layout(self, client: Any) -> None:
        names = list(getattr(client, "whole_body_names", ()))
        dofs = list(getattr(client, "whole_body_dofs", ()))
        expected_names = list(SDK_WHOLE_BODY_NAMES)
        expected_dofs = list(SDK_WHOLE_BODY_DOFS)
        if names != expected_names or dofs != expected_dofs:
            raise RuntimeError(
                f"unsupported Astribot layout: names={names}, dofs={dofs}; "
                f"expected S1 names={expected_names}, dofs={expected_dofs}"
            )

    def _record_error(self, exc: Exception) -> None:
        self._last_error = (str(exc), time.time())

    def _set_streaming_position(
        self,
        names: list[str],
        positions: list[list[float]],
        *,
        use_wbc: bool,
        control_way: str | None = None,
    ) -> None:
        kwargs = _position_kwargs(control_way or self.control_way, use_wbc)
        with self._sdk_call() as client:
            dofs = _dofs_for_names(names)
            previous = self._streaming_command
            if previous is not None and previous.matches(names, kwargs):
                current = [list(part) for part in previous.current]
            else:
                current = _nested_vector(
                    client.get_current_joints_position(names),
                    sum(dofs),
                    "streaming position",
                    dofs=dofs,
                )
            self._streaming_command = _StreamingCommand(
                names=list(names),
                current=current,
                target=[list(part) for part in positions],
                kwargs=kwargs,
            )

    def _command_loop(self) -> None:
        period = 1.0 / self.frequency_hz
        while not self._command_stop.wait(period):
            with self._lock:
                client = self._client
                command = self._streaming_command
                if client is None or command is None:
                    continue
                try:
                    if not bool(client.get_control_rights_status()):
                        self._streaming_command = None
                        continue
                    values = command.advance(self.frequency_hz)
                    client.set_joints_position(
                        command.names, values, **command.kwargs
                    )
                except Exception as exc:
                    self._record_error(exc)
                    now = time.monotonic()
                    if now - self._last_command_error_log >= 5.0:
                        self._last_command_error_log = now
                        self.logger.warning(
                            "Astribot streaming command failed: %s", exc
                        )

    def _health_item(self, code: str, message: str, timestamp: float) -> dict[str, Any]:
        return {
            "code": code,
            "message": message,
            "source": "robot",
            "timestamp": timestamp,
            "details": {"robot_id": self.robot_id, "sdk_root": str(self.sdk_root)},
        }

    class _SdkCall:
        def __init__(self, controller: "AstribotS1Controller") -> None:
            self.controller = controller

        def __enter__(self) -> Any:
            self.controller._lock.acquire()
            client = self.controller._client
            if client is None:
                self.controller._lock.release()
                raise RuntimeError("Astribot S1 controller is not started")
            return client

        def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> bool:
            if exc is not None:
                self.controller._record_error(exc)
            self.controller._lock.release()
            return False

    def _sdk_call(self) -> "AstribotS1Controller._SdkCall":
        return self._SdkCall(self)


@contextmanager
def _sdk_control_rights_input(mode: str):
    if mode == "prompt":
        yield
        return
    response = "yes" if mode == "force" else ""
    with _SDK_INPUT_LOCK:
        original_input = builtins.input
        builtins.input = lambda *args, **kwargs: response
        try:
            yield
        finally:
            builtins.input = original_input


def _flatten_parts(values: Any, label: str) -> list[float]:
    nested = _nested_vector(values, JOINT_COUNT, label, dofs=PART_DOFS)
    return [item for part in nested for item in part]


def _nested_vector(
    values: Any, total: int, label: str, *, dofs: Sequence[int] | None = None
) -> list[list[float]]:
    expected_dofs = tuple(dofs or (total,))
    if not isinstance(values, Sequence) or isinstance(values, (str, bytes)):
        raise RuntimeError(f"Astribot SDK returned invalid {label}")
    if len(values) != len(expected_dofs):
        raise RuntimeError(f"Astribot SDK returned {len(values)} {label} parts; expected {len(expected_dofs)}")
    result: list[list[float]] = []
    for index, (part, dof) in enumerate(zip(values, expected_dofs)):
        result.append(_finite_vector(part, dof, f"{label} part {index}"))
    if sum(len(part) for part in result) != total:
        raise RuntimeError(f"Astribot SDK returned invalid {label} length")
    return result


def _split_parts(values: Sequence[float]) -> list[list[float]]:
    result: list[list[float]] = []
    offset = 0
    for dof in PART_DOFS:
        result.append(list(values[offset : offset + dof]))
        offset += dof
    return result


def _normalize_gripper_slots(values: Sequence[float]) -> list[float]:
    result = list(values)
    for index in GRIPPER_JOINT_INDICES:
        result[index] /= 100.0
    return result


def _part_for_joint_index(index: int) -> tuple[str, int, int]:
    if index not in range(JOINT_COUNT):
        raise ValueError(f"joint index must be between 0 and {JOINT_COUNT - 1}")
    offset = 0
    for name, dof in zip(PART_NAMES, PART_DOFS):
        if index < offset + dof:
            return name, index - offset, dof
        offset += dof
    raise AssertionError("unreachable joint index")


def _dofs_for_names(names: Sequence[str]) -> tuple[int, ...]:
    try:
        return tuple(SDK_PART_DOFS[name] for name in names)
    except KeyError as exc:
        raise ValueError(f"unsupported Astribot part: {exc.args[0]}") from exc


def _position_rate(name: str) -> float:
    if name in GRIPPER_NAMES:
        return GRIPPER_POSITION_RATE_PER_S
    return JOINT_POSITION_RATE_PER_S


def _position_kwargs(control_way: str, use_wbc: bool) -> dict[str, Any]:
    return {
        "control_way": control_way,
        "use_wbc": use_wbc,
        "add_default_torso": False,
    }


def _move_towards(current: float, target: float, step: float) -> float:
    if target > current:
        return min(target, current + step)
    return max(target, current - step)


def _vector_from_action(value: Mapping[str, Any], key: str, length: int) -> list[float]:
    if not isinstance(value, Mapping) or key not in value:
        raise ValueError(f"action requires {key}")
    return _finite_vector(value[key], length, key)


def _finite_vector(value: Any, length: int, name: str) -> list[float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != length:
        raise ValueError(f"{name} must contain {length} values")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


def _optional_xyz(value: Any, name: str) -> list[float]:
    return [0.0, 0.0, 0.0] if value is None else _finite_vector(value, 3, name)


def _positive(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be a positive finite value")
    return result


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _require_empty(value: Mapping[str, Any] | None, name: str) -> None:
    if value:
        raise ValueError(f"{name} action value must be empty")
