"""Meta Quest 3 UDP leaders using the RCP Cartesian convention."""

from __future__ import annotations

import json
import logging
import math
import socket
import threading
import time
from collections.abc import Mapping, Sequence
from typing import Any

from rynnrcp.robot.base_controller import BaseRobotController
from rynnrcp.robot.cartesian import (
    Pose3D,
    matrix_multiply,
    matrix_to_quaternion,
    matrix_transpose,
    quaternion_to_matrix,
    transform_rotation_basis,
)


CONTROLLER_SIDES = ("left", "right")

# UDP sender convention currently used by the Quest application:
# +x right, +y up, +z forward.
# RCP Cartesian v1: +x forward, +y left, +z up.
QUEST_TO_RCP_BASIS = (
    (0.0, 0.0, -1.0),
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
)
QUEST_TO_RCP_ROTATION_BASIS = (
    (0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
)
QUEST_ROTATION_COMPONENT_SIGNS = (-1.0, -1.0, 1.0)

ZERO_POSE = Pose3D(
    position=(0.0, 0.0, 0.0),
    orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
)


class _MetaQuest3BaseController(BaseRobotController):
    """Shared UDP, calibration, and pose-mapping implementation."""

    def __init__(
        self,
        robot_id: str = "meta_quest3",
        *,
        controller_sides: Sequence[str],
        calibration_label: str,
        bind_host: str = "0.0.0.0",
        udp_port: int = 8888,
        source_ip: str = "192.168.1.120",
        stale_timeout_s: float = 0.2,
        gripper_invert: bool = True,
        calibration_button: str = "primaryButton",
        coordinate_basis: Sequence[Sequence[float]] = QUEST_TO_RCP_BASIS,
        rotation_basis: Sequence[
            Sequence[float]
        ] = QUEST_TO_RCP_ROTATION_BASIS,
        rotation_component_signs: Sequence[
            float
        ] = QUEST_ROTATION_COMPONENT_SIGNS,
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__(logger=logger)
        self.robot_id = str(robot_id)
        self.controller_sides = tuple(str(side) for side in controller_sides)
        if not self.controller_sides or any(
            side not in CONTROLLER_SIDES for side in self.controller_sides
        ):
            raise ValueError("controller_sides must contain left or right")
        self.controller_mode = (
            self.controller_sides[0]
            if len(self.controller_sides) == 1
            else "dual"
        )
        self.calibration_label = str(calibration_label)
        self.bind_host = str(bind_host)
        self.udp_port = int(udp_port)
        if not 0 <= self.udp_port <= 65535:
            raise ValueError("udp_port must be between 0 and 65535")
        self.source_ip = str(source_ip).strip()
        self.stale_timeout_s = float(stale_timeout_s)
        if not math.isfinite(self.stale_timeout_s) or self.stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be positive")
        self.gripper_invert = bool(gripper_invert)
        self.calibration_button = str(calibration_button).strip()
        if not self.calibration_button:
            raise ValueError("calibration_button must not be empty")
        self.coordinate_basis = _validated_basis(coordinate_basis)
        self.rotation_basis = _validated_basis(rotation_basis)
        self.rotation_component_signs = _validated_rotation_signs(
            rotation_component_signs
        )

        self._lock = threading.RLock()
        self._packet_condition = threading.Condition(self._lock)
        self._socket: socket.socket | None = None
        self._worker: threading.Thread | None = None
        self._stop = threading.Event()
        self._poses = {side: ZERO_POSE for side in CONTROLLER_SIDES}
        self._raw_poses: dict[str, Pose3D] = {}
        self._calibration_references: dict[str, Pose3D] = {}
        self._gripper_positions = {side: 1.0 for side in CONTROLLER_SIDES}
        self._grip_pressed = {side: False for side in CONTROLLER_SIDES}
        self._primary_pressed = {side: False for side in CONTROLLER_SIDES}
        self._calibrated = False
        self._calibration_chord_pressed = False
        self._calibration_count = 0
        self._last_packet_at = 0.0
        self._last_sender_timestamps: dict[str, float] = {}
        self._packet_count = 0
        self._last_error = ""
        self._sender: tuple[str, int] | None = None

    def start(self) -> None:
        if self._socket is not None:
            return
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp_socket.bind((self.bind_host, self.udp_port))
        udp_socket.settimeout(0.2)
        self._socket = udp_socket
        self.udp_port = int(udp_socket.getsockname()[1])
        self._stop.clear()
        self._worker = threading.Thread(
            target=self._receive_loop,
            name=f"meta-quest3-{self.robot_id}",
            daemon=True,
        )
        self._worker.start()
        self.logger.info(
            "Meta Quest 3 leader listening on UDP %s:%d",
            self.bind_host,
            self.udp_port,
        )

    def shutdown(self) -> None:
        self._stop.set()
        with self._packet_condition:
            self._packet_condition.notify_all()
        udp_socket, self._socket = self._socket, None
        if udp_socket is not None:
            udp_socket.close()
        worker, self._worker = self._worker, None
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=1.0)

    def get_ee_pose(self) -> dict[str, list[float]]:
        if len(self.controller_sides) != 1:
            raise RuntimeError("use the left and right ee_pose observations")
        return self._get_pose(self.controller_sides[0])

    def get_left_ee_pose(self) -> dict[str, list[float]]:
        return self._get_pose("left")

    def get_right_ee_pose(self) -> dict[str, list[float]]:
        return self._get_pose("right")

    def get_gripper_state(self) -> dict[str, float]:
        if len(self.controller_sides) != 1:
            raise RuntimeError("use the left and right gripper observations")
        return self._get_gripper(self.controller_sides[0])

    def get_left_gripper_state(self) -> dict[str, float]:
        return self._get_gripper("left")

    def get_right_gripper_state(self) -> dict[str, float]:
        return self._get_gripper("right")

    def get_controller_state(self) -> dict[str, Any]:
        with self._lock:
            age_s = (
                None
                if self._last_packet_at <= 0.0
                else max(0.0, time.monotonic() - self._last_packet_at)
            )
            return {
                "controller_mode": self.controller_mode,
                "calibrated": self._calibrated,
                "calibration_count": self._calibration_count,
                "calibration_chord_pressed": self._calibration_chord_pressed,
                "primary_pressed": dict(self._primary_pressed),
                "grip_pressed": dict(self._grip_pressed),
                "packet_count": self._packet_count,
                "packet_age_s": age_s,
                "source": None
                if self._sender is None
                else {"ip": self._sender[0], "port": self._sender[1]},
            }

    def wait_for_active_grip(
        self, sides: Sequence[str] | None = None
    ) -> dict[str, bool]:
        """Wait until fresh calibrated data reports an active Grip."""
        requested_sides = tuple(sides or self.controller_sides)
        if not requested_sides or any(
            side not in self.controller_sides for side in requested_sides
        ):
            raise ValueError("requested Grip is not enabled")
        with self._packet_condition:
            while not self._stop.is_set():
                packet_age = (
                    math.inf
                    if self._last_packet_at <= 0.0
                    else max(0.0, time.monotonic() - self._last_packet_at)
                )
                pressed = {
                    side: bool(self._grip_pressed[side])
                    for side in requested_sides
                }
                if (
                    self._calibrated
                    and packet_age <= self.stale_timeout_s
                    and any(pressed.values())
                ):
                    return pressed
                self._packet_condition.wait(
                    timeout=min(0.05, self.stale_timeout_s)
                )
        raise RuntimeError("Meta Quest 3 controller stopped")

    def get_health(self) -> dict[str, list[dict[str, Any]]]:
        warnings: list[dict[str, Any]] = []
        now = time.time()
        with self._lock:
            packet_count = self._packet_count
            packet_age = (
                math.inf
                if self._last_packet_at <= 0.0
                else time.monotonic() - self._last_packet_at
            )
            calibrated = self._calibrated
            last_error = self._last_error
        if packet_count == 0:
            warnings.append(
                self._health_item(
                    "meta_quest3.waiting",
                    f"Waiting for UDP data on port {self.udp_port}",
                    now,
                )
            )
        elif packet_age > self.stale_timeout_s:
            warnings.append(
                self._health_item(
                    "meta_quest3.stale",
                    f"Quest data is {packet_age:.3f} seconds old",
                    now,
                )
            )
        if packet_count > 0 and not calibrated:
            warnings.append(
                self._health_item(
                    "meta_quest3.calibration_required",
                    "Hold the neutral pose and press "
                    f"{self.calibration_label}",
                    now,
                )
            )
        if last_error:
            warnings.append(
                self._health_item("meta_quest3.packet_error", last_error, now)
            )
        return {"errors": [], "warnings": warnings}

    def _get_pose(self, side: str) -> dict[str, list[float]]:
        with self._lock:
            if side not in self.controller_sides:
                raise RuntimeError(
                    f"{side} controller is not enabled in {self.controller_mode} mode"
                )
            self._require_fresh_packet()
            if not self._calibrated:
                raise RuntimeError(
                    "Calibration required: stand in the neutral pose and "
                    f"press {self.calibration_label}"
                )
            if side not in self._raw_poses:
                raise RuntimeError(f"Waiting for {side}Controller UDP data")
            return self._poses[side].to_value()

    def _get_gripper(self, side: str) -> dict[str, float]:
        with self._lock:
            if side not in self.controller_sides:
                raise RuntimeError(
                    f"{side} controller is not enabled in {self.controller_mode} mode"
                )
            self._require_fresh_packet()
            if side not in self._raw_poses:
                raise RuntimeError(f"Waiting for {side}Controller UDP data")
            return {"position": self._gripper_positions[side]}

    def _require_fresh_packet(self) -> None:
        if self._packet_count == 0 or self._last_packet_at <= 0.0:
            raise RuntimeError("Waiting for Meta Quest 3 UDP data")
        packet_age = max(0.0, time.monotonic() - self._last_packet_at)
        if packet_age > self.stale_timeout_s:
            raise RuntimeError(
                "Meta Quest 3 UDP data timed out "
                f"({packet_age:.3f}s > {self.stale_timeout_s:.3f}s); "
                "joint output is paused"
            )

    def _receive_loop(self) -> None:
        while not self._stop.is_set():
            udp_socket = self._socket
            if udp_socket is None:
                return
            try:
                data, sender = udp_socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                return
            if self.source_ip and sender[0] != self.source_ip:
                continue
            try:
                payload = json.loads(data.decode("utf-8"))
                self._process_payload(payload, sender, time.monotonic())
            except Exception as exc:
                message = f"{type(exc).__name__}: {exc}"
                with self._lock:
                    if message != self._last_error:
                        self.logger.warning(
                            "Meta Quest 3 UDP packet rejected: %s", message
                        )
                    self._last_error = message

    def _process_payload(
        self,
        payload: Any,
        sender: tuple[str, int],
        received_at: float,
    ) -> None:
        if not isinstance(payload, Mapping):
            raise ValueError("UDP payload must be a JSON object")

        samples: dict[str, tuple[Pose3D, Mapping[str, Any], float | None]] = {}
        for side in self.controller_sides:
            controller = payload.get(f"{side}Controller")
            if not isinstance(controller, Mapping):
                continue
            raw_pose = _controller_pose(controller)
            input_state = controller.get("input")
            if not isinstance(input_state, Mapping):
                input_state = {}
            samples[side] = (
                raw_pose,
                input_state,
                _optional_timestamp(controller.get("timestamp")),
            )
        if not samples:
            raise ValueError(
                "UDP payload is missing the configured controller data"
            )

        with self._lock:
            accepted: dict[str, tuple[Pose3D, Mapping[str, Any]]] = {}
            for side, (raw_pose, input_state, sender_timestamp) in samples.items():
                previous_timestamp = self._last_sender_timestamps.get(side)
                if (
                    sender_timestamp is not None
                    and previous_timestamp is not None
                    and sender_timestamp < previous_timestamp
                ):
                    continue
                if sender_timestamp is not None:
                    self._last_sender_timestamps[side] = sender_timestamp
                accepted[side] = (raw_pose, input_state)
                self._raw_poses[side] = raw_pose
                trigger = _clamp(
                    float(input_state.get("trigger", 0.0)), 0.0, 1.0
                )
                self._gripper_positions[side] = (
                    1.0 - trigger if self.gripper_invert else trigger
                )
                self._grip_pressed[side] = bool(
                    input_state.get("gripPressed")
                )
                self._primary_pressed[side] = bool(
                    input_state.get(self.calibration_button)
                )
            if not accepted:
                return

            calibration_chord = all(
                side in self._raw_poses and self._primary_pressed[side]
                for side in self.controller_sides
            )
            if calibration_chord and not self._calibration_chord_pressed:
                self._calibration_references = {
                    side: self._raw_poses[side]
                    for side in self.controller_sides
                }
                for side in self.controller_sides:
                    self._poses[side] = ZERO_POSE
                self._calibrated = True
                self._calibration_count += 1
                self.logger.info(
                    "Meta Quest 3 neutral pose calibrated with %s",
                    self.calibration_label,
                )
            self._calibration_chord_pressed = calibration_chord

            if self._calibrated:
                for side, (raw_pose, _input_state) in accepted.items():
                    reference = self._calibration_references.get(side)
                    if reference is not None:
                        self._poses[side] = _map_calibrated_pose(
                            raw_pose,
                            reference,
                            self._poses[side].orientation_xyzw,
                            self.coordinate_basis,
                            self.rotation_basis,
                            self.rotation_component_signs,
                        )

            self._last_packet_at = received_at
            self._packet_count += 1
            self._last_error = ""
            self._sender = sender
            self._packet_condition.notify_all()

    def _health_item(
        self, code: str, message: str, timestamp: float
    ) -> dict[str, Any]:
        return {
            "code": code,
            "message": message,
            "source": "robot",
            "timestamp": timestamp,
            "details": {
                "robot_id": self.robot_id,
                "controller_mode": self.controller_mode,
                "udp_port": self.udp_port,
                "source_ip": self.source_ip,
            },
        }


class MetaQuest3RightController(_MetaQuest3BaseController):
    """Expose the right controller and calibrate it with A."""

    def __init__(self, robot_id: str = "meta_quest3_right", **kwargs: Any) -> None:
        super().__init__(
            robot_id=robot_id,
            controller_sides=("right",),
            calibration_label="A",
            **kwargs,
        )


class MetaQuest3DualController(_MetaQuest3BaseController):
    """Expose both controllers and calibrate them with X+A."""

    def __init__(self, robot_id: str = "meta_quest3_dual", **kwargs: Any) -> None:
        super().__init__(
            robot_id=robot_id,
            controller_sides=CONTROLLER_SIDES,
            calibration_label="X+A",
            **kwargs,
        )


def _controller_pose(controller: Mapping[str, Any]) -> Pose3D:
    position = controller.get("position")
    rotation = controller.get("rotation")
    if not isinstance(position, Mapping) or not isinstance(rotation, Mapping):
        raise ValueError("controller position and rotation are required")
    return Pose3D.from_value(
        {
            "position": [
                position.get("x"),
                position.get("y"),
                position.get("z"),
            ],
            "orientation_quat_xyzw": [
                rotation.get("x"),
                rotation.get("y"),
                rotation.get("z"),
                rotation.get("w"),
            ],
        }
    )


def _map_calibrated_pose(
    current: Pose3D,
    reference: Pose3D,
    previous_quaternion: tuple[float, float, float, float],
    coordinate_basis: Sequence[Sequence[float]],
    rotation_basis: Sequence[Sequence[float]],
    rotation_component_signs: Sequence[float],
) -> Pose3D:
    source_delta = tuple(
        current.position[index] - reference.position[index]
        for index in range(3)
    )
    position = tuple(
        sum(
            coordinate_basis[row][column] * source_delta[column]
            for column in range(3)
        )
        for row in range(3)
    )
    source_rotation_delta = matrix_multiply(
        quaternion_to_matrix(current.orientation_xyzw),
        matrix_transpose(quaternion_to_matrix(reference.orientation_xyzw)),
    )
    mapped_quaternion = matrix_to_quaternion(
        transform_rotation_basis(source_rotation_delta, rotation_basis)
    )
    quaternion = (
        mapped_quaternion[0] * rotation_component_signs[0],
        mapped_quaternion[1] * rotation_component_signs[1],
        mapped_quaternion[2] * rotation_component_signs[2],
        mapped_quaternion[3],
    )
    if sum(a * b for a, b in zip(quaternion, previous_quaternion)) < 0.0:
        quaternion = tuple(-item for item in quaternion)  # type: ignore[assignment]
    return Pose3D(position=position, orientation_xyzw=quaternion)


def _optional_timestamp(value: Any) -> float | None:
    if value is None:
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _validated_rotation_signs(
    value: Sequence[float],
) -> tuple[float, float, float]:
    try:
        signs = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "rotation_component_signs must contain three values"
        ) from exc
    if len(signs) != 3 or any(item not in {-1.0, 1.0} for item in signs):
        raise ValueError(
            "rotation_component_signs must contain three values of -1 or 1"
        )
    return signs  # type: ignore[return-value]


def _validated_basis(
    value: Sequence[Sequence[float]],
) -> tuple[tuple[float, float, float], ...]:
    try:
        basis = tuple(tuple(float(item) for item in row) for row in value)
    except (TypeError, ValueError) as exc:
        raise ValueError("coordinate_basis must be a 3x3 matrix") from exc
    if len(basis) != 3 or any(len(row) != 3 for row in basis):
        raise ValueError("coordinate_basis must be a 3x3 matrix")
    if not all(math.isfinite(item) for row in basis for item in row):
        raise ValueError("coordinate_basis must contain finite numbers")
    gram = matrix_multiply(basis, matrix_transpose(basis))
    if any(
        abs(gram[row][column] - (1.0 if row == column else 0.0)) > 1e-6
        for row in range(3)
        for column in range(3)
    ):
        raise ValueError("coordinate_basis must be orthonormal")
    determinant = (
        basis[0][0] * (basis[1][1] * basis[2][2] - basis[1][2] * basis[2][1])
        - basis[0][1] * (basis[1][0] * basis[2][2] - basis[1][2] * basis[2][0])
        + basis[0][2] * (basis[1][0] * basis[2][1] - basis[1][1] * basis[2][0])
    )
    if abs(abs(determinant) - 1.0) > 1e-6:
        raise ValueError("coordinate_basis determinant must be +1 or -1")
    return basis


def _clamp(value: float, low: float, high: float) -> float:
    if not math.isfinite(value):
        raise ValueError("controller input must be finite")
    return max(low, min(high, value))
