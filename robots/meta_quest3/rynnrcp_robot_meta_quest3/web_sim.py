"""Configuration-driven robot-arm simulator for Meta Quest 3."""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
import webbrowser
import xml.etree.ElementTree as ET
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from rynnrcp.robot.cartesian import (
    matrix_multiply,
    matrix_transpose,
    quaternion_to_matrix,
)

from .controller import MetaQuest3DualController, MetaQuest3RightController
from .kinematics import (
    IkResult,
    Matrix3,
    UrdfKinematicChain,
    quaternion_xyzw_from_rotation,
    rotation_from_rpy,
    rotation_multiply,
    rpy_from_rotation,
)


SIMULATION_RATE_HZ = 60.0
MAX_JOINT_VELOCITY_RAD_S = 0.25
MAX_JOINT_ACCELERATION_RAD_S2 = 0.5
MAX_TARGET_TRANSLATION_M_S = 0.15
MAX_TARGET_ROTATION_RAD_S = 0.6
WORKSPACE_DELTA_LIMITS_M = (
    (-0.18, 0.40),
    (-0.50, 0.50),
    (-0.50, 0.00),
)
PACKAGE_DIR = Path(__file__).resolve().parent
WEB_DIR = PACKAGE_DIR / "web"
MODEL_ASSET_DIR = PACKAGE_DIR / "models" / "assets"
WEB_PAGE_PATH = WEB_DIR / "index.html"
DUAL_WEB_PAGE_PATH = WEB_DIR / "dual.html"


class UrdfJointSimulation:
    """Thread-safe URDF joint state and Cartesian IK target."""

    def __init__(
        self,
        urdf_path: str | Path,
        *,
        base_link: str,
        tip_link: str,
        home_joint_positions: Sequence[float],
        workspace_delta_limits_m: Sequence[
            Sequence[float]
        ] = WORKSPACE_DELTA_LIMITS_M,
        max_joint_velocity_rad_s: float = MAX_JOINT_VELOCITY_RAD_S,
        max_joint_acceleration_rad_s2: float = MAX_JOINT_ACCELERATION_RAD_S2,
        max_target_translation_m_s: float = MAX_TARGET_TRANSLATION_M_S,
        max_target_rotation_rad_s: float = MAX_TARGET_ROTATION_RAD_S,
        model_name: str = "robot_arm",
        display_name: str | None = None,
        quest_translation_scale: float = 1.0,
        has_gripper: bool = True,
        home_gripper_position: float = 1.0,
        native_gripper_joint: str | None = None,
        gripper_mount_link: str | None = None,
        joint_names: Sequence[str] | None = None,
    ) -> None:
        self.model_name = str(model_name)
        self.display_name = str(display_name or model_name)
        self.has_gripper = bool(has_gripper)
        self.native_gripper_joint = (
            str(native_gripper_joint).strip()
            if native_gripper_joint
            else None
        )
        self.gripper_mount_link = (
            str(gripper_mount_link).strip()
            if gripper_mount_link
            else None
        )
        self.quest_translation_scale = _positive(
            quest_translation_scale, "translation_scale"
        )
        self.chain = UrdfKinematicChain(
            urdf_path,
            base_link=base_link,
            tip_link=tip_link,
        )
        self.joint_names = (
            tuple(str(name) for name in joint_names)
            if joint_names is not None
            else self.chain.joint_names
        )
        if len(self.joint_names) != len(self.chain.active_joints):
            raise ValueError(
                "joint_names must match the active arm joint count"
            )
        if self.native_gripper_joint:
            urdf_joint_names = {joint.name for joint in self.chain.joints}
            if self.native_gripper_joint not in urdf_joint_names:
                raise ValueError(
                    f"native gripper joint {self.native_gripper_joint!r} "
                    "was not found in the URDF"
                )
        if self.gripper_mount_link:
            urdf_link_names = {
                self.chain.base_link,
                *(
                    joint.parent
                    for joint in self.chain.joints
                ),
                *(
                    joint.child
                    for joint in self.chain.joints
                ),
            }
            if self.gripper_mount_link not in urdf_link_names:
                raise ValueError(
                    f"gripper mount link {self.gripper_mount_link!r} "
                    "was not found in the URDF"
                )
        self.visuals = _load_urdf_visuals(
            Path(urdf_path),
            self.model_name,
        )
        self._lock = threading.RLock()
        self.home_joint_positions = self.chain.clamp(home_joint_positions)
        self._workspace_delta_limits_m = _validated_workspace_limits(
            workspace_delta_limits_m
        )
        self.max_joint_velocity_rad_s = _positive(
            max_joint_velocity_rad_s, "max_joint_velocity_rad_s"
        )
        self.max_joint_acceleration_rad_s2 = _positive(
            max_joint_acceleration_rad_s2, "max_joint_acceleration_rad_s2"
        )
        self.max_target_translation_m_s = _positive(
            max_target_translation_m_s, "max_target_translation_m_s"
        )
        self.max_target_rotation_rad_s = _positive(
            max_target_rotation_rad_s, "max_target_rotation_rad_s"
        )
        joint_count = len(self.chain.active_joints)
        self._joint_positions = self.home_joint_positions
        home = self.chain.forward(self.home_joint_positions)
        self._home_position = home.position
        self._home_rotation = home.rotation
        self._target_position = home.position
        self._target_rotation = home.rotation
        self._desired_position = home.position
        self._desired_rotation = home.rotation
        self._joint_velocities = (0.0,) * joint_count
        self._joint_goal: tuple[float, ...] | None = None
        self._gripper_position = float(home_gripper_position)
        if (
            not math.isfinite(self._gripper_position)
            or not 0.0 <= self._gripper_position <= 1.0
        ):
            raise ValueError("home_gripper_position must be between 0 and 1")
        self._ik_result = IkResult(
            self.home_joint_positions, True, 0, 0.0, 0.0
        )
        self._source = "manual"
        self._quest_state: dict[str, Any] = {
            "enabled": False,
            "calibrated": False,
            "clutch_pressed": False,
            "gripper_position": self._gripper_position,
            "packet_count": 0,
            "packet_age_s": None,
        }

    @property
    def target_pose(self) -> tuple[tuple[float, ...], Matrix3]:
        with self._lock:
            return self._target_position, self._target_rotation

    def home(self, *, source: str = "manual") -> None:
        with self._lock:
            self._desired_position = self._home_position
            self._desired_rotation = self._home_rotation
            self._joint_goal = self.home_joint_positions
            self._source = source

    def set_target_delta(
        self,
        position_delta: Sequence[float],
        rotation_delta_rpy: Sequence[float],
    ) -> IkResult:
        delta = tuple(float(item) for item in position_delta)
        if len(delta) != 3:
            raise ValueError("position_delta must contain three values")
        position = tuple(
            self._home_position[index] + delta[index] for index in range(3)
        )
        rotation = rotation_multiply(
            rotation_from_rpy(rotation_delta_rpy),
            self._home_rotation,
        )
        return self.set_target_absolute(position, rotation, source="manual")

    def set_target_ee_pose(
        self,
        position: Sequence[float],
        orientation_rpy: Sequence[float],
    ) -> IkResult:
        return self.set_target_absolute(
            position,
            rotation_from_rpy(orientation_rpy),
            source="manual",
        )

    def set_target_absolute(
        self,
        position: Sequence[float],
        rotation: Sequence[Sequence[float]],
        *,
        source: str,
    ) -> IkResult:
        with self._lock:
            desired_position = tuple(float(item) for item in position)
            if len(desired_position) != 3 or not all(
                math.isfinite(item) for item in desired_position
            ):
                raise ValueError("position must contain three finite values")
            desired_position = tuple(
                self._home_position[index]
                + max(
                    self._workspace_delta_limits_m[index][0],
                    min(
                        self._workspace_delta_limits_m[index][1],
                        desired_position[index] - self._home_position[index],
                    ),
                )
                for index in range(3)
            )
            desired_rotation = tuple(
                tuple(float(item) for item in row) for row in rotation
            )  # type: ignore[assignment]
            if len(desired_rotation) != 3 or any(
                len(row) != 3
                or not all(math.isfinite(item) for item in row)
                for row in desired_rotation
            ):
                raise ValueError("rotation must be a finite 3x3 matrix")
            self._desired_position = desired_position
            self._desired_rotation = desired_rotation
            self._joint_goal = None
            self._source = source
            return self._ik_result

    def advance(self, elapsed_s: float) -> IkResult:
        """Move one smooth 60 Hz step toward the latest Cartesian target."""

        dt = max(1.0 / 240.0, min(1.0 / 30.0, float(elapsed_s)))
        with self._lock:
            if self._joint_goal is not None:
                positions, velocities = _smooth_joint_target(
                    self._joint_positions,
                    self._joint_goal,
                    self._joint_velocities,
                    dt,
                    max_velocity_rad_s=self.max_joint_velocity_rad_s,
                    max_acceleration_rad_s2=self.max_joint_acceleration_rad_s2,
                )
                self._joint_positions = self.chain.clamp(positions)
                self._joint_velocities = velocities
                state = self.chain.forward(self._joint_positions)
                self._target_position = state.position
                self._target_rotation = state.rotation
                self._ik_result = IkResult(
                    self._joint_positions,
                    True,
                    0,
                    math.dist(state.position, self._home_position),
                    _rotation_distance(state.rotation, self._home_rotation),
                )
                if self._joint_positions == self._joint_goal:
                    self._joint_goal = None
                return self._ik_result

            next_position = _move_vector_toward(
                self._target_position,
                self._desired_position,
                self.max_target_translation_m_s * dt,
            )
            next_rotation = _move_rotation_toward(
                self._target_rotation,
                self._desired_rotation,
                self.max_target_rotation_rad_s * dt,
            )
            result = self.chain.solve_ik(
                next_position,
                next_rotation,
                self._joint_positions,
            )
            self._ik_result = result
            if not result.converged:
                self._joint_velocities = (0.0,) * len(self.chain.active_joints)
                return result

            positions, velocities = _smooth_joint_target(
                self._joint_positions,
                result.joint_positions,
                self._joint_velocities,
                dt,
                max_velocity_rad_s=self.max_joint_velocity_rad_s,
                max_acceleration_rad_s2=self.max_joint_acceleration_rad_s2,
            )
            self._joint_positions = self.chain.clamp(positions)
            self._joint_velocities = velocities
            self._target_position = next_position
            self._target_rotation = next_rotation
            return result

    def set_joint_positions(self, values: Sequence[float]) -> None:
        with self._lock:
            self._joint_positions = self.chain.clamp(values)
            self._joint_velocities = (0.0,) * len(self.chain.active_joints)
            self._joint_goal = None
            pose = self.chain.forward(self._joint_positions)
            self._target_position = pose.position
            self._target_rotation = pose.rotation
            self._desired_position = pose.position
            self._desired_rotation = pose.rotation
            self._source = "joint_sliders"
            self._ik_result = IkResult(
                self._joint_positions, True, 0, 0.0, 0.0
            )

    def set_gripper(self, value: float) -> None:
        if not self.has_gripper:
            raise ValueError(f"{self.model_name} is configured without a gripper")
        position = float(value)
        if not math.isfinite(position) or not 0.0 <= position <= 1.0:
            raise ValueError("gripper position must be between 0 and 1")
        with self._lock:
            self._gripper_position = position

    def hold_current(self, *, source: str = "clutch_released") -> None:
        """Freeze the simulated target at its current joint state."""
        with self._lock:
            self._joint_velocities = (0.0,) * len(self.chain.active_joints)
            self._joint_goal = None
            pose = self.chain.forward(self._joint_positions)
            self._target_position = pose.position
            self._target_rotation = pose.rotation
            self._desired_position = pose.position
            self._desired_rotation = pose.rotation
            self._source = source

    def update_quest_state(self, value: dict[str, Any]) -> None:
        with self._lock:
            self._quest_state.update(value)
            if self.has_gripper and "gripper_position" in value:
                self._gripper_position = float(value["gripper_position"])

    def joint_state(self) -> dict[str, list[float]]:
        with self._lock:
            return {
                "mode": "single",
                "model_name": self.model_name,
                "display_name": self.display_name,
                "control_dof": (
                    len(self._joint_positions) + int(self.has_gripper)
                ),
                "joint_positions": list(self._joint_positions),
                "joint_velocities": list(self._joint_velocities),
            }

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            auxiliary_joints = (
                {self.native_gripper_joint: self._gripper_position}
                if self.native_gripper_joint
                else None
            )
            state = self.chain.forward(
                self._joint_positions, auxiliary_joints
            )
            return {
                "mode": "single",
                "model_name": self.model_name,
                "display_name": self.display_name,
                "has_gripper": self.has_gripper,
                "native_gripper_visual": bool(self.native_gripper_joint),
                "gripper_mount_link": self.gripper_mount_link,
                "control_dof": (
                    len(self._joint_positions) + int(self.has_gripper)
                ),
                "workspace_delta_limits_m": [
                    list(axis) for axis in self._workspace_delta_limits_m
                ],
                "joint_names": list(self.joint_names),
                "joint_positions": list(self._joint_positions),
                "joint_lower_limits": list(self.chain.lower_limits),
                "joint_upper_limits": list(self.chain.upper_limits),
                "link_points": [list(point) for point in state.link_points],
                "link_transforms": {
                    name: [list(row) for row in transform]
                    for name, transform in state.link_transforms
                },
                "visuals": self.visuals,
                "ee_pose": {
                    "position": list(state.position),
                    "orientation_rpy": list(rpy_from_rotation(state.rotation)),
                    "orientation_quat_xyzw": list(
                        quaternion_xyzw_from_rotation(state.rotation)
                    ),
                },
                "target_pose": {
                    "position": list(self._desired_position),
                    "orientation_rpy": list(
                        rpy_from_rotation(self._desired_rotation)
                    ),
                    "orientation_quat_xyzw": list(
                        quaternion_xyzw_from_rotation(self._desired_rotation)
                    ),
                },
                "tracking_pose": {
                    "position": list(self._target_position),
                    "orientation_rpy": list(
                        rpy_from_rotation(self._target_rotation)
                    ),
                    "orientation_quat_xyzw": list(
                        quaternion_xyzw_from_rotation(self._target_rotation)
                    ),
                },
                "home_pose": {
                    "position": list(self._home_position),
                    "orientation_rpy": list(
                        rpy_from_rotation(self._home_rotation)
                    ),
                    "orientation_quat_xyzw": list(
                        quaternion_xyzw_from_rotation(self._home_rotation)
                    ),
                },
                "ik": {
                    "converged": self._ik_result.converged,
                    "iterations": self._ik_result.iterations,
                    "position_error_m": self._ik_result.position_error_m,
                    "rotation_error_rad": self._ik_result.rotation_error_rad,
                },
                "source": self._source,
                "quest": dict(self._quest_state),
                **(
                    {"gripper_position": self._gripper_position}
                    if self.has_gripper
                    else {}
                ),
            }


class DualArmSimulation:
    """Two configured arms exposed as one configuration-defined vector."""

    def __init__(
        self,
        left: UrdfJointSimulation,
        right: UrdfJointSimulation,
        *,
        display_name: str = "Dual robot arm",
    ) -> None:
        self.left = left
        self.right = right
        self.display_name = str(display_name)
        self._gripper_positions = {"left": 1.0, "right": 1.0}

    def advance(self, elapsed_s: float) -> None:
        self.left.advance(elapsed_s)
        self.right.advance(elapsed_s)

    def home(self, *, source: str = "manual") -> None:
        self.left.home(source=source)
        self.right.home(source=source)

    def set_target_delta(
        self,
        side: str,
        position_delta: Sequence[float],
        rotation_delta_rpy: Sequence[float],
    ) -> IkResult:
        return self._arm(side).set_target_delta(
            position_delta,
            rotation_delta_rpy,
        )

    def set_target_ee_pose(
        self,
        side: str,
        position: Sequence[float],
        orientation_rpy: Sequence[float],
    ) -> IkResult:
        return self._arm(side).set_target_ee_pose(position, orientation_rpy)

    def set_joint_positions(self, values: Sequence[float]) -> None:
        positions = tuple(float(value) for value in values)
        expected = sum(
            len(self._arm(side).chain.active_joints)
            + int(self._arm(side).has_gripper)
            for side in ("left", "right")
        )
        if len(positions) != expected:
            raise ValueError(
                f"dual joint_positions must contain {expected} values, "
                f"got {len(positions)}"
            )
        offset = 0
        for side in ("left", "right"):
            arm = self._arm(side)
            arm_dof = len(arm.chain.active_joints)
            arm.set_joint_positions(positions[offset : offset + arm_dof])
            offset += arm_dof
            if arm.has_gripper:
                self.set_gripper(side, positions[offset])
                offset += 1

    def set_gripper(self, side: str, value: float) -> None:
        if not self._arm(side).has_gripper:
            raise ValueError(f"{side} arm has no gripper")
        position = float(value)
        if not math.isfinite(position) or not 0.0 <= position <= 1.0:
            raise ValueError("gripper position must be between 0 and 1")
        self._gripper_positions[side] = position

    def snapshot(self) -> dict[str, Any]:
        left = self.left.snapshot()
        right = self.right.snapshot()
        arms = {"left": left, "right": right}
        joint_names: list[str] = []
        joint_positions: list[float] = []
        order: list[str] = []
        for side in ("left", "right"):
            arm = self._arm(side)
            snapshot = arms[side]
            joint_names.extend(
                f"{side}_{name}" for name in snapshot["joint_names"]
            )
            joint_positions.extend(snapshot["joint_positions"])
            order.append(f"{side}_joints")
            if arm.has_gripper:
                gripper = self._gripper_position(side, snapshot)
                snapshot["gripper_position"] = gripper
                joint_names.append(f"{side}_gripper")
                joint_positions.append(gripper)
                order.append(f"{side}_gripper")
        return {
            "mode": "dual",
            "display_name": self.display_name,
            "control_dof": len(joint_positions),
            "joint_order": "_".join(order),
            "joint_names": joint_names,
            "joint_positions": joint_positions,
            "left": left,
            "right": right,
        }

    def _gripper_position(self, side: str, snapshot: dict[str, Any]) -> float:
        quest = snapshot["quest"]
        if quest["enabled"] and "gripper_position" in quest:
            return float(quest["gripper_position"])
        return self._gripper_positions[side]

    def _arm(self, side: str) -> UrdfJointSimulation:
        if side == "left":
            return self.left
        if side == "right":
            return self.right
        raise ValueError("side must be left or right")


class SimulationRunner:
    """Advance Cartesian target tracking at a fixed 60 Hz."""

    def __init__(self, simulation: Any) -> None:
        self.simulation = simulation
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop,
            name="meta-quest3-web-sim-motion",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _loop(self) -> None:
        period = 1.0 / SIMULATION_RATE_HZ
        previous = time.monotonic()
        next_tick = previous
        while not self._stop.is_set():
            now = time.monotonic()
            if now < next_tick:
                self._stop.wait(next_tick - now)
                continue
            self.simulation.advance(now - previous)
            previous = now
            next_tick = max(next_tick + period, now)


class QuestSimulationBridge:
    """Apply Quest clutch motion to the simulator's absolute Cartesian target."""

    def __init__(
        self,
        simulation: UrdfJointSimulation,
        *,
        bind_host: str,
        udp_port: int,
        source_ip: str,
        translation_scale: float,
        controller_side: str = "right",
        controller: Any = None,
        manage_controller_lifecycle: bool = True,
    ) -> None:
        self.simulation = simulation
        if controller_side not in {"left", "right"}:
            raise ValueError("controller_side must be left or right")
        self.controller_side = controller_side
        if controller is None:
            if controller_side == "left":
                raise ValueError(
                    "left bridge requires a shared dual-arm controller"
                )
            controller = MetaQuest3RightController(
                bind_host=bind_host,
                udp_port=udp_port,
                source_ip=source_ip,
            )
        self.controller = controller
        self.translation_scale = float(translation_scale)
        self.manage_controller_lifecycle = bool(manage_controller_lifecycle)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._clutch_pressed = False
        self._calibration_count = 0
        self._quest_reference: dict[str, Any] | None = None
        self._target_reference: tuple[tuple[float, ...], Matrix3] | None = None

    def start(self) -> None:
        if self.manage_controller_lifecycle:
            self.controller.start()
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop,
            name="meta-quest3-web-sim-quest",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self.manage_controller_lifecycle:
            self.controller.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _loop(self) -> None:
        while not self._stop.wait(1.0 / 60.0):
            self.update_once()

    def update_once(self) -> None:
        state = self.controller.get_controller_state()
        calibrated = bool(state.get("calibrated"))
        packet_age_s = state.get("packet_age_s")
        packet_fresh = (
            isinstance(packet_age_s, (int, float))
            and packet_age_s <= float(self.controller.stale_timeout_s)
        )
        grip = bool(
            dict(state.get("grip_pressed") or {}).get(self.controller_side)
        ) and packet_fresh
        try:
            if getattr(self.controller, "controller_mode", "") == "dual":
                gripper_position = float(
                    getattr(
                        self.controller,
                        f"get_{self.controller_side}_gripper_state",
                    )()["position"]
                )
            else:
                gripper_position = float(
                    self.controller.get_gripper_state()["position"]
                )
        except RuntimeError:
            gripper_position = None
        calibration_count = int(state.get("calibration_count") or 0)
        quest_state = {
            "enabled": True,
            "calibrated": calibrated,
            "calibration_count": calibration_count,
            "clutch_pressed": grip,
            "packet_count": int(state.get("packet_count") or 0),
            "packet_age_s": packet_age_s,
        }
        if grip and gripper_position is not None:
            quest_state["gripper_position"] = gripper_position
        self.simulation.update_quest_state(quest_state)
        if not calibrated:
            self._clutch_pressed = False
            return
        if calibration_count != self._calibration_count:
            self._calibration_count = calibration_count
            self.simulation.home(source="quest_calibration")
            self._quest_reference = None
            self._target_reference = None
            self._clutch_pressed = False
        if not grip:
            if self._clutch_pressed:
                self.simulation.hold_current()
                self._quest_reference = None
                self._target_reference = None
            self._clutch_pressed = False
            return
        try:
            if getattr(self.controller, "controller_mode", "") == "dual":
                quest_pose = getattr(
                    self.controller,
                    f"get_{self.controller_side}_ee_pose",
                )()
            else:
                quest_pose = self.controller.get_ee_pose()
        except RuntimeError:
            self._clutch_pressed = False
            return
        if not self._clutch_pressed:
            self._quest_reference = quest_pose
            self._target_reference = self.simulation.target_pose
        if self._quest_reference and self._target_reference:
            position_reference, rotation_reference = self._target_reference
            quest_start = self._quest_reference
            position = tuple(
                position_reference[index]
                + self.translation_scale
                * (
                    float(quest_pose["position"][index])
                    - float(quest_start["position"][index])
                )
                for index in range(3)
            )
            current_rotation = quaternion_to_matrix(
                quest_pose["orientation_quat_xyzw"]
            )
            start_rotation = quaternion_to_matrix(
                quest_start["orientation_quat_xyzw"]
            )
            rotation_delta = matrix_multiply(
                current_rotation,
                matrix_transpose(start_rotation),
            )
            rotation = matrix_multiply(
                rotation_delta,
                rotation_reference,
            )
            self.simulation.set_target_absolute(
                position,
                rotation,
                source="quest",
            )
        self._clutch_pressed = True


def _validated_workspace_limits(
    value: Sequence[Sequence[float]],
) -> tuple[tuple[float, float], ...]:
    try:
        limits = tuple(
            (float(axis_limits[0]), float(axis_limits[1]))
            for axis_limits in value
        )
    except (IndexError, TypeError, ValueError) as exc:
        raise ValueError(
            "workspace_delta_limits_m must contain three [minimum, maximum] pairs"
        ) from exc
    if (
        len(limits) != 3
        or any(
            not all(math.isfinite(item) for item in axis_limits)
            or axis_limits[0] > 0.0
            or axis_limits[1] < 0.0
            for axis_limits in limits
        )
    ):
        raise ValueError(
            "workspace_delta_limits_m must contain three finite ranges that include zero"
        )
    return limits


def _positive(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return result


def _move_vector_toward(
    current: Sequence[float],
    target: Sequence[float],
    maximum_distance: float,
) -> tuple[float, float, float]:
    delta = tuple(float(target[index]) - float(current[index]) for index in range(3))
    distance = math.sqrt(sum(item * item for item in delta))
    if distance <= maximum_distance or distance <= 1e-12:
        return tuple(float(item) for item in target)  # type: ignore[return-value]
    scale = maximum_distance / distance
    return tuple(
        float(current[index]) + delta[index] * scale for index in range(3)
    )  # type: ignore[return-value]


def _smooth_joint_target(
    current_positions: Sequence[float],
    target_positions: Sequence[float],
    current_velocities: Sequence[float],
    elapsed_s: float,
    *,
    max_velocity_rad_s: float = MAX_JOINT_VELOCITY_RAD_S,
    max_acceleration_rad_s2: float = MAX_JOINT_ACCELERATION_RAD_S2,
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    positions: list[float] = []
    velocities: list[float] = []
    for current, target, velocity in zip(
        current_positions,
        target_positions,
        current_velocities,
    ):
        error = float(target) - float(current)
        braking_velocity = math.sqrt(
            2.0 * max_acceleration_rad_s2 * abs(error)
        )
        desired_velocity = math.copysign(
            min(max_velocity_rad_s, braking_velocity),
            error,
        )
        velocity_delta = max(
            -max_acceleration_rad_s2 * elapsed_s,
            min(
                max_acceleration_rad_s2 * elapsed_s,
                desired_velocity - float(velocity),
            ),
        )
        next_velocity = float(velocity) + velocity_delta
        step = next_velocity * elapsed_s
        if abs(step) >= abs(error):
            positions.append(float(target))
            velocities.append(0.0)
        else:
            positions.append(float(current) + step)
            velocities.append(next_velocity)
    return tuple(positions), tuple(velocities)


def _move_rotation_toward(
    current: Matrix3,
    target: Matrix3,
    maximum_angle: float,
) -> Matrix3:
    current_q = quaternion_xyzw_from_rotation(current)
    target_q = quaternion_xyzw_from_rotation(target)
    dot = sum(left * right for left, right in zip(current_q, target_q))
    if dot < 0.0:
        target_q = tuple(-item for item in target_q)
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    angle = 2.0 * math.acos(dot)
    if angle <= maximum_angle or angle <= 1e-12:
        return target
    alpha = maximum_angle / angle
    blended = tuple(
        (1.0 - alpha) * current_q[index] + alpha * target_q[index]
        for index in range(4)
    )
    magnitude = math.sqrt(sum(item * item for item in blended))
    return quaternion_to_matrix(tuple(item / magnitude for item in blended))


def _rotation_distance(left: Matrix3, right: Matrix3) -> float:
    left_q = quaternion_xyzw_from_rotation(left)
    right_q = quaternion_xyzw_from_rotation(right)
    dot = abs(sum(a * b for a, b in zip(left_q, right_q)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


class _SimulatorHandler(BaseHTTPRequestHandler):
    simulation: UrdfJointSimulation | DualArmSimulation

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/":
            page_path = (
                DUAL_WEB_PAGE_PATH
                if isinstance(self.simulation, DualArmSimulation)
                else WEB_PAGE_PATH
            )
            self._send(
                HTTPStatus.OK,
                page_path.read_bytes(),
                "text/html; charset=utf-8",
            )
            return
        static_files = {
            "/static/three.min.js": (
                WEB_DIR / "three.min.js",
                "text/javascript; charset=utf-8",
            ),
            "/static/STLLoader.classic.js": (
                WEB_DIR / "STLLoader.classic.js",
                "text/javascript; charset=utf-8",
            ),
            "/static/sim_shared.js": (
                WEB_DIR / "sim_shared.js",
                "text/javascript; charset=utf-8",
            ),
        }
        if self.path in static_files:
            path, content_type = static_files[self.path]
            self._send(HTTPStatus.OK, path.read_bytes(), content_type)
            return
        model_mesh_prefix = "/assets/models/"
        if self.path.startswith(model_mesh_prefix):
            relative_name = self.path.removeprefix(model_mesh_prefix)
            asset_root = MODEL_ASSET_DIR.resolve()
            asset_path = (asset_root / relative_name).resolve()
            if (
                asset_path.is_relative_to(asset_root)
                and asset_path.suffix.lower() == ".stl"
                and asset_path.is_file()
            ):
                self._send(
                    HTTPStatus.OK,
                    asset_path.read_bytes(),
                    "model/stl",
                )
                return
        if self.path == "/api/state":
            self._json(HTTPStatus.OK, self.simulation.snapshot())
            return
        self._json(HTTPStatus.NOT_FOUND, {"error": "Not found"})

    def do_POST(self) -> None:  # noqa: N802
        try:
            payload = self._read_json()
            if self.path == "/api/home":
                self.simulation.home()
            elif self.path == "/api/target":
                ee_pose = payload.get("ee_pose")
                if isinstance(self.simulation, DualArmSimulation):
                    side = str(payload.get("side", ""))
                    if isinstance(ee_pose, Mapping):
                        self.simulation.set_target_ee_pose(
                            side,
                            ee_pose.get("position", ()),
                            ee_pose.get("orientation_rpy", ()),
                        )
                    else:
                        self.simulation.set_target_delta(
                            side,
                            payload.get("position", (0.0, 0.0, 0.0)),
                            payload.get("rotation_rpy", (0.0, 0.0, 0.0)),
                        )
                    if "gripper" in payload:
                        self.simulation.set_gripper(
                            side,
                            float(payload["gripper"]),
                        )
                else:
                    if isinstance(ee_pose, Mapping):
                        self.simulation.set_target_ee_pose(
                            ee_pose.get("position", ()),
                            ee_pose.get("orientation_rpy", ()),
                        )
                    else:
                        self.simulation.set_target_delta(
                            payload.get("position", (0.0, 0.0, 0.0)),
                            payload.get("rotation_rpy", (0.0, 0.0, 0.0)),
                        )
                    if "gripper" in payload:
                        self.simulation.set_gripper(float(payload["gripper"]))
            elif self.path == "/api/joints":
                self.simulation.set_joint_positions(payload["joint_positions"])
            else:
                self._json(HTTPStatus.NOT_FOUND, {"error": "Not found"})
                return
            self._json(HTTPStatus.OK, self.simulation.snapshot())
        except Exception as exc:
            self._json(
                HTTPStatus.BAD_REQUEST,
                {"error": f"{type(exc).__name__}: {exc}"},
            )

    def log_message(self, _format: str, *_args: Any) -> None:
        return

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length") or 0)
        value = json.loads(self.rfile.read(length) or b"{}")
        if not isinstance(value, dict):
            raise ValueError("JSON body must be an object")
        return value

    def _json(self, status: HTTPStatus, value: Any) -> None:
        self._send(
            status,
            json.dumps(value, separators=(",", ":")).encode("utf-8"),
            "application/json",
        )

    def _send(
        self,
        status: HTTPStatus,
        body: bytes,
        content_type: str,
    ) -> None:
        try:
            self.send_response(status.value)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
            # Browsers routinely cancel an in-flight polling request while
            # refreshing, navigating away, or closing the page.
            return


def _load_urdf_visuals(
    urdf_path: Path,
    model_name: str,
) -> list[dict[str, Any]]:
    """Return browser-loadable STL visuals, falling back to collision meshes."""

    root = ET.parse(urdf_path).getroot()
    asset_dir = MODEL_ASSET_DIR / model_name
    asset_names = (
        {
            path.name.lower(): path.name
            for path in asset_dir.iterdir()
            if path.is_file() and path.suffix.lower() == ".stl"
        }
        if asset_dir.is_dir()
        else {}
    )
    visuals: list[dict[str, Any]] = []
    for link in root.findall("link"):
        link_name = str(link.get("name") or "")
        supported = _supported_mesh_elements(
            link.findall("visual"),
            asset_names,
        )
        if not supported:
            supported = _supported_mesh_elements(
                link.findall("collision"),
                asset_names,
            )
        for index, (element, asset_name) in enumerate(supported):
            origin = element.find("origin")
            mesh = element.find("geometry/mesh")
            assert mesh is not None
            origin_xyz = _xml_vector(
                None if origin is None else origin.get("xyz"),
                (0.0, 0.0, 0.0),
            )
            origin_rpy = _xml_vector(
                None if origin is None else origin.get("rpy"),
                (0.0, 0.0, 0.0),
            )
            scale = _xml_vector(
                mesh.get("scale"),
                (1.0, 1.0, 1.0),
            )
            visuals.append(
                {
                    "id": f"{link_name}:{index}",
                    "link_name": link_name,
                    "url": f"/assets/models/{model_name}/{asset_name}",
                    "origin_transform": _visual_transform(
                        origin_xyz,
                        origin_rpy,
                        scale,
                    ),
                }
            )
    return visuals


def _supported_mesh_elements(
    elements: Sequence[ET.Element],
    asset_names: Mapping[str, str],
) -> list[tuple[ET.Element, str]]:
    result: list[tuple[ET.Element, str]] = []
    for element in elements:
        mesh = element.find("geometry/mesh")
        if mesh is None:
            continue
        filename = str(mesh.get("filename") or "")
        asset_name = asset_names.get(Path(filename).name.lower())
        if asset_name is not None:
            result.append((element, asset_name))
    return result


def _xml_vector(
    value: str | None,
    default: tuple[float, float, float],
) -> list[float]:
    if not value:
        return list(default)
    values = [float(item) for item in value.split()]
    if len(values) != 3:
        raise ValueError(f"URDF vector must contain three values: {value}")
    return values


def _visual_transform(
    xyz: Sequence[float],
    rpy: Sequence[float],
    scale: Sequence[float],
) -> list[list[float]]:
    rotation = rotation_from_rpy(rpy)
    return [
        [
            rotation[row][column] * float(scale[column])
            for column in range(3)
        ]
        + [float(xyz[row])]
        for row in range(3)
    ] + [[0.0, 0.0, 0.0, 1.0]]


def load_simulation_config(
    config_path: str | Path,
) -> tuple[
    UrdfJointSimulation | DualArmSimulation,
    dict[str, Any],
]:
    """Build a single- or dual-arm simulator from a YAML configuration."""

    path = Path(config_path).expanduser().resolve()
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, Mapping):
        raise ValueError("simulator config must contain a YAML object")
    components = value.get("components")
    if not isinstance(components, Mapping):
        raise ValueError("simulator config requires components")
    robot = components.get("robot")
    if not isinstance(robot, Mapping):
        raise ValueError("simulator config requires components.robot")
    manifest = value.get("manifest")
    if not isinstance(manifest, Mapping):
        manifest = {}
    display_name = str(
        manifest.get("robot_name") or robot.get("target_model") or path.stem
    )
    arms = robot.get("arms")
    if isinstance(arms, Mapping):
        left = arms.get("left")
        right = arms.get("right")
        if not isinstance(left, Mapping) or not isinstance(right, Mapping):
            raise ValueError("dual-arm config requires arms.left and arms.right")
        simulation: UrdfJointSimulation | DualArmSimulation
        simulation = DualArmSimulation(
            _arm_from_config(left, path.parent),
            _arm_from_config(right, path.parent),
            display_name=display_name,
        )
    else:
        controller_side = str(robot.get("controller_side") or "right")
        if controller_side != "right":
            raise ValueError(
                "single-arm simulator config requires controller_side: right"
            )
        simulation = _arm_from_config(robot, path.parent)
    return simulation, dict(robot)


def _arm_from_config(
    value: Mapping[str, Any],
    config_dir: Path,
) -> UrdfJointSimulation:
    model_name = str(value.get("target_model") or "robot_arm")
    urdf_path = _resolve_model_path(value.get("target_urdf"), config_dir)
    home = value.get("home_joint_positions")
    if not isinstance(home, Sequence) or isinstance(home, (str, bytes)):
        raise ValueError(f"{model_name} requires home_joint_positions")
    if "has_gripper" not in value:
        raise ValueError(f"{model_name} requires has_gripper")
    has_gripper = bool(value["has_gripper"])
    home_includes_gripper = bool(
        value.get("home_includes_gripper", False)
    )
    expected_dof = int(value.get("target_dof") or len(home))
    arm_home = home
    home_gripper_position = value.get("home_gripper_position", 1.0)
    if home_includes_gripper:
        if not has_gripper or len(home) != expected_dof + 1:
            raise ValueError(
                f"{model_name} home_includes_gripper requires "
                "target_dof arm values followed by one gripper value"
            )
        arm_home = home[:-1]
        home_gripper_position = home[-1]
    simulation = UrdfJointSimulation(
        urdf_path,
        base_link=str(value.get("base_link") or ""),
        tip_link=str(value.get("tip_link") or ""),
        home_joint_positions=arm_home,
        workspace_delta_limits_m=value.get(
            "workspace_delta_limits_m", WORKSPACE_DELTA_LIMITS_M
        ),
        max_joint_velocity_rad_s=value.get(
            "max_joint_velocity_rad_s", MAX_JOINT_VELOCITY_RAD_S
        ),
        max_joint_acceleration_rad_s2=value.get(
            "max_joint_acceleration_rad_s2", MAX_JOINT_ACCELERATION_RAD_S2
        ),
        max_target_translation_m_s=value.get(
            "max_target_translation_m_s", MAX_TARGET_TRANSLATION_M_S
        ),
        max_target_rotation_rad_s=value.get(
            "max_target_rotation_rad_s", MAX_TARGET_ROTATION_RAD_S
        ),
        model_name=model_name,
        display_name=str(value.get("display_name") or model_name),
        quest_translation_scale=value.get("translation_scale", 1.0),
        has_gripper=has_gripper,
        home_gripper_position=home_gripper_position,
        native_gripper_joint=value.get("native_gripper_joint"),
        gripper_mount_link=value.get("gripper_mount_link"),
        joint_names=value.get("joint_names"),
    )
    actual_dof = len(simulation.chain.active_joints)
    if actual_dof != expected_dof:
        raise ValueError(
            f"{model_name} target_dof is {expected_dof}, "
            f"but the configured URDF chain has {actual_dof} joints"
        )
    return simulation


def _resolve_model_path(value: Any, config_dir: Path) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValueError("target_urdf must be a path or package URI")
    raw = value.strip()
    package_prefix = "package://rynnrcp_robot_meta_quest3/"
    if raw.startswith(package_prefix):
        path = PACKAGE_DIR / raw.removeprefix(package_prefix)
    else:
        path = Path(raw).expanduser()
        if not path.is_absolute():
            path = config_dir / path
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"URDF file not found: {path}")
    return path


def _quest_controller_kwargs(
    robot_config: Mapping[str, Any],
    args: argparse.Namespace,
) -> dict[str, Any]:
    keys = (
        "stale_timeout_s",
        "gripper_invert",
        "calibration_button",
        "coordinate_basis",
        "rotation_basis",
        "rotation_component_signs",
    )
    result = {key: robot_config[key] for key in keys if key in robot_config}
    result.update(
        {
            "bind_host": args.quest_bind_host
            or str(robot_config.get("bind_host") or "0.0.0.0"),
            "udp_port": (
                args.quest_port
                if args.quest_port is not None
                else int(robot_config.get("udp_port") or 8888)
            ),
            "source_ip": (
                args.quest_source_ip
                if args.quest_source_ip is not None
                else str(robot_config.get("source_ip") or "")
            ),
        }
    )
    return result


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Run the Meta Quest 3 Web simulator with a configured robot URDF."
        )
    )
    parser.add_argument(
        "--config",
        required=True,
        help="Simulator YAML containing the target URDF and arm settings.",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--quest",
        action="store_true",
        help="Listen for Meta Quest 3 controller data on UDP.",
    )
    parser.add_argument("--quest-bind-host")
    parser.add_argument("--quest-port", type=int)
    parser.add_argument("--quest-source-ip")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    simulation, robot_config = load_simulation_config(args.config)
    simulation_runner = SimulationRunner(simulation)
    _SimulatorHandler.simulation = simulation
    server = ThreadingHTTPServer((args.host, args.port), _SimulatorHandler)
    simulation_runner.start()
    quest_bridges: list[QuestSimulationBridge] = []
    dual_controller: MetaQuest3DualController | None = None
    if args.quest:
        controller_kwargs = _quest_controller_kwargs(robot_config, args)
        if isinstance(simulation, DualArmSimulation):
            dual_controller = MetaQuest3DualController(**controller_kwargs)
            dual_controller.start()
            for side in ("left", "right"):
                arm = getattr(simulation, side)
                bridge = QuestSimulationBridge(
                    arm,
                    bind_host=str(controller_kwargs["bind_host"]),
                    udp_port=int(controller_kwargs["udp_port"]),
                    source_ip=str(controller_kwargs["source_ip"]),
                    translation_scale=arm.quest_translation_scale,
                    controller_side=side,
                    controller=dual_controller,
                    manage_controller_lifecycle=False,
                )
                bridge.start()
                quest_bridges.append(bridge)
        else:
            side = str(robot_config.get("controller_side") or "right")
            if side != "right":
                raise ValueError(
                    "single-arm Quest mode requires controller_side: right"
                )
            bridge = QuestSimulationBridge(
                simulation,
                bind_host=str(controller_kwargs["bind_host"]),
                udp_port=int(controller_kwargs["udp_port"]),
                source_ip=str(controller_kwargs["source_ip"]),
                translation_scale=simulation.quest_translation_scale,
                controller_side=side,
                controller=MetaQuest3RightController(**controller_kwargs),
            )
            bridge.start()
            quest_bridges.append(bridge)
    url = f"http://{args.host}:{server.server_address[1]}"
    state = simulation.snapshot()
    print(f"{state['display_name']} Web simulator: {url}")
    print(f"Configuration: {Path(args.config).expanduser().resolve()}")
    if args.quest and isinstance(simulation, DualArmSimulation):
        print(
            "Quest dual mode: hold both controllers still, press X+A to "
            "calibrate, then hold either grip to move that arm."
        )
    elif args.quest:
        print(
            "Quest mode: hold the right controller still, press A to "
            "calibrate, then hold grip to move."
        )
    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        simulation_runner.stop()
        for bridge in quest_bridges:
            bridge.stop()
        if dual_controller is not None:
            dual_controller.shutdown()


if __name__ == "__main__":
    main()
