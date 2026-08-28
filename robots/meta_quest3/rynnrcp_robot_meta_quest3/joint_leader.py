"""Config-driven Meta Quest 3 leader that emits URDF joint targets."""

from __future__ import annotations

import importlib.util
import logging
import math
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from rynnrcp.robot.base_controller import BaseRobotController

from .controller import (
    QUEST_ROTATION_COMPONENT_SIGNS,
    QUEST_TO_RCP_BASIS,
    QUEST_TO_RCP_ROTATION_BASIS,
    MetaQuest3DualController,
    MetaQuest3RightController,
)
from .web_sim import (
    WORKSPACE_DELTA_LIMITS_M,
    QuestSimulationBridge,
    SimulationRunner,
    UrdfJointSimulation,
)


class MetaQuest3UrdfJointController(BaseRobotController):
    """Publish one arm's joints and its optional normalized gripper position."""

    n_dof = 0

    def __init__(
        self,
        target_model: str,
        target_urdf: str,
        target_dof: int,
        base_link: str,
        tip_link: str,
        home_joint_positions: Sequence[float],
        robot_id: str = "meta_quest3_joint_leader",
        *,
        has_gripper: bool,
        control_dof: int | None = None,
        joint_order: str | None = None,
        controller_side: str = "right",
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
        translation_scale: float = 1.0,
        workspace_delta_limits_m: Sequence[
            Sequence[float]
        ] = WORKSPACE_DELTA_LIMITS_M,
        max_joint_velocity_rad_s: float = 0.25,
        max_joint_acceleration_rad_s2: float = 0.5,
        max_target_translation_m_s: float = 0.15,
        max_target_rotation_rad_s: float = 0.6,
        home_includes_gripper: bool = False,
        native_gripper_joint: str | None = None,
        joint_names: Sequence[str] | None = None,
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__(logger=logger)
        self.robot_id = str(robot_id)
        self.target_model = str(target_model).strip()
        if not self.target_model:
            raise ValueError("target_model must not be empty")
        self.controller_side = str(controller_side).strip()
        if self.controller_side != "right":
            raise ValueError(
                "single-arm mode requires controller_side: right"
            )
        try:
            configured_dof = int(target_dof)
        except (TypeError, ValueError) as exc:
            raise ValueError("target_dof must be a positive integer") from exc
        if configured_dof <= 0:
            raise ValueError("target_dof must be a positive integer")
        configured_home = tuple(float(item) for item in home_joint_positions)
        home_gripper_position = 1.0
        if home_includes_gripper:
            if not has_gripper or len(configured_home) != configured_dof + 1:
                raise ValueError(
                    "home_includes_gripper requires target_dof arm values "
                    "followed by one gripper value"
                )
            configured_home, home_gripper_position = (
                configured_home[:-1],
                configured_home[-1],
            )

        self.quest = MetaQuest3RightController(
            robot_id=robot_id,
            bind_host=bind_host,
            udp_port=udp_port,
            source_ip=source_ip,
            stale_timeout_s=stale_timeout_s,
            gripper_invert=gripper_invert,
            calibration_button=calibration_button,
            coordinate_basis=coordinate_basis,
            rotation_basis=rotation_basis,
            rotation_component_signs=rotation_component_signs,
            logger=logger,
        )
        self.simulation = UrdfJointSimulation(
            _resolve_package_path(target_urdf),
            base_link=str(base_link),
            tip_link=str(tip_link),
            home_joint_positions=configured_home,
            workspace_delta_limits_m=workspace_delta_limits_m,
            max_joint_velocity_rad_s=max_joint_velocity_rad_s,
            max_joint_acceleration_rad_s2=max_joint_acceleration_rad_s2,
            max_target_translation_m_s=max_target_translation_m_s,
            max_target_rotation_rad_s=max_target_rotation_rad_s,
            has_gripper=has_gripper,
            home_gripper_position=home_gripper_position,
            native_gripper_joint=native_gripper_joint,
            joint_names=joint_names,
        )
        actual_dof = len(self.simulation.chain.active_joints)
        if actual_dof != configured_dof:
            raise ValueError(
                f"target_dof is {configured_dof}, but the configured URDF chain "
                f"contains {actual_dof} active joints"
            )
        self.arm_dof = actual_dof
        self.has_gripper = bool(has_gripper)
        self.n_dof = actual_dof + int(self.has_gripper)
        expected_joint_order = (
            "joints_then_gripper" if self.has_gripper else "joints"
        )
        if joint_order is not None and str(joint_order) != expected_joint_order:
            raise ValueError(
                f"joint_order is {joint_order!r}, but the configured output "
                f"requires {expected_joint_order!r}"
            )
        self.joint_order = expected_joint_order
        if control_dof is not None:
            configured_control_dof = _positive_int(
                control_dof, "control_dof"
            )
            if configured_control_dof != self.n_dof:
                raise ValueError(
                    f"control_dof is {configured_control_dof}, but "
                    f"target_dof and has_gripper require {self.n_dof}"
                )
        self.runner = SimulationRunner(self.simulation)
        self.bridge = QuestSimulationBridge(
            self.simulation,
            bind_host=bind_host,
            udp_port=udp_port,
            source_ip=source_ip,
            translation_scale=_positive(
                translation_scale, "translation_scale"
            ),
            controller_side=self.controller_side,
            controller=self.quest,
        )

    def start(self) -> None:
        self.runner.start()
        try:
            self.bridge.start()
        except Exception:
            self.runner.stop()
            raise

    def shutdown(self) -> None:
        self.bridge.stop()
        self.runner.stop()

    def get_joint_positions(self) -> dict[str, list[float]]:
        self.quest.wait_for_active_grip(("right",))
        self.quest.get_ee_pose()
        joints = self.simulation.joint_state()["joint_positions"]
        if not self.has_gripper:
            return {"joint_positions": joints}
        gripper = float(self.simulation.snapshot()["quest"]["gripper_position"])
        return {
            "joint_positions": [*joints, gripper]
        }

    def get_controller_state(self) -> dict[str, Any]:
        state = self.quest.get_controller_state()
        simulation = self.simulation.snapshot()
        state.update(
            {
                "target_model": self.target_model,
                "target_dof": self.arm_dof,
                "control_dof": self.n_dof,
                "joint_order": self.joint_order,
                "controller_side": self.controller_side,
                "ik": simulation["ik"],
                "workspace_source": simulation["source"],
            }
        )
        return state

    def get_health(self) -> dict[str, list[dict[str, Any]]]:
        health = self.quest.get_health()
        state = self.quest.get_controller_state()
        simulation = self.simulation.snapshot()
        if state["calibrated"] and not simulation["ik"]["converged"]:
            health["warnings"].append(
                {
                    "code": "meta_quest3.ik_not_converged",
                    "message": (
                        f"{self.target_model} IK is holding the last valid "
                        "joint target"
                    ),
                    "source": "robot",
                    "timestamp": time.time(),
                    "details": {
                        "robot_id": self.robot_id,
                        "target_model": self.target_model,
                        "target_dof": self.arm_dof,
                        "control_dof": self.n_dof,
                        **simulation["ik"],
                    },
                }
            )
        return health


class MetaQuest3DualUrdfJointController(BaseRobotController):
    """Publish two configured arms and their grippers as one vector."""

    n_dof = 0

    def __init__(
        self,
        arms: Mapping[str, Mapping[str, Any]],
        robot_id: str = "meta_quest3_dual_joint_leader",
        *,
        control_dof: int | None = None,
        joint_order: str | None = None,
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
        if not isinstance(arms, Mapping) or set(arms) != {"left", "right"}:
            raise ValueError("arms must contain exactly left and right mappings")
        self.robot_id = str(robot_id)
        self.quest = MetaQuest3DualController(
            robot_id=robot_id,
            bind_host=bind_host,
            udp_port=udp_port,
            source_ip=source_ip,
            stale_timeout_s=stale_timeout_s,
            gripper_invert=gripper_invert,
            calibration_button=calibration_button,
            coordinate_basis=coordinate_basis,
            rotation_basis=rotation_basis,
            rotation_component_signs=rotation_component_signs,
            logger=logger,
        )
        self.arm_configs = {
            side: dict(arms[side]) for side in ("left", "right")
        }
        self.simulations = {
            side: _simulation_from_config(self.arm_configs[side])
            for side in ("left", "right")
        }
        self.target_models = {
            side: str(self.arm_configs[side]["target_model"])
            for side in ("left", "right")
        }
        self.arm_dofs: dict[str, int] = {}
        self.arm_has_gripper: dict[str, bool] = {}
        for side in ("left", "right"):
            actual_dof = len(self.simulations[side].chain.active_joints)
            configured_dof = _positive_int(
                self.arm_configs[side].get("target_dof"),
                f"arms.{side}.target_dof",
            )
            if actual_dof != configured_dof:
                raise ValueError(
                    f"arms.{side}.target_dof is {configured_dof}, but the "
                    f"configured URDF chain contains {actual_dof} active joints"
                )
            self.arm_dofs[side] = actual_dof
            self.arm_has_gripper[side] = bool(
                self.arm_configs[side]["has_gripper"]
            )
        self.n_dof = sum(
            self.arm_dofs[side] + int(self.arm_has_gripper[side])
            for side in ("left", "right")
        )
        if control_dof is not None:
            configured_control_dof = _positive_int(
                control_dof, "control_dof"
            )
            if configured_control_dof != self.n_dof:
                raise ValueError(
                    f"control_dof is {configured_control_dof}, but the "
                    f"configured arms require {self.n_dof}"
                )
        order = []
        for side in ("left", "right"):
            order.append(f"{side}_joints")
            if self.arm_has_gripper[side]:
                order.append(f"{side}_gripper")
        self.joint_order = "_".join(order)
        if joint_order is not None and str(joint_order) != self.joint_order:
            raise ValueError(
                f"joint_order is {joint_order!r}, but the configured output "
                f"requires {self.joint_order!r}"
            )
        self.runners = {
            side: SimulationRunner(self.simulations[side])
            for side in ("left", "right")
        }
        self.bridges = {
            side: QuestSimulationBridge(
                self.simulations[side],
                bind_host=bind_host,
                udp_port=udp_port,
                source_ip=source_ip,
                translation_scale=_positive(
                    self.arm_configs[side].get("translation_scale", 1.0),
                    f"arms.{side}.translation_scale",
                ),
                controller_side=side,
                controller=self.quest,
                manage_controller_lifecycle=False,
            )
            for side in ("left", "right")
        }
        self._last_output_by_side = {
            side: self._side_output(side) for side in ("left", "right")
        }

    def start(self) -> None:
        self.quest.start()
        try:
            for side in ("left", "right"):
                self.runners[side].start()
                self.bridges[side].start()
        except Exception:
            self.shutdown()
            raise

    def shutdown(self) -> None:
        for side in ("left", "right"):
            self.bridges[side].stop()
            self.runners[side].stop()
        self.quest.shutdown()

    def get_joint_positions(self) -> dict[str, list[float]]:
        grip_pressed = self.quest.wait_for_active_grip(("left", "right"))
        self.quest.get_left_ee_pose()
        self.quest.get_right_ee_pose()
        for side in ("left", "right"):
            if bool(grip_pressed[side]):
                self._last_output_by_side[side] = self._side_output(side)
        values = [
            *self._last_output_by_side["left"],
            *self._last_output_by_side["right"],
        ]
        return {
            "joint_positions": values
        }

    def _side_output(self, side: str) -> list[float]:
        values = list(
            self.simulations[side].joint_state()["joint_positions"]
        )
        if self.arm_has_gripper[side]:
            values.append(
                float(
                    self.simulations[side].snapshot()["quest"][
                        "gripper_position"
                    ]
                )
            )
        return values

    def get_controller_state(self) -> dict[str, Any]:
        state = self.quest.get_controller_state()
        state.update(
            {
                "joint_order": self.joint_order,
                "target_dof": self.n_dof,
                "control_dof": self.n_dof,
                "arms": {
                    side: {
                        "target_model": self.target_models[side],
                        "ik": self.simulations[side].snapshot()["ik"],
                        "workspace_source": self.simulations[side].snapshot()[
                            "source"
                        ],
                    }
                    for side in ("left", "right")
                },
            }
        )
        return state

    def get_health(self) -> dict[str, list[dict[str, Any]]]:
        health = self.quest.get_health()
        state = self.quest.get_controller_state()
        if not state["calibrated"]:
            return health
        for side in ("left", "right"):
            simulation = self.simulations[side].snapshot()
            if simulation["ik"]["converged"]:
                continue
            health["warnings"].append(
                {
                    "code": f"meta_quest3.{side}_ik_not_converged",
                    "message": (
                        f"{side} {self.target_models[side]} IK is holding "
                        "the last valid joint target"
                    ),
                    "source": "robot",
                    "timestamp": time.time(),
                    "details": {
                        "robot_id": self.robot_id,
                        "side": side,
                        "target_model": self.target_models[side],
                        **simulation["ik"],
                    },
                }
            )
        return health


def _simulation_from_config(
    config: Mapping[str, Any],
) -> UrdfJointSimulation:
    required = (
        "target_model",
        "target_urdf",
        "target_dof",
        "base_link",
        "tip_link",
        "home_joint_positions",
        "workspace_delta_limits_m",
        "has_gripper",
    )
    missing = [name for name in required if name not in config]
    if missing:
        raise ValueError(
            "arm configuration is missing: " + ", ".join(missing)
        )
    home = tuple(float(item) for item in config["home_joint_positions"])
    has_gripper = bool(config["has_gripper"])
    target_dof = int(config["target_dof"])
    home_gripper_position = config.get("home_gripper_position", 1.0)
    if bool(config.get("home_includes_gripper", False)):
        if not has_gripper or len(home) != target_dof + 1:
            raise ValueError(
                "home_includes_gripper requires target_dof arm values "
                "followed by one gripper value"
            )
        home, home_gripper_position = home[:-1], home[-1]
    return UrdfJointSimulation(
        _resolve_package_path(str(config["target_urdf"])),
        base_link=str(config["base_link"]),
        tip_link=str(config["tip_link"]),
        home_joint_positions=home,
        workspace_delta_limits_m=config["workspace_delta_limits_m"],
        max_joint_velocity_rad_s=config.get(
            "max_joint_velocity_rad_s", 0.25
        ),
        max_joint_acceleration_rad_s2=config.get(
            "max_joint_acceleration_rad_s2", 0.5
        ),
        max_target_translation_m_s=config.get(
            "max_target_translation_m_s", 0.15
        ),
        max_target_rotation_rad_s=config.get(
            "max_target_rotation_rad_s", 0.6
        ),
        has_gripper=has_gripper,
        home_gripper_position=home_gripper_position,
        native_gripper_joint=config.get("native_gripper_joint"),
        joint_names=config.get("joint_names"),
    )


def _resolve_package_path(value: str) -> Path:
    raw = str(value).strip()
    prefix = "package://"
    if raw.startswith(prefix):
        package_ref = raw[len(prefix) :]
        package_name, separator, relative_path = package_ref.partition("/")
        if not separator or not package_name or not relative_path:
            raise ValueError(
                "target_urdf package URI must include a package and file path"
            )
        spec = importlib.util.find_spec(package_name)
        if spec is None:
            raise ValueError(f"target_urdf package was not found: {package_name}")
        locations = list(spec.submodule_search_locations or ())
        package_dir = (
            Path(locations[0])
            if locations
            else Path(str(spec.origin)).resolve().parent
        )
        path = package_dir / relative_path
    else:
        path = Path(raw).expanduser()
        if not path.is_absolute():
            raise ValueError(
                "target_urdf must be an absolute path or package:// URI"
            )
    path = path.resolve()
    if not path.is_file():
        raise ValueError(f"target_urdf file was not found: {path}")
    return path


def _positive(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return result


def _positive_int(value: Any, name: str) -> int:
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a positive integer") from exc
    if result <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return result
