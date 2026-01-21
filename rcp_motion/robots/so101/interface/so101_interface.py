"""
Unified Robot Interface for SO101 Hardware.

This module provides a single interface that can handle all robot types:
- Real hardware robots using Feetech motors
- Mock robots for testing
- Teleop mode for leader robot control

Inherits from RobotInterfaceBase to follow the unified interface pattern.
For MuJoCo simulation, use MujocoRobotInterface from rcp_motion.core.mjrobot_interface.
"""

import os
import numpy as np
import time
import yaml
import logging
from typing import Dict, Optional
from pathlib import Path

from rcp_motion.robots.so101.hardware.robots.so101_follower.config_so101_follower import (
    SO101FollowerConfig,
)
from rcp_motion.robots.so101.hardware.robots.so101_follower.so101_follower import SO101Follower
from rcp_motion.robots.so101.hardware.robots.lekiwi.config_lekiwi import LeKiwiConfig
from rcp_motion.robots.so101.hardware.robots.lekiwi.lekiwi import LeKiwi

from rcp_motion.robots.so101.hardware.teleoperators.so101_leader.config_so101_leader import (
    SO101LeaderConfig,
)
from rcp_motion.robots.so101.hardware.teleoperators.so101_leader.so101_leader import SO101Leader

from rcp_motion.core.robotinterface_base import (
    RobotInterfaceBase,
    register_robotinterface_factory_func,
)
from rcp_motion.core.data.robot_state import RobotState


# Default joint names for SO101 (with .pos suffix for hardware API)
DEFAULT_JOINT_NAMES = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
]


class MockMotorsBus:
    """Mock motor bus for testing without real hardware."""

    def __init__(self, config):
        self.config = config
        self.is_connected = False
        self.motors = config.motors
        self._positions = np.zeros(len(self.motors))
        self._torque_enabled = False

    def connect(self):
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False

    def write(self, data_name: str, values, motor_names=None):
        if data_name == "Goal_Position":
            if hasattr(values, "__len__"):
                self._positions[: len(values)] = values
            else:
                if motor_names and isinstance(motor_names, str):
                    motor_list = list(self.motors.keys())
                    if motor_names in motor_list:
                        idx = motor_list.index(motor_names)
                        if idx < len(self._positions):
                            self._positions[idx] = values
        elif data_name == "Torque_Enable":
            self._torque_enabled = (
                bool(values) if not hasattr(values, "__len__") else any(values)
            )

    def read(self, data_name: str, motor_names=None):
        if data_name == "Present_Position":
            if motor_names is None:
                return self._positions.copy()
            elif isinstance(motor_names, list):
                return self._positions[: len(motor_names)].copy()
            else:
                return self._positions[0]
        elif data_name == "Moving":
            if motor_names is None:
                return np.zeros(len(self._positions), dtype=bool)
            elif isinstance(motor_names, list):
                return np.zeros(len(motor_names), dtype=bool)
            else:
                return False
        elif data_name == "Torque_Enable":
            if motor_names is None:
                return np.ones(len(self._positions), dtype=int) * int(
                    self._torque_enabled
                )
            elif isinstance(motor_names, list):
                return np.ones(len(motor_names), dtype=int) * int(self._torque_enabled)
            else:
                return int(self._torque_enabled)
        else:
            if motor_names is None:
                return np.zeros(len(self._positions))
            elif isinstance(motor_names, list):
                return np.zeros(len(motor_names))
            else:
                return 0.0

    def set_calibration(self, calibration):
        pass


class MockMotorsBusConfig:
    """Mock motor bus configuration."""

    def __init__(self, port: str, motors: dict, mock: bool = True):
        self.port = port
        self.motors = motors
        self.mock = mock


class LeRobotModel:
    """Lightweight robot model for LeRobot hardware interface."""

    def __init__(self, config: Dict):
        self.config = config
        self.joint_names = config.get("joint_names", DEFAULT_JOINT_NAMES)
        self._control_freq = config.get("control_freq", 100)

    def get_actuator_num(self) -> int:
        return len(self.joint_names)

    def get_ee_num(self) -> int:
        return 1

    def get_gripper_num(self) -> int:
        return 1

    def get_site_num(self) -> int:
        return 0

    def get_joint_state_num(self) -> int:
        return len(self.joint_names)

    def get_ft_sensor_num(self) -> int:
        return 0

    def get_ee_site_name(self) -> list:
        return ["gripper"]

    def get_ee_site_index(self) -> list:
        return [0]

    def get_gripper_joint_name(self) -> list:
        if self.joint_names:
            return [self.joint_names[-1]]
        return []

    def get_robot_control_freq(self) -> int:
        return self._control_freq


@register_robotinterface_factory_func("lerobot_real")
def lerobot_real_factory(robot_model, robot_config: dict, logger=None):
    """Factory function for LeRobot hardware interface."""
    return LeRobotInterface(robot_model, robot_config, logger)


class LeRobotInterface(RobotInterfaceBase):
    """Unified interface for controlling LeRobot hardware in different modes."""

    def __init__(
        self,
        robot_model,
        robot_config: Dict,
        logger=None,
    ):
        self._raw_config = robot_config
        self.mode = robot_config.get("mode", "mock")
        self.name = robot_config.get("name", "lerobot")

        super().__init__(robot_model, robot_config, logger)

        self.robot = None
        self.motor_bus = None
        self.is_connected = False
        self.start_time = time.time()

        self.joint_names = robot_config.get("joint_names", DEFAULT_JOINT_NAMES)
        self._init_joint_range()

        self.logger.info(
            f"Initialized LeRobotInterface: mode={self.mode}, joints={len(self.joint_names)}"
        )

    def _load_robot_config(self, robot_config):
        self.joint_names = robot_config.get("joint_names", DEFAULT_JOINT_NAMES)
        self.mdof = len(self.joint_names)
        self.ee_num = 1
        self.gripper_num = 1
        self.site_num = 0
        self.joint_state_num = self.mdof
        self.ft_sensor_num = 0

        self.robot_feedback = RobotState()
        self.robot_feedback.num_joints = self.mdof
        self.robot_feedback.num_end_effectors = self.ee_num
        self.robot_feedback.num_grippers = self.gripper_num
        self.robot_feedback.num_sites = self.site_num
        self.robot_feedback.num_ft_sensors = self.ft_sensor_num

        self.robot_command = RobotState()
        self.robot_command.num_joints = self.mdof

        self.ee_site_name = ["gripper"]
        self.ee_site_index = [0]
        self.gripper_names = [self.joint_names[-1]] if self.joint_names else []

        self.logger.info(f"LeRobot interface: {self.mdof} DOF")

    def _init_joint_range(self):
        self.joint_range = {}
        joint_range_config = self._raw_config.get("joint_range")
        if joint_range_config is not None:
            for i, name in enumerate(self.joint_names):
                if (i + 1) in joint_range_config:
                    self.joint_range[name] = joint_range_config[i + 1]

    def connect(self) -> None:
        if self.is_connected:
            self.logger.warning("Already connected")
            return

        try:
            if self.mode == "real":
                self._init_real_robot()
            elif self.mode == "mock":
                self._init_mock_robot()
            elif self.mode == "teleop":
                self._init_teleop_robot()

            self.is_connected = True
            self.logger.info(f"Connected to {self.mode} robot")

        except Exception as e:
            self.logger.error(f"Failed to connect to {self.mode} robot: {e}")
            raise

    def _init_real_robot(self):
        robot_config = self._raw_config.get("robot", {})

        port = robot_config.get("port", "/dev/ttyACM0")
        calibration_dir = robot_config.get("calibration_dir")

        if calibration_dir is not None:
            calibration_dir = Path(calibration_dir).expanduser()

        robot_id = calibration_dir.name if calibration_dir else "so101_follower"

        self.logger.info(f"Initializing real robot on port: {port}")

        if self.name in ["lerobot", "so101"]:
            real_robot_config = SO101FollowerConfig(
                port=port, calibration_dir=calibration_dir, id=robot_id
            )
            self.robot = SO101Follower(real_robot_config)
        elif self.name == "lekiwi":
            real_robot_config = LeKiwiConfig(port=port)
            self.robot = LeKiwi(real_robot_config)

        self.robot.connect()

    def _init_mock_robot(self):
        robot_config = self._raw_config.get("robot", {})

        port = robot_config.get("port", "/dev/ttyACM0")
        motor_model = robot_config.get("model", "sts3215")

        self.logger.info(f"Initializing mock robot on port: {port}")

        motors = {}
        for i, joint_name in enumerate(self.joint_names):
            motors[joint_name] = (i + 1, motor_model)

        config = MockMotorsBusConfig(port=port, motors=motors, mock=True)
        self.motor_bus = MockMotorsBus(config)
        self.motor_bus.connect()

    def _init_teleop_robot(self):
        teleop_config = self._raw_config.get("teleoperate", {})

        port = teleop_config.get("port", "/dev/ttyACM1")
        calibration_dir = teleop_config.get("calibration_dir")

        if calibration_dir is not None:
            calibration_dir = Path(calibration_dir).expanduser()

        teleop_id = calibration_dir.name if calibration_dir else "so101_leader"

        self.logger.info(f"Initializing teleop robot on port: {port}")

        if self.name in ["lerobot", "so101"]:
            teleop_robot_config = SO101LeaderConfig(
                port=port, calibration_dir=calibration_dir, id=teleop_id
            )
            self.robot = SO101Leader(teleop_robot_config)
        elif self.name == "lekiwi":
            teleop_robot_config = SO101LeaderConfig(
                port=port, calibration_dir=calibration_dir, id=teleop_id
            )
            self.robot = SO101Leader(teleop_robot_config)

        self.robot.connect()

    def get_robot_state_feedbacks(self) -> RobotState:
        if not self.is_connected:
            raise RuntimeError("Robot is not connected")

        positions = self._read_joint_positions()

        for i in range(min(len(positions), self.mdof)):
            self.robot_feedback.joint_pos[i] = positions[i]
            self.robot_feedback.joint_vel[i] = 0.0

        return self.robot_feedback

    def _read_joint_positions(self) -> np.ndarray:
        try:
            if self.mode == "real" and self.robot is not None:
                positions_dict = self.robot.get_observation()
                positions = np.array(
                    [positions_dict[name] for name in self.joint_names]
                )
                positions[:-1] *= np.pi / 180.0
                return positions

            elif self.mode == "mock" and self.motor_bus is not None:
                positions = self.motor_bus.read("Present_Position", self.joint_names)
                positions[:-1] *= np.pi / 180.0
                return positions

            elif self.mode == "teleop" and self.robot is not None:
                positions_dict = self.robot.get_action()
                positions = np.array(
                    [positions_dict[name] for name in self.joint_names]
                )
                positions[:-1] *= np.pi / 180.0
                return positions

            return np.zeros(len(self.joint_names))

        except Exception as e:
            self.logger.error(f"Failed to read joint positions: {e}")
            return np.zeros(len(self.joint_names))

    def set_robot_command(self, command: RobotState) -> None:
        if not self.is_connected:
            raise RuntimeError("Robot is not connected")

        positions = np.array(command.joint_pos[:self.mdof])
        self._write_joint_positions(positions)

    def _write_joint_positions(self, positions: np.ndarray) -> None:
        try:
            positions_copy = positions.copy()

            if self.mode == "real" and self.robot is not None:
                positions_copy[:-1] *= 180.0 / np.pi

                data = {}
                for i, name in enumerate(self.joint_names):
                    if name in self.joint_range:
                        data[name] = np.clip(
                            positions_copy[i],
                            self.joint_range[name][0],
                            self.joint_range[name][1],
                        )
                    else:
                        data[name] = positions_copy[i]

                self.robot.send_action(data)

            elif self.mode == "mock" and self.motor_bus is not None:
                num_joints = min(len(positions), len(self.joint_names))
                self.motor_bus.write(
                    "Goal_Position",
                    positions_copy[:num_joints],
                    self.joint_names[:num_joints],
                )

        except Exception as e:
            self.logger.error(f"Failed to write joint positions: {e}")
            raise

    def step(self) -> None:
        pass

    def disconnect(self) -> None:
        if not self.is_connected:
            return

        try:
            if self.mode == "real" and self.robot:
                if self.robot.is_connected:
                    self.robot.disconnect()
            elif self.mode == "mock" and self.motor_bus:
                if self.motor_bus.is_connected:
                    self.motor_bus.disconnect()
            elif self.mode == "teleop" and self.robot:
                if self.robot.is_connected:
                    self.robot.disconnect()

            self.is_connected = False
            self.logger.info(f"Disconnected from {self.mode} robot")

        except Exception as e:
            self.logger.error(f"Error during disconnect: {e}")

    def disable_robot_torque(self) -> None:
        if not self.is_connected:
            return
        try:
            if self.robot:
                self.robot.disable_robot_torque()
        except Exception as e:
            self.logger.error(f"Error disabling torque: {e}")

    # ========== Backward Compatibility Methods ==========

    def init(self, timestep: float = 0.01) -> None:
        self.timestep = timestep
        self.connect()

    def get_joint_positions(self) -> np.ndarray:
        state = self.get_robot_state_feedbacks()
        return np.array(state.joint_pos[:self.mdof])

    def get_joint_velocities(self) -> np.ndarray:
        return np.zeros(len(self.joint_names))

    def set_joint_positions(self, positions: np.ndarray) -> None:
        command = RobotState()
        command.num_joints = self.mdof
        for i in range(min(len(positions), self.mdof)):
            command.joint_pos[i] = positions[i]
        self.set_robot_command(command)

    def get_current_time(self) -> float:
        return time.time() - self.start_time


# ========== Factory Function (Backward Compatibility) ==========

def create_robot_interface(
    name: str = "lerobot",
    mode: str = "mock",
    config_path: Optional[str] = None,
    port: Optional[str] = None,
    calibration_dir: Optional[str] = None,
    **kwargs,
) -> LeRobotInterface:
    """Factory function to create a LeRobot hardware interface."""
    config = {}
    if config_path and os.path.exists(config_path):
        with open(config_path, "r") as f:
            config = yaml.safe_load(f) or {}

    config["mode"] = mode
    config["name"] = name

    if port is not None:
        if mode == "teleop":
            if "teleoperate" not in config:
                config["teleoperate"] = {}
            config["teleoperate"]["port"] = port
        else:
            if "robot" not in config:
                config["robot"] = {}
            config["robot"]["port"] = port

    if calibration_dir is not None:
        if mode == "teleop":
            if "teleoperate" not in config:
                config["teleoperate"] = {}
            config["teleoperate"]["calibration_dir"] = calibration_dir
        else:
            if "robot" not in config:
                config["robot"] = {}
            config["robot"]["calibration_dir"] = calibration_dir

    robot_model = LeRobotModel(config)

    return LeRobotInterface(robot_model, config, logging.getLogger(__name__))
