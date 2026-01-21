"""
RM75 Robot Interface.

Inherits from RobotInterfaceBase to follow the unified interface pattern.
Uses ROS2 topics for communication with real hardware.
For MuJoCo simulation, use MujocoRobotInterface from rcp_motion.core.mjrobot_interface.
"""

import os
import numpy as np
import time
import yaml
import logging
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from rcp_motion.core.robotinterface_base import (
    RobotInterfaceBase,
    register_robotinterface_factory_func,
)
from rcp_motion.core.data.robot_state import RobotState


class RM75Node(Node):
    """ROS2 node for RM75 robot communication."""

    def __init__(self):
        super().__init__("rm75_control_node")
        self.joint_command_pub = self.create_publisher(JointState, "joint_command", 10)
        self.joint_state_fb_sub = self.create_subscription(
            JointState, "joint_states", self._joint_state_callback, 10
        )
        self.connected = False
        self.joint_command = JointState()
        self.joint_command.position = [0.0] * 7
        self.gripper_command = 0.0
        self.joint_state_fb = None
        self.joint_positions = [0.0] * 7
        self.gripper_position = 0.0
        self.state_fb_new = False
        self.connect_count = 0
        self.timer = self.create_timer(0.1, self._timer_callback)

    def _timer_callback(self):
        self.connect_count += 1
        if self.connect_count > 10:
            self.connected = False

    def get_joint_positions(self):
        if self.connected:
            self.state_fb_new = False
            return self.joint_positions + [self.gripper_position]
        return None

    def set_joint_command(self, positions):
        for i in range(7):
            self.joint_command.position[i] = positions[i]
        self.gripper_command = positions[7] if len(positions) > 7 else 0.0
        self.joint_command_pub.publish(self.joint_command)

    def _joint_state_callback(self, msg):
        self.state_fb_new = True
        self.connected = True
        self.connect_count = 0
        self.joint_state_fb = msg
        for i in range(min(7, len(msg.position))):
            self.joint_positions[i] = msg.position[i]


class RM75Model:
    """Lightweight robot model for RM75 hardware interface."""

    def __init__(self, config: Dict):
        self.config = config
        self.joint_names = config.get("joint_names", [])
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


@register_robotinterface_factory_func("rm75_real")
def rm75_real_factory(robot_model, robot_config: dict, logger=None):
    """Factory function for RM75 hardware interface."""
    return RM75Interface(robot_model, robot_config, logger)


class RM75Interface(RobotInterfaceBase):
    """Unified interface for controlling RM75 robot via ROS2."""

    def __init__(self, robot_model, robot_config: Dict, logger=None):
        self._raw_config = robot_config
        self.mode = robot_config.get("mode", "real")

        super().__init__(robot_model, robot_config, logger)

        self.ros_node = None
        self.is_connected = False
        self.start_time = time.time()

        self.joint_names = robot_config.get("joint_names", [
            "rm75_joint1", "rm75_joint2", "rm75_joint3", "rm75_joint4",
            "rm75_joint5", "rm75_joint6", "rm75_joint7", "gripper"
        ])

        self.logger.info(f"Initialized RM75Interface: mode={self.mode}, joints={len(self.joint_names)}")

    def _load_robot_config(self, robot_config):
        self.joint_names = robot_config.get("joint_names", [])
        self.mdof = len(self.joint_names) if self.joint_names else 8
        self.ee_num = 1
        self.gripper_num = 1
        self.site_num = 0
        self.joint_state_num = self.mdof
        self.ft_sensor_num = 0

        self.robot_feedback = RobotState()
        self.robot_feedback.num_joints = self.mdof
        self.robot_feedback.num_end_effectors = self.ee_num
        self.robot_feedback.num_grippers = self.gripper_num

        self.robot_command = RobotState()
        self.robot_command.num_joints = self.mdof

        self.ee_site_name = ["gripper"]
        self.ee_site_index = [0]
        self.gripper_names = ["gripper"]

        self.logger.info(f"RM75 interface: {self.mdof} DOF")

    def connect(self) -> None:
        if self.is_connected:
            self.logger.warning("Already connected")
            return

        try:
            if not rclpy.ok():
                rclpy.init()
            self.ros_node = RM75Node()
            self.is_connected = True
            self.logger.info("Connected to RM75 robot via ROS2")

        except Exception as e:
            self.logger.error(f"Failed to connect to RM75: {e}")
            raise

    def get_robot_state_feedbacks(self) -> RobotState:
        if not self.is_connected:
            raise RuntimeError("Robot is not connected")

        if rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0.0)

        positions = self.ros_node.get_joint_positions()
        if positions:
            for i in range(min(len(positions), self.mdof)):
                self.robot_feedback.joint_pos[i] = positions[i]
                self.robot_feedback.joint_vel[i] = 0.0

        return self.robot_feedback

    def set_robot_command(self, command: RobotState) -> None:
        if not self.is_connected:
            raise RuntimeError("Robot is not connected")

        positions = [command.joint_pos[i] for i in range(self.mdof)]
        if rclpy.ok():
            self.ros_node.set_joint_command(positions)

    def step(self) -> None:
        pass

    def disconnect(self) -> None:
        if not self.is_connected:
            return

        try:
            if self.ros_node:
                self.ros_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            self.is_connected = False
            self.logger.info("Disconnected from RM75")

        except Exception as e:
            self.logger.error(f"Error during disconnect: {e}")

    # ========== Backward Compatibility Methods ==========

    def get_joint_positions(self) -> np.ndarray:
        state = self.get_robot_state_feedbacks()
        return np.array(state.joint_pos[:self.mdof])

    def get_joint_velocities(self) -> np.ndarray:
        return np.zeros(self.mdof)

    def set_joint_positions(self, positions: np.ndarray) -> None:
        command = RobotState()
        command.num_joints = self.mdof
        for i in range(min(len(positions), self.mdof)):
            command.joint_pos[i] = positions[i]
        self.set_robot_command(command)

    def get_current_time(self) -> float:
        return time.time() - self.start_time


# ========== Factory Function ==========

def create_robot_interface(
    name: str = "rm75",
    mode: str = "real",
    config_path: Optional[str] = None,
    **kwargs,
) -> RM75Interface:
    """Factory function to create RM75 interface."""
    config = {}
    if config_path and os.path.exists(config_path):
        with open(config_path, "r") as f:
            config = yaml.safe_load(f) or {}

    config["mode"] = mode
    config["name"] = name

    if "joint_names" not in config:
        config["joint_names"] = [
            "rm75_joint1", "rm75_joint2", "rm75_joint3", "rm75_joint4",
            "rm75_joint5", "rm75_joint6", "rm75_joint7", "gripper"
        ]

    robot_model = RM75Model(config)
    return RM75Interface(robot_model, config, logging.getLogger(__name__))
