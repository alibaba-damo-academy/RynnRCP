"""
Unified Robot Interface.

This module provides a single interface that can handle all robot types:
- Real hardware robots using ros2
- Mock robots for testing
- MuJoCo simulation
"""

import os
import sys
import numpy as np
import time
import yaml
import json
import logging
from typing import Dict, Optional

from interface.base_interface import BaseRobotInterface
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import WrenchStamped  # geometry_msgs/msg/WrenchStamped


class FrankaNode(Node):
    def __init__(self):
        super().__init__("franka_control_node")
        self.joint_command_pub = self.create_publisher(
            JointState,
            "franka_driver/joint_cmd",
            10,  # "teleop_controller/desired_joint"
        )
        self.gripper_command_pub = self.create_publisher(
            JointState,
            "franka_driver/gripper_cmd",
            10,  # "teleop_controller/desired_gripper"
        )
        self.joint_state_fb_sub = self.create_subscription(
            JointState,
            "franka_driver/joint_states",
            self.joint_state_fb_callback,
            10,  # "joint_states"
        )
        self.ft_sensor_state_fb_sub = self.create_subscription(
            WrenchStamped,
            "franka_driver/ee_ftsensor",
            self.ft_sensor_state_fb_callback,
            10,  # "franka_robot_state_broadcaster/external_wrench_in_base_frame"
        )
        self.connected = False
        self.joint_command = JointState()
        self.joint_command.position = [0.0] * 7
        self.gripper_command = JointState()
        self.gripper_command.position = [0.0]
        # self.gripper_command = 0
        self.joint_state_fb = None
        self.ft_sensor_fb = None
        self.joint_position_state = [0.0] * 7
        self.gripper_position = 0.0
        self.ft_sensor_state = [0.0] * 6
        self.state_fb_new = False
        self.state_fb_count = 0
        self.connect_count = 0
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # self.get_logger().info("Timer callback:" + str(self.connect_count))
        self.connect_count += 1
        if self.connect_count > 100:
            self.connected = False

    def get_joint_position_state(self):
        if self.connected:
            # print("get position:",self.joint_position_state + [self.gripper_position])
            self.state_fb_new = False
            joint_position = self.joint_position_state + [self.gripper_position]
            return joint_position
        else:
            return None

    def get_ftsensor_state(self):
        if self.connected:
            return self.ft_sensor_state
        else:
            # return self.ft_sensor_state
            return None

    def set_joint_command(self, joint_position_command):
        for i in range(7):
            self.joint_command.position[i] = joint_position_command[i]
        if joint_position_command[7] > 0.5:
            self.gripper_command.position[0] = 1
        else:
            self.gripper_command.position[0] = 0
        # self.gripper_command = joint_position_command[7]
        self.joint_command_pub.publish(self.joint_command)
        self.gripper_command_pub.publish(self.gripper_command)

    def joint_state_fb_callback(self, msg):
        self.state_fb_new = True
        self.connected = True
        self.connect_count = 0
        self.joint_state_fb = msg
        for i in range(7):
            self.joint_position_state[i] = self.joint_state_fb.position[i]
        # print("joint state feedback 1 is", self.joint_state_fb.position)

    def ft_sensor_state_fb_callback(self, msg):
        self.ft_sensor_fb = msg
        self.ft_sensor_state[0] = self.ft_sensor_fb.wrench.force.x
        self.ft_sensor_state[1] = self.ft_sensor_fb.wrench.force.y
        self.ft_sensor_state[2] = self.ft_sensor_fb.wrench.force.z
        self.ft_sensor_state[3] = self.ft_sensor_fb.wrench.torque.x
        self.ft_sensor_state[4] = self.ft_sensor_fb.wrench.torque.y
        self.ft_sensor_state[5] = self.ft_sensor_fb.wrench.torque.z


class FrankaInterface(BaseRobotInterface):
    """
    Unified interface for controlling robots in different modes.

    Supports:
    - franka robots using ros2 topic
    """

    def __init__(
        self,
        config: Optional[Dict] = None,
        scene: int = 1,
        **kwargs,
    ):
        """
        Initialize the unified robot interface.

        Args:
            config: Configuration dictionary or None for defaults
            scene: scene path number for simulation (default: 1)
            **kwargs: Additional arguments for specific modes
        """
        super().__init__()

        self.scene = scene
        self.config = config or self._get_default_config()
        self.logger = logging.getLogger(__name__)
        self.robot_config = self.config.get("robot", {})
        self.joint_names = self.robot_config.get("joint_names", {})
        self.start_time = time.time()

        self.logger.info(
            f"Initialized franka interface with {len(self.joint_names)} joints"
        )

    def _get_default_config(self) -> Dict:
        """Get default configuration."""
        return {
            "robot": {
                "joint_names": [
                    "fr3_joint1",
                    "fr3_joint2",
                    "fr3_joint3",
                    "fr3_joint4",
                    "fr3_joint5",
                    "fr3_joint6",
                    "fr3_joint7",
                ],
            },
            "simulation": {
                "scene_path": {1: "models/franka/franka_fr3/scene/scene.xml"},
                "random_init_pose": False,
            },
        }

    def init(self, timestep: float = 0.01) -> None:
        """Initialize the robot/simulator with specified timestep."""
        if self.is_connected:
            self.logger.warning("Already connected")
            return

        self.timestep = timestep

        try:
            self._init_real_robot()
            self.is_connected = True
            self.logger.info(f"Initialized franka robot with timestep {timestep}s")

        except Exception as e:
            self.logger.error(f"Failed to initialize franka robot: {e}")
            raise

    def _init_real_robot(self):
        """Initialize real robot using Feetech motors."""
        motor_config = self.config.get("motors", {})
        robot_config = self.config.get("robot", {})

        rclpy.init()
        self.franka_node = FrankaNode()
        # rclpy.spin(franka_node)

    def _set_home_pose(self):
        """Set home pose for mujoco."""
        try:
            home_qpos = np.zeros(len(self.joint_names))
            home_ctrl = np.zeros(len(self.joint_names))

            self.logger.info("initial pose buy home qpos set successfully")

        except Exception as e:
            self.logger.error(f"Failed to set random pose: {e}")

    def disconnect(self) -> None:
        """Disconnect from the robot/simulator."""
        if not self.is_connected:
            return

        # TODO: disconnect real robot
        try:
            self.franka_node.destroy_node()
            rclpy.shutdown()
            self.is_connected = False
            self.logger.info(f"Disconnected from franka")

        except Exception as e:
            self.logger.error(f"Error during disconnect: {e}")

    def get_joint_positions(self) -> np.ndarray:
        """Get current joint positions."""
        if not self.is_connected:
            raise RuntimeError("Robot is not connected")

        try:
            # get real robot position
            positions = None
            if rclpy.ok():
                rclpy.spin_once(self.franka_node, timeout_sec=0.000)  # don't block
                positions = self.franka_node.get_joint_position_state()
            current_time = time.time() - self.start_time
            return positions

        except Exception as e:
            self.logger.error(f"Failed to read joint positions: {e}")
            import traceback

            self.logger.error(f"Traceback: {traceback.format_exc()}")
            return np.zeros(len(self.joint_names))

    def get_ftsensor_state(self) -> np.ndarray:
        if not self.is_connected:
            raise RuntimeError("Robot is not connected")
        try:
            # get real robot ftsensor state
            ftsensor_state = None
            if rclpy.ok():
                ftsensor_state = self.franka_node.get_ftsensor_state()
            return ftsensor_state
        except Exception as e:
            self.logger.error(f"Failed to read ftsensor state: {e}")
            return np.zeros(6)

    def get_joint_velocities(self) -> np.ndarray:
        return np.zeros(len(self.joint_names))

    def set_joint_positions(self, positions: np.ndarray, radians: bool = True) -> None:
        """Send joint position commands to the robot."""
        if not self.is_connected:
            raise RuntimeError("Robot is not connected")

        try:
            # Log original command positions for debugging with time
            positions_copy = positions.copy()

            # Send command to real robot
            if rclpy.ok():
                self.franka_node.set_joint_command(positions_copy)

            # self.franka_node.set_joint_command(positions_copy)

        except Exception as e:
            self.logger.error(f"Failed to write joint positions: {e}")
            raise

    def enable_torque(self, enable: bool = True) -> None:
        """Enable or disable motor torque."""
        if not self.is_connected:
            raise RuntimeError("Robot is not connected")

        pass

    def step(self) -> None:
        """Step the robot/simulation forward."""
        pass

    def reset(self) -> None:
        """Reset the robot to initial state."""
        pass

    def get_current_time(self) -> float:
        """
        Get the current time appropriate for this interface mode.

        Returns:
            float: system time
        """
        return time.time() - self.start_time


def create_franka_interface(
    config_path: Optional[str] = None, scene: int = 1, **kwargs
) -> FrankaInterface:
    """
    Factory function to create a robot interface.

    Args:
        config_path: Path to configuration file (optional)
        scene: scene path number for simulation (default: 1)
        **kwargs: Additional arguments

    Returns:
        UnifiedRobotInterface instance

    Raises:
        FileNotFoundError: If calibration file is required but missing
    """
    config = None
    if config_path and os.path.exists(config_path):
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

    return FrankaInterface(config=config, scene=scene, **kwargs)
