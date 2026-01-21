"""
Abstract Base Class for Robot Interfaces.

This module provides a unified interface for controlling different types of robots:
- Real hardware robots
- Mock robots for testing
- Simulated robots (MuJoCo, etc.)
"""

from abc import ABC, abstractmethod
import numpy as np
from typing import List, Optional


class BaseRobotInterface(ABC):
    """
    Abstract base class for robot interfaces.

    This class defines the common interface that all robot implementations
    (real, mock, simulation) must follow.
    """

    def __init__(self):
        self.is_connected = False
        self.joint_names = []
        self.timestep = 0.01

    @abstractmethod
    def init(self, timestep: float = 0.01) -> None:
        """Initialize the robot/simulator default timestep."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the robot/simulator."""
        pass

    @abstractmethod
    def get_joint_positions(self, radians: bool = True) -> np.ndarray:
        """
        Get current joint positions.

        Args:
            radians: If True, return positions in radians, otherwise in degrees

        Returns:
            Array of joint positions
        """
        pass

    @abstractmethod
    def set_joint_positions(self, positions: np.ndarray, radians: bool = True) -> None:
        """
        Send joint position commands to the robot.

        Args:
            positions: Array of joint positions
            radians: If True, positions are in radians, otherwise in degrees
        """
        pass

    def get_joint_position(self, joint_name: str, radians: bool = True) -> float:
        """
        Get position of a specific joint.

        Args:
            joint_name: Name of the joint
            radians: If True, return position in radians, otherwise in degrees

        Returns:
            Joint position
        """
        if joint_name not in self.joint_names:
            raise ValueError(f"Unknown joint name: {joint_name}")

        positions = self.get_joint_positions(radians=radians)
        joint_idx = self.joint_names.index(joint_name)
        return positions[joint_idx]

    def set_joint_position(
        self, joint_name: str, position: float, radians: bool = True
    ) -> None:
        """
        Send position command to a specific joint.

        Args:
            joint_name: Name of the joint
            position: Joint position
            radians: If True, position is in radians, otherwise in degrees
        """
        if joint_name not in self.joint_names:
            raise ValueError(f"Unknown joint name: {joint_name}")

        current_positions = self.get_joint_positions(radians=radians)
        joint_idx = self.joint_names.index(joint_name)
        current_positions[joint_idx] = position
        self.set_joint_positions(current_positions, radians=radians)

    @abstractmethod
    def enable_torque(self, enable: bool = True) -> None:
        """Enable or disable motor torque."""
        pass

    def get_joint_names(self) -> List[str]:
        """Get list of joint names."""
        return self.joint_names.copy()

    def get_num_joints(self) -> int:
        """Get number of joints."""
        return len(self.joint_names)

    def is_robot_connected(self) -> bool:
        """Check if robot is connected."""
        return self.is_connected

    def step(self) -> None:
        """
        Step the robot/simulation forward.
        Default implementation does nothing (for real robots).
        """
        pass

    def reset(self) -> None:
        """
        Reset the robot to initial state.
        Default implementation does nothing.
        """
        pass

    def init_camera(self):
        """
        Setup camera for viewing (mujoco only).
        Default implementation does nothing.
        """
        pass

    def set_camera(self, distance=None, azimuth=None, elevation=None, lookat=None):
        """
        Set camera parameters for custom viewing (simulation only).

        Args:
            distance: Distance from lookat point
            azimuth: Horizontal rotation around lookat point (degrees)
            elevation: Vertical rotation (degrees)
            lookat: 3D point to look at [x, y, z]

        Default implementation does nothing.
        """
        pass

    def __enter__(self):
        """Context manager entry."""
        self.init()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
