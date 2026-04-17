# rcp_motion/core/base_robot_interface.py
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Sequence


class BaseRobotInterface(ABC):
    """Abstract base robot interface.

    Expected methods:
      - init()
      - disconnect()
      - get_joint_positions()
      - set_joint_positions(positions)
    """

    @abstractmethod
    def init(self, *args, **kwargs) -> None:
        """Initialize / connect the robot."""
        raise NotImplementedError

    @abstractmethod
    def disconnect(self, *args, **kwargs) -> None:
        """Disconnect / cleanup."""
        raise NotImplementedError

    @abstractmethod
    def get_joint_positions(self, *args, **kwargs) -> List[float]:
        """Return current joint positions."""
        raise NotImplementedError

    @abstractmethod
    def set_joint_positions(self, positions: Sequence[float], *args, **kwargs) -> None:
        """Send joint position command."""
        raise NotImplementedError
