"""
RCP Motion Core - Shared base classes for robot interfaces

This module provides the core abstractions that all robot implementations
(so101, franka, realman, etc.) can inherit from or instantiate.
"""

from rcp_motion.core.robotinterface_base import (
    RobotInterfaceBase,
    register_robotinterface_factory_func,
    REGISTERED_ROBOT_INTERFACE_FACTORY_FUNCS,
)
from rcp_motion.core.mjrobot_interface import MujocoRobotInterface
from rcp_motion.core.robot_manager import RobotManager

__all__ = [
    "RobotInterfaceBase",
    "MujocoRobotInterface",
    "RobotManager",
    "register_robotinterface_factory_func",
    "REGISTERED_ROBOT_INTERFACE_FACTORY_FUNCS",
]
