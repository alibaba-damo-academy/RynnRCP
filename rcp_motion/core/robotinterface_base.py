#!/usr/bin/env python3
"""
Base Unified Controller - Abstract base class for all robot controllers

This module provides the common control logic and infrastructure that all robot
implementations (LeRobot, Lekiwi, etc.) can inherit from.
"""

from abc import ABC, abstractmethod
from collections import OrderedDict
import logging

from rcp_motion.core.data.robot_state import RobotState

REGISTERED_ROBOT_INTERFACE_FACTORY_FUNCS = OrderedDict()


def register_robotinterface_factory_func(robot_type):
    """
    Function decorator to register algo generator functions that map generator class names.
    Each generator implements such a function, and decorates it with this decorator.

    Args:
        generator_name (str): the algorithm name to register the algorithm under
    """

    def decorator(factory_func):
        REGISTERED_ROBOT_INTERFACE_FACTORY_FUNCS[robot_type] = factory_func

    return decorator


def robotinterface_factory(robot_type, robot_model, robot_config, logger=None):
    """
    Factory function for creating trajectory generator based on the generator name and config.

    Args:
        generator_name (str): the generator name

        config (BaseConfig instance): config object
    """
    robotinterface_class = REGISTERED_ROBOT_INTERFACE_FACTORY_FUNCS[robot_type]

    # create algo instance
    return robotinterface_class(
        robot_model=robot_model, robot_config=robot_config, logger=logger
    )


class RobotInterfaceBase(ABC):
    """
    robot interface base class
    """

    def __init__(self, robot_model, robot_config, logger):
        self.logger = logger or logging.getLogger(__name__)
        self.robot_model = robot_model
        self.connected = False
        self._load_robot_config(robot_config)

    def _load_robot_config(self, robot_config):
        """load robot config"""

        self.mdof = self.robot_model.get_actuator_num()
        self.ee_num = self.robot_model.get_ee_num()
        self.gripper_num = self.robot_model.get_gripper_num()
        self.site_num = self.robot_model.get_site_num()
        self.joint_state_num = self.robot_model.get_joint_state_num()
        self.ft_sensor_num = self.robot_model.get_ft_sensor_num()

        self.robot_feedback = RobotState()
        self.robot_feedback.num_joints = self.mdof
        self.robot_feedback.num_end_effectors = self.ee_num
        self.ee_site_name = self.robot_model.get_ee_site_name()
        self.ee_site_index = self.robot_model.get_ee_site_index()
        self.robot_feedback.num_grippers = self.gripper_num
        self.gripper_names = self.robot_model.get_gripper_joint_name()
        self.robot_feedback.num_sites = self.site_num
        self.robot_feedback.num_ft_sensors = self.ft_sensor_num

        self.robot_command = RobotState()
        self.robot_command.num_joints = self.mdof
        self.robot_feedback.num_end_effectors = self.ee_num
        self.robot_feedback.num_grippers = self.gripper_num
        self.robot_feedback.num_sites = self.site_num

        self.logger.info(f"robot interface initialized:")
        self.logger.info(f"mdof: {self.mdof}")
        self.logger.info(f"ee_num: {self.ee_num}")
        self.logger.info(f"ee_site_name: {self.ee_site_name}")
        self.logger.info(f"gripper_num: {self.gripper_num}")
        self.logger.info(f"gripper_names: {self.gripper_names}")
        self.logger.info(f"site_num: {self.site_num}")
        self.logger.info(f"joint_state_num: {self.joint_state_num}")
        self.logger.info(f"ft_sensor_num: {self.ft_sensor_num}")

    @abstractmethod
    def connect(self):
        """connect to robot."""
        pass

    @abstractmethod
    def get_robot_state_feedbacks(self) -> RobotState:
        """get robot state feedbacks."""
        pass

    @abstractmethod
    def set_robot_command(self, command: RobotState) -> None:
        """set robot command."""
        pass

    @abstractmethod
    def step(self) -> None:
        pass

    @abstractmethod
    def disconnect(self) -> None:
        pass

    def is_connected(self) -> bool:
        """Check if robot interface is connected."""
        return self.connected

    def get_robot_state_without_update(self) -> RobotState:
        """get robot state feedbacks without update."""
        return self.robot_feedback.copy()
