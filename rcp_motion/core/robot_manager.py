"""
PRobotManager - Python Robot Manager (Minimal Version)

Naming convention: PRobotManager (Python) vs RobotManager (C++)
- Easy to search and navigate between C++ and Python implementations
- P prefix indicates Python version

Supports: fr3, ur5e, piper, rm75, so101
Methods match C++ API:
- getPinoMJCF() - Pinocchio MJCF path
- getRobotMJCF() - Robot MJCF path (for MuJoCo simulation)
- getRobotNumber()
- getRobotName()
"""

from abc import ABC, abstractmethod
from pathlib import Path
from typing import Dict
import numpy as np
import logging

from rcp_motion.core.mjcf_parser import MjcfParser
# PinKine removed - not used in RobotManager


class RobotManager(ABC):
    """
    Python Robot Manager - Minimal hardcoded version

    Usage:
        robot = PRobotManager(robotmodel_config, logger)
        pino_path = robot.getPinoMJCF()
        robot_path = robot.getRobotMJCF()
    """

    def __init__(self, robotmodel_config: dict, logger):
        """
        Initialize PRobotManager

        Args:
            robot_identifier: Robot name (str) or number (int)
                Examples: "fr3", "piper", 20, 22

        Raises:
            ValueError: If robot name/number is not supported
        """
        self.logger = logger or logging.getLogger(__name__)

        self._robot_name = robotmodel_config.get("robot_name", "")
        self._robot_control_freq = robotmodel_config.get("robot_control_freq", 100)
        # Load paths
        self._robot_mjcf = robotmodel_config.get("robot_mjcf", {})
        self._pino_mjcf = robotmodel_config.get("pino_mjcf", {})

        self.logger.info("robot name: %s", self._robot_name)
        self.logger.info("robot mjcf: %s", self._robot_mjcf)
        self.logger.info("pino mjcf: %s", self._pino_mjcf)

        self._robotstate_massage = self._load_robotstate_massage()

        # Load keyframes from Pinocchio MJCF
        self._keyframes = self._load_keyframes()

        self.logger.info("Load robot manager sucessful!\n")

    def _load_robotstate_massage(self) -> dict:
        if not self._robot_mjcf:
            return {
                "joint_dim": 0,
                "actuator_dim": 0,
                "ee_num": 0,
                "gripper_dim": 0,
                "ft_sensor_num": 0,
                "site_num": 0,
                "gripper_joint_idx": [],
                "gripper_joint_name": [],
                "ee_site_idx": [],
                "ee_site_name": [],
            }

        try:
            robotstate_dim = MjcfParser.extractRobotStateDim(self._robot_mjcf)
            return robotstate_dim
        except Exception as e:
            print(
                f"Warning: Failed to extract robotstate dim from {self._robot_mjcf}: {e}"
            )
            return {
                "joint_dim": 0,
                "actuator_dim": 0,
                "ee_num": 0,
                "gripper_dim": 0,
                "ft_sensor_num": 0,
                "site_num": 0,
                "gripper_joint_idx": [],
                "gripper_joint_name": [],
                "ee_site_idx": [],
                "ee_site_name": [],
            }

    def _load_keyframes(self) -> dict:
        """Load keyframes (home, stand) from Pinocchio MJCF"""
        if not self._pino_mjcf:
            return {"home": None, "stand": None, "nq": 0}

        try:
            keyframes = MjcfParser.extractKeyframes(self._pino_mjcf)
            return keyframes
        except Exception as e:
            print(f"Warning: Failed to load keyframes from {self._pino_mjcf}: {e}")
            return {"home": None, "stand": None, "nq": 0}

    # ========== Getters (match C++ API) ==========

    def getRobotMJCF(self) -> str:
        """Get robot MJCF path (C++ API style)"""
        return self._robot_mjcf

    def getPinoMJCF(self) -> str:
        """Get Pinocchio MJCF path (C++ API style)"""
        return self._pino_mjcf

    def getRobotName(self) -> str:
        """Get robot name (C++ API style)"""
        return self._robot_name

    def getActuatorNum(self) -> int:
        """Get robot number (C++ API style)"""
        return self._actuator_num

    def get_qKeyFrame(self, index: int) -> np.ndarray:
        """
        Get keyframe by index (new indexed API)

        Universal index mapping:
        - 0: home
        - 1: standby1
        - 2: standby2
        - 3+: extra keyframes (e.g., SO101's "rest")

        Args:
            index: Keyframe index

        Returns:
            Keyframe position as numpy array (nq,)
            Returns zeros if index out of range
        """
        if "keyframes" in self._keyframes and index < len(self._keyframes["keyframes"]):
            return self._keyframes["keyframes"][index]
        return np.zeros(self._keyframes["nq"])

    def get_qStandby(self, side: str) -> np.ndarray:
        """
        Get standby keyframe for specified arm side

        Args:
            side: "left" or "right"

        Returns:
            Standby position for the specified side
            - "left" returns standby1 (index 1)
            - "right" returns standby2 (index 2)
        """
        if side == "left":
            return self.get_qKeyFrame(1)  # standby1
        elif side == "right":
            return self.get_qKeyFrame(2)  # standby2
        else:
            # Default to standby1 if invalid side
            return self.get_qKeyFrame(1)

    def get_num_keyframes(self) -> int:
        """Get total number of keyframes"""
        if "keyframes" in self._keyframes:
            return len(self._keyframes["keyframes"])
        return 0

    def get_qHome(self) -> np.ndarray:
        """
        Get home keyframe configuration (C++ API style)

        Returns:
            Home position as numpy array (nq,)
            Returns zeros if keyframe not found
        """
        # Backward compatibility: home is index 0
        return self.get_qKeyFrame(0)

    def get_qStand(self) -> np.ndarray:
        """
        Get stand keyframe configuration (C++ API style)

        Returns:
            Stand position as numpy array (nq,)
            Returns zeros if keyframe not found
        """
        # Backward compatibility: stand maps to standby1 (index 1)
        return self.get_qKeyFrame(1)

    def get_robot_control_freq(self):
        return self._robot_control_freq

    def get_actuator_num(self) -> int:
        return self._robotstate_massage["actuator_dim"]

    def get_joint_state_num(self) -> int:
        return self._robotstate_massage["joint_dim"]

    def get_gripper_num(self) -> int:
        return self._robotstate_massage["gripper_dim"]

    def get_ee_num(self) -> int:
        return self._robotstate_massage["ee_num"]

    def get_site_num(self) -> int:
        return self._robotstate_massage["site_num"]

    def get_ft_sensor_num(self) -> int:
        return self._robotstate_massage["ft_sensor_num"]

    def get_gripper_joint_index(self) -> int:
        return self._robotstate_massage["gripper_joint_idx"]

    def get_gripper_joint_name(self):
        return self._robotstate_massage["gripper_joint_name"]

    def get_ee_site_index(self) -> int:
        return self._robotstate_massage["ee_site_idx"]

    def get_ee_site_name(self):
        return self._robotstate_massage["ee_site_name"]

    # ========== Python-style getters (optional aliases) ==========

    def get_robot_mjcf(self) -> str:
        """Get robot MJCF path (Python style)"""
        return self.getRobotMJCF()

    def get_pino_mjcf(self) -> str:
        """Get Pinocchio MJCF path (Python style)"""
        return self.getPinoMJCF()

    def get_robot_name(self) -> str:
        """Get robot name (Python style)"""
        return self.getRobotName()
