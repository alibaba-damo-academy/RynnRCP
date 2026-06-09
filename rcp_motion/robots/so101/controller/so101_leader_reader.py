#!/usr/bin/env python3
"""
SO101 Leader Reader - Lightweight leader arm state reader.

Only reads joint positions from the leader (teleop) arm hardware.
No LCM, no interpolator, no plotting — just connect and read.

Usage in config YAML:
    module_name: controller.so101_leader_reader.SO101LeaderReader
"""

import logging
import numpy as np
import yaml
from pathlib import Path

from rcp_motion.robots.so101.interface.so101_interface import create_robot_interface

logger = logging.getLogger(__name__)


def _default_config_path(robot_name: str) -> Path:
    import rcp_motion
    return Path(rcp_motion.__file__).parent / "robots" / "so101" / "configs" / f"{robot_name}.yaml"


class SO101LeaderReader:
    """Read-only joint state provider for SO101 leader arm."""

    def __init__(self, robot_name: str = "so101", config_path: str = None):
        self.robot_name = robot_name
        self.config_path = config_path or str(_default_config_path(robot_name))
        self._interface = None
        self._load_config()

    def _load_config(self):
        try:
            with open(self.config_path, "r", encoding="utf-8") as f:
                self.config = yaml.safe_load(f) or {}
        except FileNotFoundError:
            logger.warning(f"Config not found: {self.config_path}, using defaults")
            self.config = {}

    def start(self):
        """Connect to the leader arm hardware."""
        if self._interface is not None:
            logger.warning("Already started")
            return

        self._interface = create_robot_interface(
            name=self.robot_name,
            mode="teleop",
            config_path=self.config_path,
        )
        self._interface.connect()
        self._interface.disable_robot_torque()
        logger.info("SO101LeaderReader started")

    def get_joint_positions(self) -> np.ndarray:
        """Return current joint positions (radians + gripper ratio)."""
        if self._interface is None:
            raise RuntimeError("Leader not started, call start() first")
    
        joint_positions = self._interface.get_joint_positions()
    
        return joint_positions

    def stop(self):
        """Disconnect from the leader arm hardware."""
        if self._interface is not None:
            self._interface.disconnect()
            self._interface = None
            logger.info("SO101LeaderReader stopped")
