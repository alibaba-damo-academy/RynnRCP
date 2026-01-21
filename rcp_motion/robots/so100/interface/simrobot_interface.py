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
import random
from typing import Dict, Optional

from interface.base_interface import BaseRobotInterface

import mujoco
import mujoco.viewer


class SimulationRobotInterface(BaseRobotInterface):
    """
    Unified interface for controlling simulation robots.

    Supports:
    - MuJoCo simulation scene
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

        self.model = None
        self.data = None
        self.viewer = None

        self.render_freq = kwargs.get("render_freq", 60)
        self.render_interval = 1.0 / self.render_freq
        self.last_render_time = 0
        self.start_time = time.time()

        self.camera = None
        self.logger.info(
            f"Initialized simulation robot interface with {len(self.joint_names)} joints"
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
        """Initialize the simulator with specified timestep."""
        if self.is_connected:
            self.logger.warning("Already connected")
            return

        self.timestep = timestep

        try:
            self._init_mujoco()
            self.is_connected = True
            self.logger.info(f"Initialized simulation robot with timestep {timestep}s")

        except Exception as e:
            self.logger.error(f"Failed to initialize simulation robot: {e}")
            raise

    def _init_mujoco(self):
        """Initialize MuJoCo simulation with custom timestep."""
        sim_config = self.config.get("simulation", {})
        if self.scene not in sim_config.get("scene_path", {}):
            raise ValueError(
                f"Scene {self.scene} not found in simulation configuration"
            )
        mjcf_path = sim_config.get("scene_path")[self.scene]

        if not os.path.exists(mjcf_path):
            # script_dir = os.path.dirname(os.path.abspath(__file__))
            # project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
            project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            mjcf_path = os.path.join(project_root, mjcf_path)

        if not os.path.exists(mjcf_path):
            raise FileNotFoundError(f"MuJoCo model file not found: {mjcf_path}")

        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.model.opt.timestep = self.timestep
        self.logger.info(f"Set MuJoCo timestep to: {self.model.opt.timestep}")
        self.data = mujoco.MjData(self.model)

        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)
        self.init_camera()
        self._set_home_pose()

    def _set_home_pose(self):
        """Set home pose for mujoco."""
        try:
            home_qpos = self.model.keyframe("home").qpos
            home_ctrl = self.model.keyframe("home").ctrl
            if home_qpos is None:
                home_qpos = self.data.qpos
            if home_ctrl is None:
                home_ctrl = self.data.ctrl

            self.data.qpos[:] = home_qpos
            self.data.qvel[:] = 0
            self.data.qacc[:] = 0
            self.data.ctrl[:] = home_ctrl
            mujoco.mj_forward(self.model, self.data)
            self.logger.info("initial pose buy home qpos set successfully")

        except Exception as e:
            self.logger.error(f"Failed to set random pose: {e}")
            mujoco.mj_resetData(self.model, self.data)

    def disconnect(self) -> None:
        """Disconnect from the robot/simulator."""
        if not self.is_connected:
            return

        # TODO: disconnect real robot
        try:
            if self.viewer:
                self.viewer.close()
                self.viewer = None

            self.is_connected = False
            self.logger.info(f"Disconnected from simulation robot")

        except Exception as e:
            self.logger.error(f"Error during disconnect: {e}")

    def get_joint_positions(self) -> np.ndarray:
        """Get current joint positions."""
        if not self.is_connected:
            raise RuntimeError("Robot is not connected")

        try:
            num_joints = len(self.joint_names)
            positions = self.data.qpos[:num_joints].copy()

            # Use simulation time for MuJoCo
            sim_time = self.data.time
            self.logger.debug(
                f"time = {sim_time:.3f}s, mujoco qPos Feedback {np.array2string(positions, precision=3, suppress_small=True, floatmode='fixed')}"
            )
            return positions

        except Exception as e:
            self.logger.error(f"Failed to read joint positions: {e}")
            import traceback

            self.logger.error(f"Traceback: {traceback.format_exc()}")
            return np.zeros(len(self.joint_names))

    def get_joint_velocities(self) -> np.ndarray:
        vels = self.data.qvel.copy()

        # Use simulation time for MuJoCo
        sim_time = self.data.time
        self.logger.debug(
            f"time = {sim_time:.3f}s, mujoco qvel Feedback {np.array2string(vels, precision=3, suppress_small=True, floatmode='fixed')}"
        )
        return vels

        return np.zeros(len(self.joint_names))

    def get_ftsensor_state(self) -> np.ndarray:
        # TODO: get ftsensor state in mujoco
        return np.zeros(6)

    def set_joint_positions(self, positions: np.ndarray, radians: bool = True) -> None:
        """Send joint position commands to the robot."""
        if not self.is_connected:
            raise RuntimeError("Robot is not connected")

        try:
            # Log original command positions for debugging with time
            positions_copy = positions.copy()
            mode_name = "mujoco"

            # Get appropriate time based on mode
            current_time = self.data.time

            if radians:
                self.logger.debug(
                    f"time = {current_time:.3f}s, {mode_name} qPos Command {np.array2string(positions_copy, precision=3, suppress_small=True, floatmode='fixed')}"
                )
            else:
                self.logger.debug(
                    f"time = {current_time:.3f}s, {mode_name} qPos Command (deg) {np.array2string(positions_copy, precision=3, suppress_small=True, floatmode='fixed')}"
                )

            if not radians:
                positions_copy[0 : len(self.joint_names) - 1] *= np.pi / 180.0

            num_joints = min(len(positions), len(self.joint_names))
            self.data.ctrl[:num_joints] = positions_copy[:num_joints]

        except Exception as e:
            self.logger.error(f"Failed to write joint positions: {e}")
            raise

    def enable_torque(self, enable: bool = True) -> None:
        """Enable or disable motor torque."""
        pass

    def step(self) -> None:
        """Step the robot/simulation forward."""
        mujoco.mj_step(self.model, self.data)

        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.last_render_time = self.data.time

        if self.viewer.is_running():
            current_time = self.data.time
            if current_time - self.last_render_time >= self.render_interval:
                # if self.camera is not None:
                # with self.viewer.lock():
                #    self.viewer.cam.distance = self.camera.distance
                #    self.viewer.cam.azimuth = self.camera.azimuth
                #    self.viewer.cam.elevation = self.camera.elevation
                #    self.viewer.cam.lookat[:] = self.camera.lookat
                self.viewer.sync()
                self.last_render_time = current_time

    def reset(self) -> None:
        """Reset the robot to initial state."""
        if self.model and self.data:
            mujoco.mj_resetData(self.model, self.data)

    def is_viewer_alive(self) -> bool:
        """Check if the viewer is still alive."""
        if self.viewer:
            return self.viewer.is_running()
        return True

    def init_camera(self):
        """Setup camera position and orientation for simulation."""
        if self.camera is not None:
            self.camera.distance = 1.0
            self.camera.azimuth = 45.0
            self.camera.elevation = -20.0
            self.camera.lookat = np.array([0.0, -0.2, 0.2])

    def set_camera(self, distance=None, azimuth=None, elevation=None, lookat=None):
        """
        Set camera parameters for custom viewing (simulation only).

        Args:
            distance: Distance from lookat point
            azimuth: Horizontal rotation around lookat point (degrees)
            elevation: Vertical rotation (degrees)
            lookat: 3D point to look at [x, y, z]
        """
        if self.camera is not None:
            if distance is not None:
                self.camera.distance = distance
            if azimuth is not None:
                self.camera.azimuth = azimuth
            if elevation is not None:
                self.camera.elevation = elevation
            if lookat is not None:
                self.camera.lookat = np.array(lookat)

    def get_current_time(self) -> float:
        """
        Get the current time appropriate for this interface mode.

        Returns:
            float: Simulation time for sim mode, system time for real/mock modes
        """
        if self.data is not None:
            return self.data.time
        else:
            return time.time() - self.start_time


def create_sim_robot_interface(
    config_path: Optional[str] = None, scene: int = 1, **kwargs
) -> SimulationRobotInterface:
    """
    Factory function to create a simulation robot interface.

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

    return SimulationRobotInterface(config=config, scene=scene, **kwargs)
