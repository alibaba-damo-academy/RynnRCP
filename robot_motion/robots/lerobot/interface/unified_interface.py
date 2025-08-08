"""
Unified Robot Interface.

This module provides a single interface that can handle all robot types:
- Real hardware robots using Feetech motors
- Mock robots for testing
- MuJoCo simulation
"""

import os
import sys
import numpy as np
import time
import yaml
import json
import tempfile
import logging
import random
from typing import List, Dict, Any, Optional

from interface.base_interface import BaseRobotInterface
from robot_devices.motors.feetech import FeetechMotorsBus
from robot_devices.motors.configs import FeetechMotorsBusConfig

import mujoco
import mujoco.viewer


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
        """Mock calibration method."""
        pass


class MockMotorsBusConfig:
    """Mock motor bus configuration."""

    def __init__(self, port: str, motors: dict, mock: bool = True):
        self.port = port
        self.motors = motors
        self.mock = mock


class UnifiedRobotInterface(BaseRobotInterface):
    """
    Unified interface for controlling robots in different modes.

    Supports:
    - Real hardware robots using Feetech motors
    - Mock robots for testing
    - MuJoCo simulation
    """

    def __init__(
        self,
        mode: str = "mock",
        config: Optional[Dict] = None,
        scene: int = 1,
        **kwargs,
    ):
        """
        Initialize the unified robot interface.

        Args:
            mode: "real", "mock", or "sim"
            config: Configuration dictionary or None for defaults
            scene: scene path number for simulation (default: 1)
            **kwargs: Additional arguments for specific modes
        """
        super().__init__()

        self.mode = mode
        self.scene = scene
        self.config = config or self._get_default_config()
        self.logger = logging.getLogger(__name__)
        self.joint_names = self.config.get(
            "joint_names",
            [
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
                "gripper",
            ],
        )

        if self.mode == "sim":
            self.invert_indices = [0, 1, 4]
        else:
            self.invert_indices = self.config.get("invert_indices", [])

        self.motor_bus = None
        self.model = None
        self.data = None
        self.viewer = None
        self.cube_initialized = False
        self.calibration_file = kwargs.get("calibration_file", None)

        self.render_freq = kwargs.get("render_freq", 60)
        self.render_interval = 1.0 / self.render_freq
        self.last_render_time = 0
        self.start_time = time.time()

        self.camera = None

        self.logger.info(
            f"Initialized {mode} robot interface with {len(self.joint_names)} joints"
        )
        if self.invert_indices:
            self.logger.info(f"Invert indices: {self.invert_indices}")

    def _get_default_config(self) -> Dict:
        """Get default configuration."""
        return {
            "joint_names": [
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
                "gripper",
            ],
            "motors": {
                "port": "/dev/ttyACM0",
                "baudrate": 1000000,
                "model": "sts3215",
            },
            "simulation": {
                "scene_path": {1: "models/lerobot/so101/scene/scene.xml"},
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
            if self.mode == "real":
                self._init_real_robot()
            elif self.mode == "mock":
                self._init_mock_robot()
            elif self.mode == "sim":
                self._init_mujoco()

            self.is_connected = True
            self.logger.info(f"Initialized {self.mode} robot with timestep {timestep}s")

        except Exception as e:
            self.logger.error(f"Failed to initialize {self.mode} robot: {e}")
            raise

    def _init_real_robot(self):
        """Initialize real robot using Feetech motors."""
        motor_config = self.config.get("motors", {})
        robot_config = self.config.get("robot", {})

        port = robot_config.get("port", motor_config.get("port", "/dev/ttyACM0"))
        self.logger.info(f"Initializing real robot on port: {port}")

        motors = {}
        for i, joint_name in enumerate(self.joint_names):
            motors[joint_name] = (i + 1, "sts3215")

        config = FeetechMotorsBusConfig(port=port, motors=motors, mock=False)
        self.motor_bus = FeetechMotorsBus(config)
        self.motor_bus.connect()

        if self.calibration_file:
            if os.path.exists(self.calibration_file):
                with open(self.calibration_file) as f:
                    calibration = json.load(f)
                    self.motor_bus.set_calibration(calibration)
                    self.logger.info(f"Loaded calibration from {self.calibration_file}")
            else:
                error_msg = (
                    f"❌ CALIBRATION FILE NOT FOUND: {self.calibration_file}\n"
                    f"   This is required for real robot operation.\n"
                    f"   Please run calibration first:\n"
                    f"   python -m scripts.follower_calibrate --mode manual"
                )
                self.logger.error(error_msg)
                raise FileNotFoundError(error_msg)
        else:
            self.logger.warning(
                "⚠️  No calibration file specified for real robot. "
                "Robot may not function correctly without calibration."
            )

    def _init_mock_robot(self):
        """Initialize mock robot."""
        motor_config = self.config.get("motors", {})
        robot_config = self.config.get("robot", {})

        port = robot_config.get("port", motor_config.get("port", "/dev/ttyACM0"))
        self.logger.info(f"Initializing mock robot on port: {port}")

        motors = {}
        for i, joint_name in enumerate(self.joint_names):
            motors[joint_name] = (i + 1, "sts3215")

        config = MockMotorsBusConfig(port=port, motors=motors, mock=True)
        self.motor_bus = MockMotorsBus(config)
        self.motor_bus.connect()

    def _init_mujoco(self):
        """Initialize MuJoCo simulation with custom timestep."""
        sim_config = self.config.get("simulation", {})
        if self.scene not in sim_config.get("scene_path", {}):
            raise ValueError(
                f"Scene {self.scene} not found in simulation configuration"
            )
        mjcf_path = sim_config.get("scene_path")[self.scene]

        if not os.path.exists(mjcf_path):
            script_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
            mjcf_path = os.path.join(project_root, mjcf_path)

        if not os.path.exists(mjcf_path):
            raise FileNotFoundError(f"MuJoCo model file not found: {mjcf_path}")

        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.model.opt.timestep = self.timestep
        self.logger.info(f"Set MuJoCo timestep to: {self.model.opt.timestep}")
        self.data = mujoco.MjData(self.model)



        random_init_pose = sim_config.get("random_init_pose", False)
        if random_init_pose:
            self._set_random_pose()

        if self.scene == 2:
            self.randomize_cube_pos()

        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)
        self.init_camera()

    def _set_random_pose(self):
        """Set random initial pose for mujoco."""
        try:
            q = np.empty(self.model.nq)
            q[:] = self.data.qpos[:]

            for j in range(self.model.njnt):
                if self.model.jnt_limited[j]:
                    low = (
                        2.0 * self.model.jnt_range[j, 0]
                        + 1.0 * self.model.jnt_range[j, 1]
                    ) / 3.0
                    high = (
                        1.0 * self.model.jnt_range[j, 0]
                        + 2.0 * self.model.jnt_range[j, 1]
                    ) / 3.0
                    qpos_addr = self.model.jnt_qposadr[j]
                    q[qpos_addr] = np.random.uniform(low, high)
                    self.logger.debug(
                        f"Joint {j}: random pos {q[qpos_addr]:.3f} in range [{low:.3f}, {high:.3f}]"
                    )

            self.data.qpos[:] = q
            self.data.qvel[:] = 0
            self.data.qacc[:] = 0
            self.data.ctrl[:] = q
            mujoco.mj_forward(self.model, self.data)
            self.logger.info("✓ Random initial pose set successfully")

        except Exception as e:
            self.logger.error(f"Failed to set random pose: {e}")
            # If random pose setting fails, continue with default pose
            mujoco.mj_resetData(self.model, self.data)

    def disconnect(self) -> None:
        """Disconnect from the robot/simulator."""
        if not self.is_connected:
            return

        try:
            if self.mode in ["real", "mock"] and self.motor_bus:
                if self.motor_bus.is_connected:
                    self.enable_torque(False)
                    self.motor_bus.disconnect()
            elif self.mode == "sim":
                if self.viewer:
                    self.viewer.close()
                    self.viewer = None

            self.is_connected = False
            self.logger.info(f"Disconnected from {self.mode} robot")

        except Exception as e:
            self.logger.error(f"Error during disconnect: {e}")

    def get_joint_positions(self, radians: bool = True) -> np.ndarray:
        """Get current joint positions."""
        if not self.is_connected:
            raise RuntimeError("Robot is not connected")

        try:
            if self.mode in ["real", "mock"]:
                positions = self.motor_bus.read("Present_Position", self.joint_names)
                if radians:
                    positions[0 : len(self.joint_names) - 1] *= np.pi / 180

                # Use system time for real/mock modes
                current_time = time.time() - self.start_time
                self.logger.debug(
                    f"time = {current_time:.3f}s, {self.mode} qPos Feedback {np.array2string(positions, precision=3, suppress_small=True, floatmode='fixed')}"
                )
                return positions

            elif self.mode == "sim":
                num_joints = len(self.joint_names)
                positions = self.data.qpos[:num_joints].copy()
                positions[num_joints - 1] *= 180.0 / np.pi

                for idx in self.invert_indices:
                    if idx < len(positions):
                        positions[idx] *= -1

                if not radians:
                    positions[0 : num_joints - 1] *= 180.0 / np.pi

                # Use simulation time for MuJoCo
                sim_time = self.data.time
                self.logger.debug(
                    f"time = {sim_time:.3f}s, mujoco qPos Feedback {np.array2string(positions, precision=3, suppress_small=True, floatmode='fixed')}"
                )
                return positions

            self.logger.error(f"Unknown mode: {self.mode}")
            return np.zeros(len(self.joint_names))

        except Exception as e:
            self.logger.error(f"Failed to read joint positions: {e}")
            import traceback

            self.logger.error(f"Traceback: {traceback.format_exc()}")
            return np.zeros(len(self.joint_names))

    def get_joint_velocities(self) -> np.ndarray:
        if self.mode == "sim":

            vels = self.data.qvel.copy()

            # Use simulation time for MuJoCo
            sim_time = self.data.time
            self.logger.debug(
                f"time = {sim_time:.3f}s, mujoco qvel Feedback {np.array2string(vels, precision=3, suppress_small=True, floatmode='fixed')}"
            )
            return vels

        return np.zeros(len(self.joint_names))

    def set_joint_positions(self, positions: np.ndarray, radians: bool = True) -> None:
        """Send joint position commands to the robot."""
        if not self.is_connected:
            raise RuntimeError("Robot is not connected")

        try:
            # Log original command positions for debugging with time
            positions_copy = positions.copy()
            mode_name = "mujoco" if self.mode == "sim" else self.mode

            # Get appropriate time based on mode
            if self.mode == "sim":
                current_time = self.data.time
            else:
                current_time = time.time() - self.start_time

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

            if self.mode in ["real", "mock"]:
                if self.mode == "real":
                    positions_copy[0 : len(self.joint_names) - 1] *= 180.0 / np.pi

                num_joints = min(len(positions), len(self.joint_names))
                self.motor_bus.write(
                    "Goal_Position",
                    positions_copy[:num_joints],
                    self.joint_names[:num_joints],
                )

            elif self.mode == "sim":
                positions_copy[len(self.joint_names) - 1] *= np.pi / 180
                for idx in self.invert_indices:
                    if idx < len(positions):
                        positions_copy[idx] *= -1

                num_joints = min(len(positions), len(self.joint_names))
                self.data.ctrl[:num_joints] = positions_copy[:num_joints]

        except Exception as e:
            self.logger.error(f"Failed to write joint positions: {e}")
            raise

    def set_chassis_wheels_vels(self, vels: np.ndarray) -> None:
        """Send chassis wheels vels for lekiwi robot."""
        if self.mode == "sim":
            if len(self.data.ctrl) >= 6:
                self.data.ctrl[3:6] = vels

    def enable_torque(self, enable: bool = True) -> None:
        """Enable or disable motor torque."""
        if not self.is_connected:
            raise RuntimeError("Robot is not connected")

        if self.mode in ["real", "mock"]:
            try:
                torque_value = [1 if enable else 0] * len(self.joint_names)
                self.motor_bus.write(
                    "Torque_Enable", np.array(torque_value), self.joint_names
                )
                self.logger.info(f"Torque {'enabled' if enable else 'disabled'}")
            except Exception as e:
                self.logger.error(
                    f"Failed to {'enable' if enable else 'disable'} torque: {e}"
                )
                raise
        elif self.mode == "sim":
            pass

    def step(self) -> None:
        """Step the robot/simulation forward."""
        if self.mode != "sim":
            return

        mujoco.mj_step(self.model, self.data)

        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.last_render_time = self.data.time

        if self.viewer.is_running():
            current_time = self.data.time
            if current_time - self.last_render_time >= self.render_interval:
                if self.camera is not None:
                    with self.viewer.lock():
                        self.viewer.cam.distance = self.camera.distance
                        self.viewer.cam.azimuth = self.camera.azimuth
                        self.viewer.cam.elevation = self.camera.elevation
                        self.viewer.cam.lookat[:] = self.camera.lookat
                self.viewer.sync()
                self.last_render_time = current_time

    def reset(self) -> None:
        """Reset the robot to initial state."""
        if self.mode == "sim" and self.model and self.data:
            mujoco.mj_resetData(self.model, self.data)

    def is_viewer_alive(self) -> bool:
        """Check if the viewer is still alive."""
        if self.mode == "sim" and self.viewer:
            return self.viewer.is_running()
        return True

    def init_camera(self):
        """Setup camera position and orientation for simulation."""
        if self.mode == "sim" and self.camera is not None:
            self.camera.distance = 1.0
            self.camera.azimuth = 45.0
            self.camera.elevation = -20.0
            self.camera.lookat = np.array([0.0, -0.2, 0.2])

    def randomize_cube_pos(self):

        if hasattr(self, "cube_initialized") and self.cube_initialized:
            return  

        mug_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "mug")
        mug_pos = self.model.body_pos[mug_id]

        r_min = 0.2
        r_max = 0.35

        cube_names = ["Cube1", "Cube2", "Cube3", "Cube4", "Cube5"]

        for name in cube_names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            qpos_adr = self.model.jnt_qposadr[joint_id]

            if qpos_adr + 6 >= len(self.data.qpos):
                continue 

            while True:
                r = np.sqrt(np.random.uniform(r_min**2, r_max**2))
                theta = np.random.uniform(-3 * np.pi / 4, -np.pi / 4)

                x = r * np.cos(theta)
                y = r * np.sin(theta)

                dist_to_mug = np.sqrt((x - mug_pos[0])**2 + (y - mug_pos[1])**2)
                if  dist_to_mug>0.025 and abs(x) > 0.05:
                    break

            self.data.qpos[qpos_adr + 0] = x
            self.data.qpos[qpos_adr + 1] = y  
            self.data.qpos[qpos_adr + 2] = random.uniform(0.2, 0.5)     

            quat = np.random.rand(4)
            quat /= np.linalg.norm(quat)
            self.data.qpos[qpos_adr + 3:qpos_adr + 7] = quat
        
        self.cube_initialized = True

    def set_camera(self, distance=None, azimuth=None, elevation=None, lookat=None):
        """
        Set camera parameters for custom viewing (simulation only).

        Args:
            distance: Distance from lookat point
            azimuth: Horizontal rotation around lookat point (degrees)
            elevation: Vertical rotation (degrees)
            lookat: 3D point to look at [x, y, z]
        """
        if self.mode == "sim" and self.camera is not None:
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
        if self.mode == "sim" and self.data is not None:
            return self.data.time
        else:
            return time.time() - self.start_time


def check_calibration_file(calibration_file: Optional[str], mode: str) -> bool:
    """
    Check if calibration file exists and provide helpful error messages.

    Args:
        calibration_file: Path to calibration file
        mode: Robot mode ("real", "mock", "sim")

    Returns:
        True if calibration is available or not required, False otherwise

    Raises:
        FileNotFoundError: If calibration file is required but missing
    """
    if mode in ["mock", "sim"]:
        return True

    if mode == "real":
        if not calibration_file:
            error_msg = (
                "❌ CALIBRATION FILE REQUIRED: No calibration file specified for real robot mode.\n"
                "   Please specify a calibration file or run calibration first:\n"
                "   python -m scripts.follower_calibrate --mode manual"
            )
            raise FileNotFoundError(error_msg)

        if not os.path.exists(calibration_file):
            error_msg = (
                f"❌ CALIBRATION FILE NOT FOUND: {calibration_file}\n"
                f"   This file is required for real robot operation.\n"
                f"   Please run calibration first:\n"
                f"   python -m scripts.follower_calibrate --mode manual\n"
                f"   \n"
                f"   Expected calibration file location: {calibration_file}"
            )
            raise FileNotFoundError(error_msg)

    return True


def create_robot_interface(
    mode: str = "mock", config_path: Optional[str] = None, scene: int = 1, **kwargs
) -> UnifiedRobotInterface:
    """
    Factory function to create a robot interface.

    Args:
        mode: "real", "mock", or "sim"
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

    calibration_file = kwargs.get("calibration_file", None)
    check_calibration_file(calibration_file, mode)

    return UnifiedRobotInterface(mode=mode, config=config, scene=scene, **kwargs)
