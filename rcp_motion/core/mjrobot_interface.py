"""
MujocoInterface - Single-threaded MuJoCo Simulation Interface

Simplified interface with passive viewer (no multi-threading):
- Single control loop with passive viewer sync
- Direct mjData access (zero-copy)
- Uses PRobotManager and PSceneManager
- Proper resource cleanup to prevent malloc corruption

Example:
    interface = MujocoInterface(robot="fr3", scene_number=1)
    while interface.is_viewer_alive():
        interface.qCmd[:] = compute_command()
        interface.step()
"""

import mujoco as mj
import mujoco.viewer
import numpy as np
from typing import Optional, Union, Dict, Any

from rcp_motion.core.robotinterface_base import (
    RobotInterfaceBase,
    register_robotinterface_factory_func,
)

from rcp_motion.core.data.robot_state import RobotState


@register_robotinterface_factory_func("mujoco_sim_robot")
def mujoco_sim_robot_factory(robot_model, robot_config: dict, logger=None):
    """
    Mujoco Robot Interface class generator function, along with additional algo kwargs.
    """
    return MujocoRobotInterface(robot_model, robot_config, logger)


class MujocoRobotInterface(RobotInterfaceBase):
    """
    Single-threaded MuJoCo simulation interface with passive viewer.

    Simplified design for passive viewer (no threading complexity):
    - Direct mjData access (no RuntimeData copying)
    - Single frequency (control and physics same rate)
    - Proper cleanup to fix malloc corruption bug
    """

    def __init__(self, robot_model, robot_config, logger):
        """
        Initialize MujocoInterface.

        Args:
            robot: Robot identifier - int (e.g., 20) or str (e.g., "fr3")
            scene_number: Scene identifier
            frequency: Control frequency in Hz (default: 100)
            config_path: Optional path to YAML config file
            scene_name: Optional scene name for special initialization
            **kwargs: Additional backward compatibility parameters
        """
        # Convert robot name to number if needed
        super().__init__(robot_model, robot_config, logger)

        # Initialize ALL resources to None FIRST (prevents GC malloc issues)
        self.mjModel: Optional[mj.MjModel] = None
        self.mjData: Optional[mj.MjData] = None
        self.viewer: Optional[any] = None
        self.camera_renderer: Optional[mj.Renderer] = None
        self.video_writer: Optional[Any] = None

        # Configuration
        self.config_path = robot_config
        self.config: Optional[Dict[str, Any]] = None

        # Timing (single frequency, no multi-threading)
        self.control_freq = robot_model.get_robot_control_freq()
        self.dt = 1.0 / self.control_freq

        # Camera settings
        self.camera_saving_mode: str = "none"
        self.camera_name: Optional[str] = None
        self.camera_id: int = -1
        self.picture_count: int = 0

        # Viewer camera settings
        self._camera_distance: float = 2.0
        self._camera_azimuth: float = 90.0
        self._camera_elevation: float = -20.0
        self._camera_lookat: np.ndarray = np.array([0.0, 0.0, 0.0])

        self.load_mujoco_model(robot_model)

        # Initialize
        # self.init_interface()

    # NOTE: No __del__ method - let Python GC handle cleanup naturally
    # This avoids malloc corruption during interpreter shutdown

    def load_mujoco_model(self, robot_model):
        """Load MuJoCo model and initialize simulation"""

        scene_path = robot_model.get_robot_mjcf()
        self.mjModel = mj.MjModel.from_xml_path(scene_path)
        self.mjData = mj.MjData(self.mjModel)

        # Set physics timestep to match control frequency
        self.mjModel.opt.timestep = self.dt

        # Forward kinematics
        mj.mj_forward(self.mjModel, self.mjData)
        self.simTime = self.mjData.time

        self.qCmd = np.zeros(self.mdof)
        self.qdCmd = np.zeros(self.mdof)
        self.qTau = np.zeros(self.mdof)

        self.logger.info(
            f"MuJoCo initialized: {self.mdof} DOF at {self.control_freq} Hz"
        )

    def connect(self):
        self.connected = True

    def get_robot_state_feedbacks(self):
        # Extract joint feedbacks
        self.simTime = self.mjData.time
        self.get_joint_feedbacks()
        self.get_endeffector_feedbacks()

        return self.robot_feedback

    def set_robot_command(self, command):
        for i in range(self.mdof):
            self.mjData.ctrl[i] = command.joint_pos[i]

    def get_joint_feedbacks(self):
        """
        Read joint states from MuJoCo.

        Direct access to mjData - zero copy!
        """
        self.simTime = self.mjData.time

        # Extract joint feedbacks
        for i in range(self.mdof):
            # Map actuator to joint
            joint_id = self.mjModel.actuator_trnid[i, 0]
            qpos_idx = self.mjModel.jnt_qposadr[joint_id]

            self.robot_feedback.joint_pos[i] = self.mjData.qpos[qpos_idx]
            self.robot_feedback.joint_vel[i] = self.mjData.qvel[qpos_idx]
            self.robot_feedback.joint_torque[i] = self.mjData.actuator_force[qpos_idx]

    def set_joint_commands(self):
        """
        Write joint commands to MuJoCo.

        Direct write to mjData.ctrl - zero copy!
        """
        # Direct write (position control assumed)
        self.mjData.ctrl[: self.mdof] = self.qCmd

    def get_endeffector_feedbacks(self):
        for i in range(self.ee_num):
            self.robot_feedback.ee_pose[i].pos = (
                self.mjData.site(self.ee_site_name[i]).xpos.copy().tolist()
            )
            mj_ori = self.mjData.site(self.ee_site_name[i]).xmat.copy()
            mj_quat_wxyz = np.zeros(4)
            mj.mju_mat2Quat(mj_quat_wxyz, mj_ori)
            self.robot_feedback.ee_pose[i].quat[:3] = mj_quat_wxyz[1:].tolist()
            self.robot_feedback.ee_pose[i].quat[3] = mj_quat_wxyz[0].tolist()  # xyzw

    def step(self, render_flag: bool = True):
        """
        Step simulation once (single-threaded).

        Args:
            render_flag: Whether to update viewer (default: True)
        """
        # Step physics
        mj.mj_step(self.mjModel, self.mjData)

        # Render if enabled
        if render_flag:
            # Lazy viewer creation (compatible with ControllerBase)
            if self.viewer is None:
                self.viewer = mujoco.viewer.launch_passive(self.mjModel, self.mjData)

            # Update viewer if running
            if self.viewer and self.viewer.is_running():
                self.viewer.sync()

    def disconnect(self):
        """
        Clean shutdown - minimal cleanup, let Python GC handle MuJoCo objects.

        Following the pattern from examples/lekiwi.py and examples/xlerobot.py:
        - Don't explicitly close viewer (causes malloc corruption)
        - Don't explicitly close mjModel/mjData (let GC handle it)
        - Only close resources we explicitly manage (video writer, camera renderer)
        """
        # Prevent double-disconnect
        if not self.connected:
            return

        self.connected = False

        # Only close resources we explicitly manage (not MuJoCo objects)
        try:
            if hasattr(self, "video_writer") and self.video_writer:
                self.video_writer.release()
                self.video_writer = None
        except Exception as e:
            print(f"Warning: Error releasing video writer: {e}")

        try:
            if hasattr(self, "camera_renderer") and self.camera_renderer:
                self.camera_renderer.close()
                self.camera_renderer = None
        except Exception as e:
            print(f"Warning: Error closing camera renderer: {e}")

        if self.viewer:
            self.viewer.close()

        # Don't close viewer - let Python GC handle it (avoids malloc corruption)
        # Don't close mjModel/mjData - let Python GC handle them

        print("✓ MuJoCo simulation disconnected")

        # Call parent disconnect
        super().disconnect()

    def _get_time_source(self) -> str:
        """Use simulation time"""
        return "sim"

    # ========== Backward Compatibility Aliases ==========

    @property
    def data(self):
        """Alias for mjData (for code using interface.data.site(), etc.)"""
        return self.mjData

    @property
    def model(self):
        """Alias for mjModel (for code using interface.model)"""
        return self.mjModel

    def get_joint_positions(self) -> np.ndarray:
        """
        Get current joint positions.
        Calls get_joint_feedbacks() internally, then returns qFb.
        """
        self.get_joint_feedbacks()
        return np.array(self.robot_feedback.joint_pos)

    def get_joint_velocities(self) -> np.ndarray:
        """
        Get current joint velocities.
        Calls get_joint_feedbacks() internally, then returns qdFb.
        """
        self.get_joint_feedbacks()
        return np.array(self.robot_feedback.joint_vel)

    def set_joint_positions(self, positions: np.ndarray) -> None:
        """Set joint position commands (updates qCmd)"""
        self.qCmd[:] = positions

    def get_current_time(self) -> float:
        """Get current simulation time from mjData.time"""
        return self.mjData.time

    def is_viewer_alive(self) -> bool:
        """
        Check if viewer is still running.

        Returns:
            True if viewer exists and is running, False otherwise
        """
        if self.viewer:
            try:
                return self.viewer.is_running()
            except:
                return False
        return True

    # ========== Camera Control & Recording ==========

    def set_camera(
        self, distance: float, azimuth: float, elevation: float, lookat: list
    ) -> None:
        """
        Set viewer camera position and orientation.

        Args:
            distance: Camera distance from lookat point
            azimuth: Azimuth angle in degrees
            elevation: Elevation angle in degrees
            lookat: [x, y, z] lookat point coordinates
        """
        self._camera_distance = distance
        self._camera_azimuth = azimuth
        self._camera_elevation = elevation
        self._camera_lookat = np.array(lookat)

        # Apply to viewer if it exists
        if self.viewer and hasattr(self.viewer, "cam"):
            self.viewer.cam.distance = distance
            self.viewer.cam.azimuth = azimuth
            self.viewer.cam.elevation = elevation
            self.viewer.cam.lookat[:] = lookat

    def init_camera_recording(
        self, mode: str, camera_name: str, width: int = 640, height: int = 480
    ) -> None:
        """
        Initialize camera recording for dataset collection.

        Args:
            mode: "image" (save PNGs), "video" (save MP4), or "none"
            camera_name: Name of camera in MuJoCo model
            width: Frame width in pixels
            height: Frame height in pixels
        """
        self.camera_saving_mode = mode
        self.camera_name = camera_name

        if mode == "none":
            return

        # Find camera ID
        try:
            self.camera_id = mj.mj_name2id(
                self.mjModel, mj.mjtObj.mjOBJ_CAMERA, camera_name
            )
        except:
            print(f"Warning: Camera '{camera_name}' not found in model")
            self.camera_saving_mode = "none"
            return

        # Create offscreen renderer
        self.camera_renderer = mj.Renderer(self.mjModel, height=height, width=width)
        print(
            f"Camera recording initialized: mode={mode}, camera={camera_name}, {width}x{height}"
        )

    def capture_camera_frame(self) -> Optional[np.ndarray]:
        """
        Capture RGB frame from camera.

        Returns:
            RGB array (H, W, 3) uint8, or None if camera not initialized
        """
        if self.camera_saving_mode == "none" or self.camera_renderer is None:
            return None

        self.camera_renderer.update_scene(self.mjData, camera=self.camera_id)
        rgb_array = self.camera_renderer.render()

        return rgb_array

    def save_camera_frame(self, filename: str) -> bool:
        """
        Capture and save current camera frame as PNG.

        Args:
            filename: Output filename (e.g., "frame_0001.png")

        Returns:
            True if saved successfully
        """
        rgb_array = self.capture_camera_frame()
        if rgb_array is None:
            return False

        try:
            import imageio

            imageio.imwrite(filename, rgb_array)
            return True
        except ImportError:
            print("Warning: imageio not installed, cannot save images")
            return False
