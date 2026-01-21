import os
import numpy as np
import logging
import random
import datetime
import cv2
from typing import Dict, Optional
from PIL import Image
from concurrent.futures import ThreadPoolExecutor

import mujoco
import mujoco.viewer

from .interface import InterfaceBase


class MuJoCoInterface(InterfaceBase):

    def __init__(
        self,
        robot_name: str = "Unknown",
        config: Optional[Dict] = None,
        scene: str = "",
        **kwargs,
    ):
        """
        Initialize the mujoco interface.

        Args:
            config: Configuration dictionary or None for defaults
            scene: scene path
            **kwargs: Additional arguments for specific modes
        """
        super().__init__()
        self.robot_name = robot_name
        self.scene = scene
        self.scene_name = kwargs.get("scene_name", "default")
        self.timestep = kwargs.get("timestep", 0.001)

        self.config = config
        self.logger = logging.getLogger(__name__)

        self.model = None
        self.data = None
        self.viewer = None
        self.cube_initialized = False

        self.render_freq = kwargs.get("render_freq", 60)
        self.render_interval = 1.0 / self.render_freq
        self.last_render_time = 0

        self.camera = None
        self._init_mujoco()

    def _init_mujoco(self):
        """Initialize MuJoCo simulation with custom timestep."""
        sim_config = self.config.get("simulation", {})
        mjcf_path = self.scene

        if not os.path.exists(mjcf_path):
            script_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.dirname(os.path.dirname(script_dir))
            mjcf_path = os.path.join(project_root, mjcf_path)

        if not os.path.exists(mjcf_path):
            raise FileNotFoundError(f"MuJoCo model file not found: {mjcf_path}")

        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        if self.model is not None:
            self.model.opt.timestep = self.timestep
            self.logger.info(f"Set MuJoCo timestep to: {self.model.opt.timestep}")
        self.data = mujoco.MjData(self.model)

        # Initialize joint names for InterfaceBase compatibility
        self.joint_names = []
        for i in range(self.model.njnt):
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if joint_name and self.model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE:
                self.joint_names.append(joint_name)
        self.logger.info(f"Found {len(self.joint_names)} joints: {self.joint_names}")

        random_init_joints = sim_config.get("random_init_joints", False)
        if random_init_joints:
            self._random_Joints()

        if self.scene_name == "so100_grip":
            keyframe_dict = {"middle": 0, "zero": 1, "erect": 2}
            if random_init_joints is False:
                keyframe = sim_config.get("cube_grabber_initial_keyframe", "zero")
                if keyframe not in ["middle", "zero", "erect"]:
                    raise ValueError(
                        f"Invalid camera saving mode: {keyframe}. "
                        "Must be one of 'middle', 'zero' or 'erect'."
                    )
                keyframeid = keyframe_dict[keyframe]
                self.data.qpos = self.model.key_qpos[keyframeid, :]
                self.data.ctrl = self.model.key_ctrl[keyframeid, :]
                mujoco.mj_step(self.model, self.data)
            self._random_Cubes()

        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)
        self._init_camera()
        self._init_camera_saving(sim_config)

    def _init_camera(self):
        """Setup camera position and orientation for simulation."""
        if self.camera is not None:
            self.camera.distance = 1.0
            self.camera.azimuth = 45.0
            self.camera.elevation = -20.0
            self.camera.lookat = np.array([0.0, -0.2, 0.2])

    def _init_camera_saving(self, sim_config):
        """Initialize camera saving mode if specified."""
        camera_saving_mode = sim_config.get("camera_saving_mode", "image")
        if camera_saving_mode not in ["image", "video", "none"]:
            raise ValueError(
                f"Invalid camera saving mode: {camera_saving_mode}. "
                "Must be one of 'image', 'video' or 'none'."
            )
        self.camera_saving_mode = camera_saving_mode

        if self.camera_saving_mode != "none":
            log_dir = os.path.expanduser(
                self.config.get("logging", {}).get("log_dir", "~/logs/lerobot")
            )
            self.camera_log_dir = os.path.join(log_dir, "camera")
            os.makedirs(self.camera_log_dir, exist_ok=True)
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

            self.camera_name = sim_config.get("camera_name", "Wrist_camera")
            self.last_save_time = self.data.time

            # Calculate timing parameters for optimized saving
            camera_frequency = sim_config.get("camera_frequency", 30)
            self.camera_saving_interval = 1.0 / camera_frequency
            self.next_save_time = self.data.time + self.camera_saving_interval

            self.logger.info(
                f"Camera: {self.camera_name} saving mode set to: {self.camera_saving_mode} "
                f"at {camera_frequency}Hz (interval: {self.camera_saving_interval:.4f}s)"
            )

            try:
                self.camera_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name
                )
                self.logger.info(
                    f"Found camera '{self.camera_name}' with ID: {self.camera_id}"
                )
            except:
                self.logger.error(f"Camera '{self.camera_name}' not found in model")
                self.camera_id = -1
                self.camera_saving_mode = "none"
                return

            # Initialize camera renderer with fixed settings
            self.camera_renderer = mujoco.Renderer(self.model, width=640, height=480)
            self.camera_scene_option = mujoco.MjvOption()
            self.camera_scene_option.geomgroup[1] = 0

            # Initialize async image saving
            self.camera_save_executor = ThreadPoolExecutor(
                max_workers=2, thread_name_prefix="camera_save"
            )
            self.pending_saves = 0
            self.max_pending_saves = 5  # Prevent memory buildup

            if self.camera_saving_mode == "image":
                self.image_save_dir = os.path.join(
                    self.camera_log_dir, "images", timestamp
                )
                os.makedirs(self.image_save_dir, exist_ok=True)
                self.picture_count = 0

                # Fixed image settings
                self.image_format = "PNG"
                self.image_quality = 95

                self.logger.info(f"Image save directory: {self.image_save_dir}")

            elif self.camera_saving_mode == "video":
                # Generate video filename with timestamp
                video_filename = os.path.join(
                    self.camera_log_dir,
                    "videos",
                    f"{self.camera_name}_video_{timestamp}.mp4",
                )

                # Set up OpenCV VideoWriter
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # Use MP4 codec
                fps = sim_config.get("camera_frequency", 30)
                frame_size = (self.camera_renderer.width, self.camera_renderer.height)
                self.video_writer = cv2.VideoWriter(
                    video_filename, fourcc, fps, frame_size
                )
                if not self.video_writer.isOpened():
                    self.logger.error(
                        f"Failed to initialize video writer for {video_filename}"
                    )
                    self.camera_saving_mode = "none"
                    return

                self.logger.info(f"Video save path: {video_filename}")

        else:
            self.logger.debug(
                "Camera saving mode is set to 'none', skipping camera setup."
            )

    def _render_camera_frame(self):
        """Save camera image asynchronously for better performance."""
        if self.camera_saving_mode == "none" or self.camera_id < 0:
            return

        # Skip if too many pending saves to prevent memory buildup
        if self.pending_saves >= self.max_pending_saves:
            return

        try:
            # Update camera scene and render
            self.camera_renderer.update_scene(
                self.data,
                camera=self.camera_name,
                scene_option=self.camera_scene_option,
            )

            rgb_array = self.camera_renderer.render()

            if self.camera_saving_mode == "image":
                self.picture_count += 1

                # Create filename with proper extension
                extension = "jpg" if self.image_format == "JPEG" else "png"
                img_filename = (
                    f"{self.camera_name}_image_{self.picture_count:06d}.{extension}"
                )
                img_path = os.path.join(self.image_save_dir, img_filename)

                # Submit async save task
                self.pending_saves += 1
                future = self.camera_save_executor.submit(
                    self._save_image_async,
                    rgb_array.copy(),  # Copy to avoid data races
                    img_path,
                    self.picture_count,
                )
                future.add_done_callback(self._on_camera_save_complete)

            elif self.camera_saving_mode == "video":
                # Submit async save task
                self.pending_saves += 1
                future = self.camera_save_executor.submit(
                    self._save_video_async, rgb_array.copy()
                )
                future.add_done_callback(self._on_camera_save_complete)

        except Exception as e:
            self.logger.error(f"Failed to render camera frame: {e}")
            import traceback

            self.logger.error(f"Traceback: {traceback.format_exc()}")

    def _save_image_async(self, rgb_array: np.ndarray, img_path: str, frame_count: int):
        """Asynchronously save image to disk."""
        try:
            image = Image.fromarray(rgb_array)

            if self.image_format == "JPEG":
                image.save(
                    img_path, format="JPEG", quality=self.image_quality, optimize=True
                )
            else:  # PNG
                image.save(img_path, format="PNG", optimize=True)

            # Only log every 30 frames to reduce log spam
            if frame_count % 30 == 0:
                self.logger.debug(f"Saved image batch around frame {frame_count}")

        except Exception as e:
            self.logger.error(f"Failed to save image {img_path}: {e}")

    def _save_video_async(self, rgb_array: np.ndarray):
        try:
            bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
            self.video_writer.write(bgr_array)
        except Exception as e:
            self.logger.error(f"Failed to write video frame: {e}")

    def _on_camera_save_complete(self, future):
        """Callback when image save completes."""
        self.pending_saves = max(0, self.pending_saves - 1)
        try:
            future.result()  # This will raise any exceptions that occurred
        except Exception as e:
            self.logger.error(f"{self.camera_saving_mode} save task failed: {e}")

    def set_chassis_wheels_vels(self, vels: np.ndarray) -> None:
        """Send chassis wheels vels for lekiwi robot."""
        if len(self.data.ctrl) >= 6:
            self.data.ctrl[3:6] = vels

    # InterfaceBase implementation
    def init(self, timestep: float = 0.01) -> None:
        """Initialize the MuJoCo simulation with specified timestep."""
        self.timestep = timestep
        if self.model is not None:
            self.model.opt.timestep = self.timestep
            self.logger.info(f"Updated MuJoCo timestep to: {self.timestep}")
        self.is_connected = True

    def disconnect(self) -> None:
        """Disconnect from MuJoCo simulation."""
        if self.viewer:
            self.viewer.close()
            self.viewer = None

        if hasattr(self, "camera_renderer") and self.camera_renderer is not None:
            self.camera_renderer.close()
            self.camera_renderer = None

        if hasattr(self, "video_writer") and self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None

        # Clean up camera saving resources
        if (
            hasattr(self, "camera_save_executor")
            and self.camera_save_executor is not None
        ):
            self.camera_save_executor.shutdown(wait=True)
            self.camera_save_executor = None

        self.is_connected = False
        self.logger.info("✓ MuJoCo simulation disconnected")

    def get_joint_positions(self) -> np.ndarray:
        """
        Get current joint positions from MuJoCo simulation.

        Returns:
            Array of joint positions
        """
        if self.data is None:
            raise RuntimeError("MuJoCo simulation not initialized")

        # Return joint positions (excluding free joints for objects)
        # This assumes robot joints are the first controlled joints
        num_joints = self.model.nu if hasattr(self.model, "nu") else len(self.data.qpos)
        joint_positions = self.data.qpos[:num_joints].copy()
        if self.robot_name == "lerobot":
            joint_positions[-1] *= 180 / np.pi

        self.logger.debug(
            f"time={self.data.time:.3f}, sim qpos Feedback {np.array2string(joint_positions, precision=3, suppress_small=True, floatmode='fixed')}"
        )
        return joint_positions

    def set_joint_positions(self, positions: np.ndarray) -> None:
        """
        Set joint position commands in MuJoCo simulation.

        Args:
            positions: Array of joint positions
        """
        if self.data is None:
            raise RuntimeError("MuJoCo simulation not initialized")

        # Set control inputs (position commands)
        num_controls = min(len(positions), len(self.data.ctrl))
        if self.robot_name == "lerobot":
            positions[-1] *= np.pi / 180

        self.logger.debug(
            f"time={self.data.time:.3f}, sim qpos Command {np.array2string(positions[:num_controls], precision=3, suppress_small=True, floatmode='fixed')}"
        )
        self.data.ctrl[:num_controls] = positions[:num_controls]

    def get_joint_velocities(self) -> np.ndarray:
        """
        Get current joint velocities from MuJoCo simulation.

        Returns:
            Array of joint velocities
        """
        if self.data is None:
            raise RuntimeError("MuJoCo simulation not initialized")

        num_joints = self.model.nu if hasattr(self.model, "nu") else len(self.data.qvel)
        return self.data.qvel[:num_joints].copy()

    def get_current_time(self) -> float:
        """
        Get current simulation time.

        Returns:
            Current simulation time in seconds
        """
        if self.data is None:
            return 0.0
        return self.data.time

    def step(self, render_flag=True) -> None:
        """Step the robot/simulation forward."""

        mujoco.mj_step(self.model, self.data)

        if not render_flag:
            return

        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.last_render_time = self.data.time

        if self.viewer.is_running():
            current_time = self.data.time
            if current_time - self.last_render_time >= self.render_interval:
                self.viewer.sync()
                self.last_render_time = current_time

        # Optimized camera saving with precise timing
        if self.camera_saving_mode != "none" and self.data.time >= self.next_save_time:
            self._render_camera_frame()
            # Schedule next save time to maintain consistent frequency
            self.next_save_time += self.camera_saving_interval

    def reset(self) -> None:
        """Reset the robot to initial state."""
        if self.model and self.data:
            mujoco.mj_resetData(self.model, self.data)

            # Reset camera timing when simulation resets
            if hasattr(self, "next_save_time") and hasattr(
                self, "camera_saving_interval"
            ):
                self.next_save_time = self.data.time + self.camera_saving_interval

    def is_viewer_alive(self) -> bool:
        """Check if the viewer is still alive."""
        if self.viewer:
            return self.viewer.is_running()
        return True

    def _random_Cubes(self):

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

                dist_to_mug = np.sqrt((x - mug_pos[0]) ** 2 + (y - mug_pos[1]) ** 2)
                if dist_to_mug > 0.05 and abs(y) > 0.1:
                    break

            self.data.qpos[qpos_adr + 0] = x
            self.data.qpos[qpos_adr + 1] = y

            random_yaw = np.random.uniform(-1, 1, size=2)
            random_yaw /= np.linalg.norm(random_yaw)
            quat = np.array([random_yaw[0], 0.0, 0.0, abs(random_yaw[1])])

            self.data.qpos[qpos_adr + 3 : qpos_adr + 7] = quat

        self.cube_initialized = True

    def _set_camera(self, distance=None, azimuth=None, elevation=None, lookat=None):
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

    def _random_Joints(self):
        """Set random initial pose for mujoco."""
        try:
            q = np.empty(self.model.nq)
            q[:] = self.data.qpos[:]

            ctrl_start = None
            ctrl_start_flag = True

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
                    if ctrl_start_flag:
                        ctrl_start_flag = False
                        ctrl_start = qpos_addr

                    q[qpos_addr] = np.random.uniform(low, high)
                    self.logger.debug(
                        f"Joint {j}: random pos {q[qpos_addr]:.3f} in range [{low:.3f}, {high:.3f}]"
                    )

            self.data.qpos[:] = q
            self.data.qvel[:] = 0
            self.data.qacc[:] = 0
            self.data.ctrl[:] = q[ctrl_start : ctrl_start + self.model.nu]
            mujoco.mj_forward(self.model, self.data)
            self.logger.info("✓ Random initial pose set successfully")

        except Exception as e:
            self.logger.error(f"Failed to set random pose: {e}")
            # If random pose setting fails, continue with default pose
            mujoco.mj_resetData(self.model, self.data)
