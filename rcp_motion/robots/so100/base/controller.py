#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Base Unified Controller - Abstract base class for all robot controllers

This module provides the common control logic and infrastructure that all robot
implementations (LeRobot, Lekiwi, etc.) can inherit from.
"""

import os
import numpy as np
import time
import logging
import yaml
import platform
from abc import ABC, abstractmethod

if platform.system() == "Darwin":
    import matplotlib

    matplotlib.use("Agg")  # Use non-interactive backend on macOS

from rcp_motion.utils.policy_interpolator import PolicyInterpolator, lerp
from rcp_motion.utils.lcm_handler import LCMHandler
from rcp_motion.utils.joint_plotter import JointPlotter
from rcp_motion.utils.interpolator_plotter import InterpolatorPlotter
from common.lcm.lcmMotion.state_feedback import state_feedback


class ControllerBase(ABC):
    """
    Abstract base class for robot controllers with common functionality.

    This class provides the core control loop, timing, communication, and
    plotting infrastructure that all robot controllers need.
    """

    def __init__(
        self,
        robot_name: str,
        mode: str = "sim",
        motion: int = 0,
        frequency: int = 100,
        config_path: str = "configs/config.yaml",
        calibration_file: str = None,
    ):
        """
        Initialize the controller.

        Args:
            mode: Control mode - "sim", "real", or "mock"
            motion: Predefined motion pattern (0=disabled)
            frequency: Control frequency in Hz
            config_path: Path to configuration file
        """
        self.robot_name = robot_name
        self.mode = mode
        self.motion = motion
        self.frequency = max(30, min(frequency, 250))
        self.timestep = 1.0 / self.frequency
        self.config_path = config_path
        self.calibration_file = calibration_file

        self.init_config()
        self.init_logging()
        self.init_state()
        self.init_interpolator()
        self.init_interface()
        self.init_communication()
        self.init_joint_plotting()
        self.init_interpolator_plotting()

    def get_supported_robots(self) -> list:
        """Return the list of supported robots."""
        return ["lerobot", "lekiwi", "franka", "realman"]

    def get_robot_name(self) -> str:
        """Return the robot name (e.g., 'lerobot', 'lekiwi')"""
        return self.robot_name

    @abstractmethod
    def get_scene_config(self) -> dict:
        """Return scene configuration for this robot type"""
        pass

    @abstractmethod
    def motion_planner(self, motion: int, current_time: float) -> np.ndarray:
        """Generate robot-specific motion patterns"""
        pass

    def custom_control_step(self):
        """Override this for robot-specific control logic (e.g., chassis control)"""
        pass

    def init_config(self):
        """Initialize configuration."""
        self.config = self.load_config(self.config_path)

        self.movement_parameters = {
            "movement_duration": 2,
            "amplitude": 0.3,
            "frequency": 0.4,
        }

        self.home_position = np.array(
            self.config.get("home_joint_positions", np.zeros(6))
        )

        self.inference_rate = self.config.get("robot", {}).get("inference_rate", 30.0)
        self.joint_plot_flag = self.config.get("plot", {}).get(
            "enable_joint_plotting", False
        )
        self.interpolator_plot_flag = self.config.get("plot", {}).get(
            "enable_interpolator_plotting", False
        )
        self.show_plots = self.config.get("plot", {}).get("show_plots", False)

    def init_logging(self):
        """Setup logging configuration."""
        log_dir = os.path.expanduser(
            self.config.get("logging", {}).get("log_dir", "~/logs/lerobot")
        )
        log_file = self.config.get("logging", {}).get(
            "log_file", "unified_controller.log"
        )

        if not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)

        from datetime import datetime

        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        log_file_with_timestamp = f"{log_file.replace('.log', '')}_{timestamp}.log"
        log_path = os.path.join(log_dir, log_file_with_timestamp)

        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s [%(levelname)s] %(message)s",
            handlers=[
                logging.FileHandler(log_path),
            ],
        )

        self.logger = logging.getLogger(__name__)
        print(f"✓ Log file: {log_path}")

    def init_state(self):
        """Initialize controller state variables."""
        self.interface = None
        self.initial_joint_positions = None
        self.lcm_handler = None
        self.joint_plotter = None
        self.interpolator_plotter = None

        self.qPos_command = None
        self.qPos_feedback = None

        self.timeout_seconds = self.config.get("robot", {}).get("timeout_seconds", 30.0)
        self.timeout_seconds = max(20.0, min(self.timeout_seconds, 90.0))
        self.go_home_start_position = None
        self.go_home_start_time = None

    def init_interpolator(self):
        """Initialize interpolation-related components."""
        self.interpolator = PolicyInterpolator()
        self.latest_ACT = None
        self.executing_ACT = None
        self.last_processed_seq = -1
        self.current_action_count = 0
        self.logger.info("✓ Interpolator initialized")

    @abstractmethod
    def init_interface(self):
        """Setup interface - can be overridden by subclasses"""
        pass

    def init_communication(self):
        """Setup LCM communication handler."""
        self.lcm_handler = LCMHandler(self.logger)
        if self.lcm_handler.connect():
            self.logger.info("✓ LCM handler initialized")
        else:
            self.logger.warning("⚠ LCM handler initialization failed")

    def init_joint_plotting(self):
        """Initialize offline joint plotting if enabled."""
        if not self.joint_plot_flag:
            self.joint_plotter = None
            return

        joint_names = self.interface.joint_names
        log_dir = self.config.get("logging", {}).get("log_dir", "~/logs/lerobot")
        self.joint_plotter = JointPlotter(joint_names, log_dir=log_dir)
        self.logger.info("✓ Offline joint plotting initialized")

    def init_interpolator_plotting(self):
        """Initialize offline interpolator plotting if enabled."""
        if not self.interpolator_plot_flag:
            self.interpolator_plotter = None
            return

        joint_names = self.interface.joint_names
        log_dir = self.config.get("logging", {}).get("log_dir", "~/logs/lerobot")
        self.interpolator_plotter = InterpolatorPlotter(joint_names, log_dir=log_dir)
        self.logger.info("✓ Offline interpolator plotting initialized")

    def load_config(self, config_path):
        """Load configuration from YAML file."""
        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
            return config
        except FileNotFoundError:
            (
                self.logger.warning(
                    f"Config file {config_path} not found, using defaults"
                )
                if hasattr(self, "logger")
                else print(
                    f"Warning: Config file {config_path} not found, using defaults"
                )
            )
            return {}

    def _pre_time_handling(self):
        """Handle timing setup at the start of each control loop iteration."""
        precall_time = self.interface.get_current_time()
        precall_wall_time = time.time()
        nextcall_time = precall_time + self.timestep
        return precall_time, precall_wall_time, nextcall_time

    def get_state(self):
        """Determine which state the robot should be in."""
        if self.lcm_handler and self.lcm_handler.check_gohome_request():
            return "go_home"

        if (
            self.lcm_handler
            and self.lcm_handler.is_command_timeout(self.timeout_seconds)
            and self.motion == 0
        ):
            return "go_home"

        if self.motion != 0:
            return "motion"

        return "policy"

    def _post_time_handling(self, precall_wall_time):
        """Handle timing synchronization at the end of each control loop iteration."""
        consumed_time = time.time() - precall_wall_time
        sleep_time = self.timestep - consumed_time

        if sleep_time > 0:
            time.sleep(sleep_time)
        elif sleep_time < -0.001:
            self.logger.warning(f"Control loop running behind by {-sleep_time:.3f}s")

    def _get_feedbacks(self):
        """Read current joint positions for feedback."""
        self.qPos_feedback = self.interface.get_joint_positions()
        self.qdFb = self.interface.get_joint_velocities()

    def _lcm_subscribing(self):
        """Handle LCM message subscription and processing."""
        if self.lcm_handler:
            self.lcm_handler.process_messages(0)

    def _run_policy(self):
        """Execute LCM policy commands (scenario 0)."""
        latest_ACT = self.lcm_handler.get_latest_ACT()
        if latest_ACT and latest_ACT.seq != self.last_processed_seq:
            self._process_new_chunk(latest_ACT)

        if self.executing_ACT and self.current_action_count < self.highfreq_chunksize:
            self.update()

    def _process_new_chunk(self, new_chunk):
        """Process a new trajectory chunk."""
        self.latest_ACT = new_chunk
        self.executing_ACT = new_chunk
        self.last_processed_seq = new_chunk.seq
        self.current_action_count = 0

        qACT = []
        for i in range(self.executing_ACT.chunkSize):
            joint_start = i * self.executing_ACT.numJoint
            joint_end = joint_start + self.executing_ACT.numJoint
            qACT.extend(self.executing_ACT.jointPos[joint_start:joint_end])

            gripper_start = i * self.executing_ACT.numGripper
            gripper_end = gripper_start + self.executing_ACT.numGripper
            qACT.extend(self.executing_ACT.gripperPos[gripper_start:gripper_end])

        total_dof = self.executing_ACT.numJoint + self.executing_ACT.numGripper
        self.interpolator.prepare_trajectory(
            qACT,
            total_dof,
            self.executing_ACT.chunkSize,
        )

        self.lowfreq_chunksize = self.executing_ACT.chunkSize
        self.highfreq_chunksize = (
            self.frequency / self.inference_rate * (self.executing_ACT.chunkSize - 1)
        )

        if self.interpolator_plot_flag and self.interpolator_plotter:
            self._store_interpolator_plot_data(qACT, total_dof)

    def _store_interpolator_plot_data(self, qACT, total_dof):
        """Store trajectory data for offline interpolator plotting."""
        sampled_joint_positions = []
        high_freq_indices = np.linspace(
            0,
            self.lowfreq_chunksize - 1,
            int(self.highfreq_chunksize),
        )

        for traj_index in high_freq_indices:
            joint_values_at_index = self.interpolator.update(traj_index)
            sampled_joint_positions.extend(joint_values_at_index)

        self.interpolator_plotter.add_trajectory_data(
            qPos_input=qACT,
            qPos_output=sampled_joint_positions,
            num_joints=total_dof,
            chunk_size=self.lowfreq_chunksize,
            timestamp=self.interface.get_current_time(),
        )

    def update(self):
        """Execute one step of the current trajectory."""
        if self.lcm_handler:
            self.lcm_handler.set_act_status(state_feedback.kExecuting)

        traj_index = (
            self.current_action_count
            / self.highfreq_chunksize
            * (self.lowfreq_chunksize - 1)
        )

        self.qPos_command = np.array(self.interpolator.update(traj_index))
        self.current_action_count += 1
        if self.current_action_count >= self.highfreq_chunksize:
            self._complete_trajectory()

    def _complete_trajectory(self):
        """Handle trajectory completion."""
        if self.lcm_handler:
            self.lcm_handler.publish_state_feedback_with_status(state_feedback.kSuccess)
            self.lcm_handler.set_act_status(state_feedback.kIdle)
        self.executing_ACT = None

    def _run_motion(self):
        """Execute hardcoded motion patterns (scenarios 1-6)."""
        current_time = self.interface.get_current_time()
        self.qPos_command = self.motion_planner(self.motion, current_time)

    def _run_go_home(self):
        """Return to initial position from current position."""
        if self.lcm_handler is None:
            return

        current_time = time.time()

        if self.go_home_start_time is None:
            if self.lcm_handler.is_command_timeout(self.timeout_seconds):
                print(
                    f"Command timeout ({self.timeout_seconds}s) - returning to initial position"
                )
            else:
                print("Go home request received - returning to initial position")

            self.go_home_start_position = self.interface.get_joint_positions()
            self.go_home_start_time = current_time

        elapsed_time = current_time - self.go_home_start_time
        duration = self.movement_parameters["movement_duration"]

        if elapsed_time >= duration:
            print("Successfully returned to initial position, awaiting commands...")
            self.go_home_start_time = None
            self.go_home_start_position = None
            if self.lcm_handler:
                self.lcm_handler.gohome_requested = False
                self.lcm_handler.has_received_command = False
                self.lcm_handler.set_act_status(state_feedback.kIdle)
            return

        if self.home_position is None:
            target_position = np.zeros(len(self.go_home_start_position))
        elif elapsed_time < duration / 2.0:
            target_position = self.home_position
            target_position[len(self.home_position) - 1] = 100
        else:
            target_position = self.home_position
            target_position[len(self.home_position) - 1] = 0

        self.qPos_command = lerp(
            self.go_home_start_position, target_position, elapsed_time, duration
        )

    def _set_commands(self):
        """Send commands to the robot - calls custom control step"""
        self.custom_control_step()
        if self.qPos_command is not None:
            self.interface.set_joint_positions(self.qPos_command)

    def _offline_joint_plot(self):
        """Handle offline joint plotting data collection."""
        if not self.joint_plot_flag:
            return

        if self.joint_plotter and self.qPos_command is not None:
            current_time = self.interface.get_current_time()
            self.joint_plotter.add_data(
                self.qPos_command, self.qPos_feedback, current_time
            )

    def _lcm_publishing(self):
        """Handle LCM feedback publishing."""
        if not self.lcm_handler:
            return

        if self.lcm_handler.check_robot_feedback_request():
            self.lcm_handler.publish_robot_feedback(self.qPos_feedback)

        if self.lcm_handler.check_state_feedback_request():
            self.lcm_handler.publish_state_feedback()

    def _print_startup_banner(self):
        """Print startup information banner."""
        self.logger.info("=" * 60)
        self.logger.info(f"{self.get_robot_name().upper()} Controller is running!")
        self.logger.info(f"Mode: {self.mode}")
        self.logger.info(f"Motion: {self.motion}")
        self.logger.info(f"Control frequency: {self.frequency} Hz")
        self.logger.info(f"Command timeout: {self.timeout_seconds}s")
        self.logger.info(
            f"Joint plotting: {'enabled' if self.joint_plot_flag else 'disabled'}"
        )
        self.logger.info(
            f"Interpolator plotting: {'enabled' if self.interpolator_plot_flag else 'disabled'}"
        )
        self.logger.info(f"Show plots: {'enabled' if self.show_plots else 'save only'}")
        if self.lcm_handler.is_connected():
            self.logger.info("LCM communication enabled")
        if self.mode == "sim":
            self.logger.info("Press Ctrl+C or close viewer window to stop")
        else:
            self.logger.info("Press Ctrl+C to stop")
        self.logger.info("=" * 60)

    def run(self):
        """Run the controller."""
        self._print_startup_banner()

        try:
            while True:
                precall_time, precall_wall_time, nextcall_time = (
                    self._pre_time_handling()
                )
                self._get_feedbacks()

                self._lcm_subscribing()

                state = self.get_state()

                if state == "go_home":
                    self._run_go_home()
                elif state == "policy":
                    self._run_policy()
                elif state == "motion":
                    self._run_motion()

                self._set_commands()

                self.interface.step()

                self._offline_joint_plot()

                self._lcm_publishing()

                if self.mode == "sim" and not self.interface.is_viewer_alive():
                    self.logger.info(
                        "MuJoCo viewer window closed, stopping controller..."
                    )
                    break

                self._post_time_handling(precall_wall_time)

        except (KeyboardInterrupt, Exception) as e:
            if isinstance(e, KeyboardInterrupt):
                self.logger.info("\nStopping robot controller...")
            else:
                self.logger.error(f"Error running controller: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup resources."""
        self.logger.info("Cleaning up...")

        self.gen_joint_plots()

        self.gen_interpolate_plots()

        if self.joint_plotter:
            self.joint_plotter.close()

        if self.interpolator_plotter:
            self.interpolator_plotter.close()

        if self.interface:
            if self.mode == "real":
                self.go_home_in_cleanup()
            self.interface.disconnect()
            self.logger.info("✓ Robot disconnected")

        self.lcm_handler.disconnect()

        self.logger.info("✓ Cleanup completed")

    def go_home_in_cleanup(self):
        self.go_home_start_position = self.interface.get_joint_positions()
        self.go_home_start_time = time.time()
        elapsed_time = time.time() - self.go_home_start_time
        duration = self.movement_parameters["movement_duration"]
        while elapsed_time < duration:
            self.qPos_command = lerp(
                self.go_home_start_position,
                self.home_position,
                elapsed_time,
                duration,
            )
            self.interface.set_joint_positions(self.qPos_command)
            elapsed_time = time.time() - self.go_home_start_time

    def gen_joint_plots(self):
        """Generate joint plots."""
        if self.joint_plotter and self.joint_plot_flag:
            try:
                self.joint_plotter.make_plots(self.mode, self.show_plots)
            except Exception as e:
                self.logger.error(f"Joint plot generation failed: {e}")
        else:
            if not self.joint_plotter:
                self.logger.info(
                    "No joint plotter initialized - joint plotting was disabled"
                )
            if not self.joint_plot_flag:
                self.logger.info(
                    "Joint plot flag is False - joint plotting was disabled"
                )

    def gen_interpolate_plots(self):
        """Generate interpolator plots."""
        if self.interpolator_plotter and self.interpolator_plot_flag:
            try:
                self.interpolator_plotter.make_plots(self.mode, self.show_plots)
            except Exception as e:
                self.logger.error(f"Interpolator plot generation failed: {e}")
        else:
            if not self.interpolator_plotter:
                self.logger.info(
                    "No interpolator plotter initialized - interpolator plotting was disabled"
                )
            if not self.interpolator_plot_flag:
                self.logger.info(
                    "Interpolator plot flag is False - interpolator plotting was disabled"
                )
