#!/usr/bin/env python3
"""
SO101 Inference Controller - Standalone LCM Inference Controller for SO101 Robot

Bridges inference (policy output) to control (robot execution) via LCM.
Uses new MujocoRobotInterface factory pattern for simulation.

Usage:
    python -m so101.controller.so101_inference -m real -f 100
    python -m so101.controller.so101_inference -m sim -f 50
"""

import os
import sys
import time
import signal
import logging
import argparse
import threading
import numpy as np
import yaml
import platform
from datetime import datetime
from pathlib import Path

if platform.system() == "Darwin":
    import matplotlib

    matplotlib.use("Agg")

from rcp_motion.core.mjrobot_interface import MujocoRobotInterface
from rcp_motion.core.robot_manager import RobotManager

from rcp_motion.utils.policy_interpolator import PolicyInterpolator, lerp
from rcp_motion.utils.lcm_handler import LCMHandler
from rcp_motion.utils.joint_plotter import JointPlotter
from rcp_motion.utils.interpolator_plotter import InterpolatorPlotter
from rcp_motion.robots.so101.lcm_msgs.state_feedback import state_feedback
from rcp_motion.robots.so101.interface.so101_interface import create_robot_interface


def get_models_root():
    """Get models root path for RynnRCP."""
    # RynnRCP models are under rcp_motion/models/
    return Path(__file__).parent.parent.parent.parent / "models"


def get_config_path(robot_name):
    """Get the path to the so101.yaml config file."""
    import rcp_motion

    package_dir = Path(rcp_motion.__file__).parent
    config_filename = str(robot_name) + ".yaml"
    return package_dir / "robots" / "so101" / "configs" / config_filename


def handle_signal(signum, frame):
    print(f"\nReceived signal {signum}: {signal.Signals(signum).name}")
    if signum in (signal.SIGINT, signal.SIGTERM):
        print("Exiting gracefully")
        raise KeyboardInterrupt


signal.signal(signal.SIGINT, handle_signal)
signal.signal(signal.SIGTERM, handle_signal)


class SO101Inference:
    """SO101 LCM inference controller for policy execution."""

    def __init__(self, mode: str = "sim", frequency: int = 100, robot_name: str = "so101"):
        self.mode = mode
        self.frequency = max(30, min(frequency, 250))
        self.robot_name = robot_name
        self.timestep = 1.0 / self.frequency
        self.config_path = str(get_config_path(self.robot_name))
        self.calibration_dir = None  # Use calibration_dir from config file

        self._init_config()
        self._init_logging()
        self._init_state()
        self._init_interpolator()
        self._init_interface()
        self._init_communication()
        self._init_joint_plotting()
        self._init_interpolator_plotting()

    def _init_config(self):
        try:
            with open(self.config_path, "r") as f:
                self.config = yaml.safe_load(f)
        except FileNotFoundError:
            self.config = {}

        self.movement_parameters = {
            "movement_duration": 2,
            "amplitude": 0.3,
            "frequency": 0.4,
        }

        self.home_position = np.array(
            self.config.get(
                "home_joint_positions", [0.0, -np.pi / 2, 1.69, 1.1, 0.0, 0.0]
            )
        )

        self.inference_rate = self.config.get("robot", {}).get("inference_rate", 30.0)
        self.joint_plot_flag = self.config.get("plot", {}).get(
            "enable_joint_plotting", False
        )
        self.interpolator_plot_flag = self.config.get("plot", {}).get(
            "enable_interpolator_plotting", False
        )
        self.show_plots = self.config.get("plot", {}).get("show_plots", False)

    def _init_logging(self):
        log_dir = os.path.expanduser(
            self.config.get("logging", {}).get("log_dir", "~/logs/lerobot")
        )
        log_file = self.config.get("logging", {}).get("log_file", "infer2control.log")
        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        log_path = os.path.join(
            log_dir, f"{log_file.replace('.log', '')}_{timestamp}.log"
        )

        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s [%(levelname)s] %(message)s",
            handlers=[logging.FileHandler(log_path)],
        )
        self.logger = logging.getLogger(__name__)

    def _init_state(self):
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

        # --- External/API command heartbeat (for direct set_joint_positions control) ---
        self.last_external_command_time = None
        self.has_external_command = False

        # --- Thread control ---
        self._stop_event = None
        self._thread = None

    def notify_command_received(self) -> None:
        self.last_external_command_time = time.time()
        self.has_external_command = True

    def is_external_command_timeout(self, timeout_seconds: float) -> bool:
        if not self.has_external_command or self.last_external_command_time is None:
            return False
        return (time.time() - self.last_external_command_time) > timeout_seconds

    def _init_interpolator(self):
        self.interpolator = PolicyInterpolator(self.config_path)
        self.latest_ACT = None
        self.executing_ACT = None
        self.last_processed_seq = -1
        self.current_action_count = 0

    def _init_interface(self):
        if self.mode == "sim":
            models_root = get_models_root()
            if self.robot_name == "so101":
                scene_path = str(models_root / "lerobot/so101/scene/scene.xml")
                pino_path = str(models_root / "lerobot/so101/mjcf/so101_pinocchio.xml")
            elif self.robot_name == "lekiwi":
                scene_path = str(models_root / "lerobot/lekiwi/scene/scene.xml")
                pino_path = str(models_root / "lerobot/lekiwi/urdf/LeKiwi.xml")

            robot_config = {
                "robot_name": self.robot_name,
                "robot_control_freq": self.frequency,
                "robot_mjcf": scene_path,
                "pino_mjcf": pino_path,
            }

            robot_model = RobotManager(robot_config, self.logger)
            self.interface = MujocoRobotInterface(robot_model, {}, self.logger)
            self.interface.connect()

            robot_state = self.interface.get_robot_state_feedbacks()
            self.initial_joint_positions = np.array(robot_state.joint_pos)
        else:
            self.interface = create_robot_interface(
                name=self.robot_name,
                mode=self.mode,
                config_path=self.config_path,
            )
            self.interface.init(timestep=self.timestep)
            self.initial_joint_positions = self.interface.get_joint_positions()

    def _init_communication(self):
        self.lcm_handler = LCMHandler(self.logger)
        self.lcm_handler.connect()

    def _init_joint_plotting(self):
        if not self.joint_plot_flag:
            self.joint_plotter = None
            return

        num_joints = len(self.interface.get_joint_positions())
        joint_names = [f"q_{i}" for i in range(num_joints)]
        log_dir = self.config.get("logging", {}).get("log_dir", "~/logs/lerobot")
        self.joint_plotter = JointPlotter(joint_names, log_dir=log_dir)

    def _init_interpolator_plotting(self):
        if not self.interpolator_plot_flag:
            self.interpolator_plotter = None
            return

        num_joints = len(self.interface.get_joint_positions())
        joint_names = [f"q_{i}" for i in range(num_joints)]
        log_dir = self.config.get("logging", {}).get("log_dir", "~/logs/lerobot")
        self.interpolator_plotter = InterpolatorPlotter(joint_names, log_dir=log_dir)

    def _pre_time_handling(self):
        precall_time = self.interface.get_current_time()
        precall_wall_time = time.time()
        nextcall_time = precall_time + self.timestep
        return precall_time, precall_wall_time, nextcall_time

    def _post_time_handling(self, precall_wall_time):
        sleep_time = self.timestep - (time.time() - precall_wall_time)
        if sleep_time > 0:
            time.sleep(sleep_time)

    def get_state(self):
        # LCM explicit gohome request
        if self.lcm_handler and self.lcm_handler.check_gohome_request():
            return "go_home"

        # Unified timeout: LCM timeout OR API/direct timeout
        lcm_timeout = (
            self.lcm_handler.is_command_timeout(self.timeout_seconds)
            if self.lcm_handler
            else False
        )
        api_timeout = self.is_external_command_timeout(self.timeout_seconds)

        if lcm_timeout or api_timeout:
            return "go_home"

        # If no ACT is executing, but someone called set_joint_positions(),
        # we should hold/use qPos_command and let loop send it.
        if self.executing_ACT is None and self.qPos_command is not None:
            return "direct"

        return "policy"

    def _get_feedbacks(self):
        if self.mode == "sim":
            robot_state = self.interface.get_robot_state_feedbacks()
            self.qPos_feedback = np.array(robot_state.joint_pos)
            self.qdFb = np.array(robot_state.joint_vel)
        else:
            self.qPos_feedback = self.interface.get_joint_positions()
            self.qdFb = self.interface.get_joint_velocities()

    def _lcm_subscribing(self):
        if self.lcm_handler:
            self.lcm_handler.process_messages(0)

    def _run_policy(self):
        latest_ACT = self.lcm_handler.get_latest_ACT()
        if latest_ACT and latest_ACT.seq != self.last_processed_seq:
            self._process_new_chunk(latest_ACT)

        if self.executing_ACT and self.current_action_count < self.highfreq_chunksize:
            self.update()

    def _process_new_chunk(self, new_chunk):
        self.latest_ACT = new_chunk
        self.executing_ACT = new_chunk
        self.last_processed_seq = new_chunk.seq
        self.current_action_count = 0
        self.notify_command_received()

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
            qACT, total_dof, self.executing_ACT.chunkSize
        )

        self.lowfreq_chunksize = self.executing_ACT.chunkSize
        self.highfreq_chunksize = (
            self.frequency / self.inference_rate * self.executing_ACT.chunkSize
        )

        if self.interpolator_plot_flag and self.interpolator_plotter:
            self._store_interpolator_plot_data(qACT, total_dof)

    def _store_interpolator_plot_data(self, qACT, total_dof):
        sampled_joint_positions = []
        high_freq_indices = np.linspace(
            0, self.lowfreq_chunksize - 1, int(self.highfreq_chunksize)
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
        if self.lcm_handler:
            self.lcm_handler.publish_state_feedback_with_status(state_feedback.kSuccess)
            self.lcm_handler.set_act_status(state_feedback.kIdle)
        self.executing_ACT = None

    def _run_go_home(self):
        if self.lcm_handler is None:
            return

        current_time = time.time()

        if self.go_home_start_time is None:
            self.go_home_start_position = self.interface.get_joint_positions()
            self.go_home_start_time = current_time

        elapsed_time = current_time - self.go_home_start_time
        duration = self.movement_parameters["movement_duration"]

        if elapsed_time >= duration:
            self.go_home_start_time = None
            self.go_home_start_position = None
            if self.lcm_handler:
                self.lcm_handler.gohome_requested = False
                self.lcm_handler.has_received_command = False
                self.lcm_handler.set_act_status(state_feedback.kIdle)

            # reset API heartbeat to avoid re-triggering go_home repeatedly
            self.has_external_command = False
            self.last_external_command_time = None
            return

        if self.home_position is None:
            target_position = np.zeros(len(self.go_home_start_position))
        elif elapsed_time < duration / 2.0:
            target_position = self.home_position.copy()
            target_position[len(self.home_position) - 1] = 1
        else:
            target_position = self.home_position.copy()
            target_position[len(self.home_position) - 1] = 0

        self.qPos_command = lerp(
            self.go_home_start_position, target_position, elapsed_time, duration
        )

    def get_joint_positions(self) -> np.ndarray:
        """Get the current joint positions from the robot (latest feedback)."""
        if self.qPos_feedback is None:
            # fallback if loop not started
            return np.asarray(self.interface.get_joint_positions(), dtype=float)
        return np.asarray(self.qPos_feedback, dtype=float)

    def set_joint_positions(self, positions: np.ndarray) -> None:
        """
        Set the joint positions command (direct control).
        NOTE: do not directly call interface here; let loop send it in _set_commands().
        """
        arr = np.asarray(positions, dtype=float).copy()
        self.qPos_command = arr
        self.notify_command_received()

    def _set_commands(self):
        if self.qPos_command is not None:
            if self.mode == "sim":
                self.interface.qCmd[:] = self.qPos_command
                self.interface.set_joint_commands()
            else:
                self.interface.set_joint_positions(self.qPos_command)

    def _offline_joint_plot(self):
        if (
            self.joint_plot_flag
            and self.joint_plotter
            and self.qPos_command is not None
        ):
            current_time = self.interface.get_current_time()
            self.joint_plotter.add_data(
                self.qPos_command, self.qPos_feedback, current_time
            )

    def _lcm_publishing(self):
        if not self.lcm_handler:
            return
        if self.lcm_handler.check_robot_feedback_request():
            self.lcm_handler.publish_robot_feedback(self.qPos_feedback, None)
            if self.interface.name == "lekiwi":
                if self.qPos_command is not None:
                    self.qPos_command[6] = 0
                    self.qPos_command[7] = 0
                    self.qPos_command[8] = 0
                    self.interface.set_joint_positions(self.qPos_command)
        if self.lcm_handler.check_state_feedback_request():
            self.lcm_handler.publish_state_feedback()

    def run(self):
        self.logger.info(
            f"SO101 Inference Controller: mode={self.mode}, freq={self.frequency}Hz"
        )

        try:
            while True:
                _precall_time, precall_wall_time, _nextcall_time = (
                    self._pre_time_handling()
                )

                self._get_feedbacks()
                self._lcm_subscribing()

                state = self.get_state()
                if state == "go_home":
                    self._run_go_home()
                elif state == "policy":
                    self._run_policy()
                elif state == "direct":
                    # qPos_command already set via API; nothing to compute here
                    pass

                self._set_commands()
                self.interface.step()

                self._offline_joint_plot()
                self._lcm_publishing()

                if self.mode == "sim" and not self.interface.is_viewer_alive():
                    break

                self._post_time_handling(precall_wall_time)

        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    def cleanup(self):
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
        if self.lcm_handler:
            self.lcm_handler.disconnect()

    def start(self) -> None:
        """Run controller loop in a background thread (non-blocking)."""
        if self._thread and self._thread.is_alive():
            return

        self._stop_event = threading.Event()

        def _loop():
            try:
                while not self._stop_event.is_set():
                    _precall_time, precall_wall_time, _nextcall_time = (
                        self._pre_time_handling()
                    )

                    self._get_feedbacks()
                    self._lcm_subscribing()

                    state = self.get_state()
                    if state == "go_home":
                        self._run_go_home()
                    elif state == "policy":
                        self._run_policy()
                    elif state == "direct":
                        pass

                    self._set_commands()
                    self.interface.step()

                    self._offline_joint_plot()
                    self._lcm_publishing()

                    if self.mode == "sim" and not self.interface.is_viewer_alive():
                        break

                    self._post_time_handling(precall_wall_time)

            except Exception as e:
                if hasattr(self, "logger"):
                    self.logger.error(f"Controller thread error: {e}")
            finally:
                self.cleanup()

        self._thread = threading.Thread(target=_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Request controller thread to stop and cleanup."""
        if self._stop_event is None:
            return
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)

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
        if self.joint_plotter and self.joint_plot_flag:
            try:
                self.joint_plotter.make_plots(self.mode, self.show_plots)
            except Exception:
                pass

    def gen_interpolate_plots(self):
        if self.interpolator_plotter and self.interpolator_plot_flag:
            try:
                self.interpolator_plotter.make_plots(self.mode, self.show_plots)
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(
        description="SO101 Inference - LCM Policy Controller"
    )
    parser.add_argument("-m", "--mode", choices=["sim", "real", "mock"], default="sim")
    parser.add_argument("-f", "--frequency", type=int, default=100)
    parser.add_argument("-n", "--robot_name", type=str, default="so101")
    args = parser.parse_args()

    controller = SO101Inference(mode=args.mode, frequency=args.frequency, robot_name=args.robot_name)
    controller.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
