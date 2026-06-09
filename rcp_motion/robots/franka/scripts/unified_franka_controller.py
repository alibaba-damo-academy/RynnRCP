#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
teleoperation Test Script for LeRobot Motion Scripts

This script simulate teleoperate in mujoco,
you can define your own scene for your task in simulation,
using LCM for communication with real robot.

Usage:
    cd robots/lerobot
    python -m scripts.teleop_simulation --mode [sim | real]
"""

import os
import sys
import argparse
import numpy as np
import math
import time

import logging
import yaml

from rcp_motion.utils.lcm_handler import LCMHandler
from interface.simrobot_interface import create_sim_robot_interface
from interface.franka_interface import create_franka_interface
from rcp_motion.utils.simulation_command_gen import SimCommandGenerator
from rcp_motion.utils.rt_trajgen import RealtimeTrajGenRuckig
from rcp_motion.utils.joint_plotter import JointPlotter

from common.lcm.lcmMotion.state_feedback import state_feedback


class FrankaController:
    """
    simulate teleoperation in mujoco, using LCM for communication with real robot.
    """

    def __init__(
        self,
        mode: str = "sim",
        signal_source: str = "sim",
        frequency: int = 100,
        config_path: str = "configs/config.yaml",
    ):
        """
        Initialize the teleoperation.

        Args:
            mode: teleoperation mode - "sim" for keyboard input, "real" for real robot input
        """
        self.mode = mode
        self.frequency = max(30, min(frequency, 1000))
        self.timestep = 1.0 / self.frequency
        self.config_path = config_path
        self.signal_source = signal_source
        self.signal_handler = None

        self.init_config()
        self.init_logging()
        self.init_signal_state()
        self.init_communication()
        self.init_rt_trajgen()
        self.init_state()
        self.init_interface()
        self.init_joint_plotting()

    def init_config(self):
        """Initialize configuration."""
        self.config = self.load_config(self.config_path)
        self.robot_config = self.config.get("robot", {})
        self.signal_config = self.config.get("signal", {})

        self.inference_rate = self.robot_config.get("inference_rate", 30.0)
        self.frequency = max(self.inference_rate, self.frequency)
        self.lowloop_saturation = self.frequency / self.inference_rate
        self.signal_chunk_size = self.signal_config.get("chunk_size", 20)
        self.robot_dof = len(self.robot_config.get("joint_names", []))

    def init_logging(self):
        """Setup logging configuration."""
        log_dir = os.path.expanduser(
            self.config.get("logging", {}).get("log_dir", "~/logs/franka")
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
        print(f"[OK] Log file: {log_path}")

    def init_signal_state(self):
        """Initialize signal state variables."""
        self.last_seq_flag = -1
        self.current_action_count = 0
        self.new_signal = False
        self.pub_lcm_state = True
        self.signal_trajectory = None
        if self.signal_source not in ["sim", "policy"]:
            raise ValueError(f"Invalid signal source: {self.signal_source}")

        if self.signal_source == "sim":

            self.signal_handler = SimCommandGenerator(
                dof=self.robot_dof,
                chunk_size=self.signal_chunk_size,
                update_freq=self.frequency,
                command_freq=self.inference_rate,
            )
            signal_params = self.signal_config
            self.signal_handler.set_traj_parameter(
                home_position=signal_params.get(
                    "signal_home_position", [0, 0, 0, -1.57079, 0, 1.57079, -0.7853]
                ),
                signal_amplitude=signal_params.get("signal_amplitude", 0.1),
                signal_frequency=signal_params.get("signal_frequency", 1),
                block_time=signal_params.get("block_time", 0.1),
                signal_type=signal_params.get("signal_type", "sin"),
                random_seed=signal_params.get("random_seed", None),
                random_ratio=signal_params.get("random_ratio", 0.1),
            )
        elif self.signal_source == "policy":
            self.signal_handler = None

        self.logger.info(f"[OK] signal ready in {self.signal_source} mode")
        self.logger.info(f"      signal freq: {self.inference_rate}")
        self.logger.info(f"      chunk size: {self.signal_chunk_size}")
        # self.logger.info(f"      block size: {self.block_count}")

    def init_rt_trajgen(self):
        """Initialize real-time trajectory generator."""
        self.first_get_fb = True
        self.rt_trajgen = RealtimeTrajGenRuckig(
            dof=self.robot_dof,
            input_freq=self.inference_rate,
            output_freq=self.frequency,
        )
        self.rt_trajgen.set_position_limits(
            position_upper_limits=self.robot_config.get("joint_limit_upper", None),
            position_lower_limits=self.robot_config.get("joint_limit_lower", None),
        )
        ratio = self.robot_config.get("vel_limit_ratio", 0.1)
        joint_vel_limit = self.robot_config.get("joint_vel_limit", None)
        joint_acc_limit = self.robot_config.get("joint_acc_limit", None)
        joint_jerk_limit = self.robot_config.get("joint_jerk_limit", None)
        self.rt_trajgen.set_robot_ability(
            velocity_limits=np.multiply(joint_vel_limit, ratio),
            acceleration_limits=np.multiply(joint_acc_limit, ratio),
            jerk_limits=np.multiply(joint_jerk_limit, ratio),
        )
        self.logger.info(f"[OK] real-time trajecory generation seccessfully!")
        self.logger.info(f"       velocity limit: {joint_vel_limit}")
        self.logger.info(f"       acceleration limit: {joint_acc_limit}")
        self.logger.info(f"       jerk limit: {joint_jerk_limit}")

    def init_state(self):
        """Initialize planner state variables."""
        self.current_joint_command = None
        self.command_joint_positions = None
        self.robot_joint_fb = None
        self.ftsensor_fb = None

        self.timeout_seconds = self.config.get("robot", {}).get("timeout_seconds", 30.0)
        self.timeout_seconds = max(20.0, min(self.timeout_seconds, 120.0))
        self.home_position = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853, 0.0])
        self.go_home_start_position = None
        self.go_home_start_time = None

    def init_interface(self):
        """Setup interface - now unified for all modes!"""
        try:
            if self.mode == "real":
                self.robot_interface = create_franka_interface(
                    config_path=self.config_path,
                    calibration_file=None,
                )
            else:
                self.robot_interface = create_sim_robot_interface(
                    config_path=self.config_path,
                    calibration_file=None,
                )

            self.robot_interface.init(timestep=self.timestep)

            self.initial_joint_positions = self.robot_interface.get_joint_positions()

            if self.mode == "real":
                self.robot_interface.enable_torque(True)

            self.logger.info(f"Initial joint positions: {self.initial_joint_positions}")
            self.logger.info(f"[OK] Robot interface ready in {self.mode} mode")

        except FileNotFoundError as e:
            self.logger.error(f"[ERR] Setup failed - missing calibration file:")
            self.logger.error(str(e))
            raise
        except Exception as e:
            self.logger.error(f"[ERR] Failed to setup interface: {e}")
            raise

    def init_communication(self):
        """Setup LCM communication handler."""
        self.lcm_handler = LCMHandler(self.logger)
        if self.lcm_handler.connect():
            self.logger.info("[OK] LCM handler initialized")
        else:
            self.logger.warning("[WARN] LCM handler initialization failed")

    def init_joint_plotting(self):
        """Initialize offline joint plotting if enabled."""
        self.joint_plot_flag = self.config.get("plot", {}).get(
            "enable_joint_plotting", False
        )

        if self.mode == "real":
            self.joint_plot_flag = False

        if not self.joint_plot_flag:
            self.joint_plotter = None
            return

        joint_names = self.robot_interface.joint_names
        log_dir = self.config.get("logging", {}).get("log_dir", "~/logs/lerobot")
        self.joint_plotter = JointPlotter(joint_names, log_dir=log_dir)
        self.logger.info("[OK] Offline joint plotting initialized")

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
        precall_time = self.robot_interface.get_current_time()
        precall_wall_time = time.time()
        nextcall_time = precall_time + self.timestep
        return precall_time, precall_wall_time, nextcall_time

    def _post_time_handling(self, precall_wall_time):
        """Handle timing synchronization at the end of each control loop iteration."""
        consumed_time = time.time() - precall_wall_time
        sleep_time = self.timestep - consumed_time

        if sleep_time > 0:
            time.sleep(sleep_time)
        elif sleep_time < -0.001:
            self.logger.warning(f"Control loop running behind by {-sleep_time:.3f}s")

    def get_state(self):
        """Determine which state the robot should be in."""
        if self.lcm_handler and self.lcm_handler.check_gohome_request():
            return "go_home"

        if self.lcm_handler and self.lcm_handler.is_command_timeout(
            self.timeout_seconds
        ):
            return "go_home"

        return "policy"

    def _get_feedback(self):
        """update robot feedback state."""
        self.robot_joint_fb = self.robot_interface.get_joint_positions()
        self.ftsensor_fb = self.robot_interface.get_ftsensor_state()
        if self.robot_joint_fb is not None and self.first_get_fb:
            if self.current_joint_command is not None:
                self.rt_trajgen.reset_state(
                    self.robot_joint_fb, self.current_joint_command
                )
            else:
                self.rt_trajgen.reset_state(self.robot_joint_fb, self.robot_joint_fb)
            self.first_get_fb = False
        if self.robot_joint_fb is None:
            self.first_get_fb = True

    def _lcm_subscribing(self):
        """Handle LCM message subscription and processing."""
        if self.lcm_handler and self.lcm_handler.is_connected():
            self.lcm_handler.process_messages(0)

    def _process_lcm_trajectory(self, new_chunk):
        """Process a new trajectory chunk."""
        self.last_seq_flag = new_chunk.seq
        self.current_action_count = 0
        self.signal_chunk_size = new_chunk.chunkSize
        
        total_dof = new_chunk.numJoint + new_chunk.numGripper
        qACT = np.zeros((self.signal_chunk_size,total_dof))
        for i in range(self.signal_chunk_size):
            q_cur = []
            joint_start = i * new_chunk.numJoint
            joint_end = joint_start + new_chunk.numJoint
            q_cur.extend(new_chunk.jointPos[joint_start:joint_end])

            gripper_start = i * new_chunk.numGripper
            gripper_end = gripper_start + new_chunk.numGripper
            q_cur.extend(new_chunk.gripperPos[gripper_start:gripper_end])
            qACT[i,:] = np.array(q_cur)

        self.signal_trajectory = np.array(qACT)
        self.new_signal = True
        self.pub_lcm_state = True

    def _get_signal_command(self):
        """Get signal command."""
        if self.signal_source == "sim":
            signal_trajectory, seq = self.signal_handler.signal_update()
            if seq != self.last_seq_flag:
                self.signal_trajectory = signal_trajectory
                self.new_signal = True
                self.last_seq_flag = seq
                self.current_action_count = 0

        elif self.signal_source == "policy":
            latest_ACT = self.lcm_handler.get_latest_ACT()
            if latest_ACT and latest_ACT.seq != self.last_seq_flag:
                self._process_lcm_trajectory(latest_ACT)

    def _set_robot_commands(self):
        """Send joint position commands to the simulation robot."""
        if self.current_joint_command is not None and not self.first_get_fb:
            self.command_joint_positions = self.rt_trajgen.update()
            # self.command_joint_positions = self.current_joint_command
        if self.command_joint_positions is not None:
            self.robot_interface.set_joint_positions(
                self.command_joint_positions, radians=True
            )

    def _lcm_publishing(self):
        """Handle LCM feedback publishing."""
        if not self.lcm_handler:
            return

        # Handle robot feedback requests
        if self.lcm_handler.check_robot_feedback_request():
            #print("get lcm publish request:",self.robot_joint_fb,self.ftsensor_fb)
            self.lcm_handler.publish_robot_feedback(
                self.robot_joint_fb, self.ftsensor_fb
            )

        # Handle state feedback requests
        if self.lcm_handler.check_state_feedback_request():
            self.lcm_handler.publish_state_feedback()

    def _run_go_home(self):
        """Return to initial position from current position."""
        if self.lcm_handler is None:
            return
        
        # TODO: add go home function
        self.current_joint_command = self.home_position
        self.rt_trajgen.set_input_target(self.home_position)
        self.new_signal = False

    def _run_policy(self):
        """Execute LCM policy commands (scenario 0)."""
        lowloop_count = self.current_action_count / self.lowloop_saturation
        traj_index = min(self.signal_chunk_size - 1, max(0, math.floor(lowloop_count)))
        if self.signal_trajectory is not None:
            self.current_joint_command = self.signal_trajectory[traj_index, :]
            self.rt_trajgen.set_input_target(self.current_joint_command)
            self.new_signal = False
            if (traj_index == self.signal_chunk_size - 1) and self.pub_lcm_state:
               self.pub_lcm_state = False
               self.lcm_handler.publish_state_feedback_with_status(state_feedback.kSuccess) 
        self.current_action_count += 1

    def _offline_joint_plot(self):
        """Handle offline joint plotting data collection."""
        if not self.joint_plot_flag:
            return

        if self.joint_plotter and self.command_joint_positions is not None:
            current_time = self.robot_interface.get_current_time()
            self.joint_plotter.add_data(
                self.command_joint_positions, self.robot_joint_fb, current_time
            )

    def run(self):
        """Run the robot controller."""

        try:
            while True:
                precall_time, precall_wall_time, nextcall_time = (
                    self._pre_time_handling()
                )
                self._get_feedback()

                self._lcm_subscribing()

                self._get_signal_command()

                state = self.get_state()

                if state == "policy":
                    self._run_policy()
                else:
                    self._run_go_home()

                self._set_robot_commands()
                self.robot_interface.step()
                self._offline_joint_plot()
                self._lcm_publishing()
                self._post_time_handling(precall_wall_time)

        except (KeyboardInterrupt, Exception) as e:
            if isinstance(e, KeyboardInterrupt):
                self.logger.info("\nStopping robot controller...")
            else:
                self.logger.error(f"Error running controller: {e}")

        self.cleanup()

    def cleanup(self):
        """Cleanup resources."""
        self.logger.info("Cleaning up...")

        self.gen_joint_plots()

        if self.robot_interface:
            self.robot_interface.disconnect()
            self.logger.info("[OK] Robot disconnected")

        if self.lcm_handler:
            self.lcm_handler.disconnect()

        self.logger.info("[OK] Cleanup completed")

    def gen_joint_plots(self):
        """Generate joint plots."""
        if self.joint_plotter and self.joint_plot_flag:
            try:
                self.joint_plotter.make_plots(self.mode, True)
            except Exception as e:
                self.logger.error(f"Joint plot generation failed: {e}")
        else:
            if not self.joint_plotter:
                self.logger.info(
                    "No joint plotter initialized - joint plotting was disabled"
                )


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description="Unified LeRobot Test with LCM Communication"
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["sim", "real"],
        default="sim",
        help="Control mode: sim (keybord input), real (robot input)",
    )

    parser.add_argument(
        "--ctrlfreq", type=int, default=100, help="Control frequency in Hz (max: 250Hz)"
    )

    parser.add_argument(
        "--signal_source",
        type=str,
        choices=["sim", "policy"],
        default="sim",
        help="signal source: sim (simulation input), policy (lcm input)",
    )

    parser.add_argument(
        "--config",
        type=str,
        default="configs/config.yaml",
        help="Path to configuration file",
    )

    args = parser.parse_args()

    franka_ctrl = FrankaController(
        mode=args.mode,
        signal_source=args.signal_source,
        frequency=args.ctrlfreq,
        config_path=args.config,
    )
    franka_ctrl.run()


if __name__ == "__main__":
    sys.exit(main())
