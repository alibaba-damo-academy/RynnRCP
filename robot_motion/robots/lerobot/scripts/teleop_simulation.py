#!/usr/bin/env python3
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
import time

import logging
import yaml

from utils.lcm_handler import LCMHandler
from interface.unified_interface import create_robot_interface


class TeleopSimulation:
    """
    simulate teleoperation in mujoco, using LCM for communication with real robot.
    """

    def __init__(
        self,
        mode: str = "sim",
        frequency: int = 100,
        config_path: str = "configs/config.yaml",
    ):
        """
        Initialize the teleoperation.

        Args:
            mode: teleoperation mode - "sim" for keyboard input, "real" for real robot input
        """
        self.mode = mode
        self.frequency = max(30, min(frequency, 250))
        self.timestep = 1.0 / self.frequency
        self.config_path = config_path

        self.init_config()
        self.init_logging()
        self.init_state()
        self.init_interface()
        self.init_communication()

    def init_config(self):
        """Initialize configuration."""
        self.config = self.load_config(self.config_path)

    def init_logging(self):
        """Setup logging configuration."""
        log_dir = os.path.expanduser("~/logs/lerobot")
        log_file = self.config.get("logging", {}).get(
            "log_file", "teleop_simulation.log"
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
        """Initialize planner state variables."""
        self.current_joint_positions = None
        self.command_joint_positions = None
        self.realrobot_joint_fb = None

    def init_interface(self):
        """Setup interface - now unified for all modes!"""
        try:
            calibration_file = ".cache/calibration/so100/main_follower.json"
            self.simrobot_interface = create_robot_interface(
                mode="sim",
                config_path=self.config_path,
                calibration_file=None,
            )
            self.realrobot_interface = create_robot_interface(
                mode="real",
                config_path=self.config_path,
                calibration_file=calibration_file,
            )

            self.simrobot_interface.init(timestep=self.timestep)
            self.realrobot_interface.init(timestep=self.timestep)

            self.simrobot_interface.set_camera(
                distance=1.0,
                azimuth=45.0,
                elevation=-20.0,
                lookat=[0.0, -0.2, 0.2],
            )
            self.logger.info("✓ Camera configured for simulation")

            self.realrobot_interface.enable_torque(False)

            self.initial_joint_positions = self.realrobot_interface.get_joint_positions(
                radians=True
            )
            self.logger.info(f"Initial joint positions: {self.initial_joint_positions}")
            self.logger.info(f"✓ Robot interface ready in {self.mode} mode")

        except FileNotFoundError as e:
            self.logger.error(f"✗ Setup failed - missing calibration file:")
            self.logger.error(str(e))
            raise
        except Exception as e:
            self.logger.error(f"✗ Failed to setup interface: {e}")
            raise

    def init_communication(self):
        """Setup LCM communication handler."""
        self.lcm_handler = LCMHandler(self.logger)
        if self.lcm_handler.connect():
            self.logger.info("✓ LCM handler initialized")
        else:
            self.logger.warning("⚠ LCM handler initialization failed")

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
        precall_time = self.simrobot_interface.get_current_time()
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

    def _get_joint_positions_from_realbot(self):
        """Get current joint positions from real robot and use this state to update the simulation state."""
        self.command_joint_positions = self.realrobot_interface.get_joint_positions(
            radians=True
        )

    def _lcm_subscribing(self):
        """Handle LCM message subscription and processing."""
        if self.lcm_handler and self.lcm_handler.is_connected():
            self.lcm_handler.process_messages(0)

    def _set_commands(self):
        """Send joint position commands to the simulation robot."""
        if self.command_joint_positions is not None:
            self.simrobot_interface.set_joint_positions(
                self.command_joint_positions, radians=True
            )

    def _lcm_publishing(self):
        """Handle LCM feedback publishing."""
        if not self.lcm_handler:
            return

        # Handle robot feedback requests
        if self.lcm_handler.check_robot_feedback_request():
            self.lcm_handler.publish_robot_feedback(self.realrobot_joint_fb)

        # Handle state feedback requests
        if self.lcm_handler.check_state_feedback_request():
            self.lcm_handler.publish_state_feedback()

    def run(self):
        """Run the robot controller."""
        logging.info(f"LeRobot Unified Framework - Mode: {self.mode}")

        try:
            while True:
                precall_time, precall_wall_time, nextcall_time = (
                    self._pre_time_handling()
                )
                self._get_joint_positions_from_realbot()
                self._lcm_subscribing()
                self._set_commands()
                self.simrobot_interface.step()
                self.realrobot_interface.step()
                self._lcm_publishing()
                self._post_time_handling(precall_wall_time)

        except KeyboardInterrupt:
            logging.info("\nStopping robot planner...")
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup resources."""
        self.logger.info("Cleaning up...")

        if self.realrobot_interface:
            self.realrobot_interface.disconnect()
            self.logger.info("✓ Robot disconnected")

        if self.lcm_handler:
            self.lcm_handler.disconnect()

        self.logger.info("✓ Cleanup completed")


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
        "--config",
        type=str,
        default="configs/config.yaml",
        help="Path to configuration file",
    )

    args = parser.parse_args()

    logging.info(f"Mode: {args.mode}")

    teleop_planner = TeleopSimulation(
        mode=args.mode,
        frequency=args.ctrlfreq,
        config_path=args.config,
    )
    teleop_planner.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
