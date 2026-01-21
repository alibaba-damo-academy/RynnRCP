#!/usr/bin/env python3
"""
RM75 Motion Controller - 7-DOF robot arm + gripper

Provides predefined motion patterns (motion 1-5) for testing and demonstration.
"""

import os
import sys
import time
import argparse
import logging
import yaml
import numpy as np
from pathlib import Path

from rcp_motion.core.robotinterface_base import robotinterface_factory
from rcp_motion.core.robot_manager import RobotManager
from rcp_motion.utils.policy_interpolator import lerp
from rcp_motion.core.data.robot_state import RobotState
from rcp_motion.robots.realman.interface.rm75_interface import create_robot_interface


def get_models_root():
    """Get models root path for RynnRCP."""
    default_path = Path("/home/oo/Documents/Damowork/RynnMotion/models")
    if default_path.exists():
        return default_path
    # Try RynnRCP local models
    rcp_models = Path(__file__).parent.parent.parent.parent / "models"
    if rcp_models.exists():
        return rcp_models
    return default_path


class RM75MotionController:
    """7-DOF robot controller for predefined motion patterns."""

    def __init__(
        self,
        mode: str = "sim",
        motion: int = 0,
        frequency: int = 100,
    ):
        self.mode = mode
        self.motion = motion
        self.frequency = max(30, min(frequency, 250))
        self.timestep = 1.0 / self.frequency
        self.config_path = "configs/rm75.yaml"
        self.config = self._load_config()

        self._init_logging()

        self.movement_parameters = {
            "movement_duration": 2.0,
            "amplitude": 0.5,
            "frequency": 0.2,
        }

        self.initial_joint_positions = None
        self.qPos_command = None
        self.robot_command = None
        self.interface = None
        self.robot_model = None

        self._init_robot_model()
        self._init_interface()

    def _load_config(self) -> dict:
        try:
            with open(self.config_path, "r") as f:
                config = yaml.safe_load(f)
            return config if config else {}
        except FileNotFoundError:
            return {}

    def _init_logging(self):
        from datetime import datetime

        log_dir = os.path.expanduser(
            self.config.get("logging", {}).get("log_dir", "~/logs/rm75")
        )
        log_file = self.config.get("logging", {}).get("log_file", "rm75_motion.log")
        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        log_path = os.path.join(log_dir, f"{log_file.replace('.log', '')}_{timestamp}.log")

        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s [%(levelname)s] %(message)s",
            handlers=[logging.FileHandler(log_path)],
        )
        self.logger = logging.getLogger(__name__)

    def _init_robot_model(self):
        if self.mode != "sim":
            return

        models_root = get_models_root()

        robotmodel_config = {
            "robot_name": "rm75",
            "robot_control_freq": self.frequency,
            "robot_mjcf": str(models_root / "realman/rm75/scene/scene.xml"),
            "pino_mjcf": str(models_root / "realman/rm75/urdf/rm75.xml"),
        }
        self.robot_model = RobotManager(robotmodel_config, self.logger)

    def _init_interface(self):
        if self.mode == "sim":
            self.interface = robotinterface_factory(
                "mujoco_sim_robot",
                self.robot_model,
                {"timestep": self.timestep},
            )
        else:
            self.interface = create_robot_interface(
                name="rm75",
                mode=self.mode,
                config_path=self.config_path,
            )

        self.interface.connect()
        self.initial_joint_positions = self.interface.get_joint_positions()

        if self.mode == "sim":
            self.robot_command = RobotState()
            self.robot_command.num_joints = self.interface.mdof

    def motion_planner(self, motion: int, current_time: float) -> np.ndarray:
        duration = self.movement_parameters["movement_duration"]
        amplitude = self.movement_parameters["amplitude"]
        frequency = self.movement_parameters["frequency"]

        # RM75 has 7 joints + gripper = 8 DOF
        if motion == 1:
            target = np.array([0.0, -np.pi/4, 0.0, np.pi/4, 0.0, np.pi/4, 0.0, 0.0])
            if current_time <= duration:
                return lerp(
                    self.initial_joint_positions.copy(), target, current_time, duration
                )
            else:
                t = current_time - duration
                target[0] += amplitude * np.sin(2 * np.pi * frequency * t)
                target[1] += 0.5 * amplitude * np.sin(3 * np.pi * frequency * t)
                target[3] += 0.5 * amplitude * np.sin(2 * np.pi * frequency * t)
                return target

        elif motion == 2:
            if current_time <= duration:
                target = np.array([amplitude, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                return lerp(
                    self.initial_joint_positions.copy(), target, current_time, duration
                )
            else:
                target = np.zeros(8)
                t = current_time - duration
                target[0] += amplitude * np.cos(2 * np.pi * frequency * t)
                target[2] += amplitude * np.sin(2 * np.pi * frequency * t)
                return target

        elif motion == 3:
            if current_time <= duration:
                target = np.array([0.0, -np.pi/3, 0.0, np.pi/2, 0.0, np.pi/3, 0.0, 0.0])
                return lerp(
                    self.initial_joint_positions.copy(), target, current_time, duration
                )
            else:
                target = np.array([0.0, -np.pi/3, 0.0, np.pi/2, 0.0, np.pi/3, 0.0, 0.0])
                t = current_time - duration
                target[0] += amplitude * np.cos(2 * np.pi * frequency * t)
                target[4] += amplitude * np.sin(2 * np.pi * frequency * t)
                target[6] += amplitude * np.cos(2 * np.pi * frequency * t)
                return target

        elif motion == 4:
            target = np.array([np.pi/4, -np.pi/4, 0.0, np.pi/2, 0.0, np.pi/4, 0.0, 0.0])
            if current_time <= duration:
                return lerp(
                    self.initial_joint_positions.copy(), target, current_time, duration
                )
            else:
                t = current_time - duration
                target[1] += amplitude * np.sin(2 * np.pi * frequency * t)
                target[3] += amplitude * np.sin(2 * np.pi * frequency * t)
                target[5] += amplitude * np.sin(2 * np.pi * frequency * t)
                return target

        elif motion == 5:
            target = np.array([0.0, -np.pi/6, 0.0, np.pi/3, 0.0, np.pi/6, 0.0, 0.0])
            return lerp(
                self.initial_joint_positions.copy(), target, current_time, duration
            )

        else:
            return self.initial_joint_positions.copy()

    def run(self):
        self.logger.info(f"RM75 Motion Controller: mode={self.mode}, motion={self.motion}, freq={self.frequency}Hz")

        try:
            while True:
                if self.mode == "sim" and not self.interface.is_viewer_alive():
                    break

                precall_wall_time = time.time()
                current_time = self.interface.get_current_time()

                self.qPos_command = self.motion_planner(self.motion, current_time)

                if self.qPos_command is not None:
                    if self.mode == "sim":
                        for i in range(len(self.qPos_command)):
                            self.robot_command.joint_pos[i] = self.qPos_command[i]
                        self.interface.set_robot_command(self.robot_command)
                    else:
                        self.interface.set_joint_positions(self.qPos_command)

                self.interface.step()

                sleep_time = self.timestep - (time.time() - precall_wall_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    def cleanup(self):
        if self.interface:
            self.interface.disconnect()


def main():
    parser = argparse.ArgumentParser(description="RM75 Motion Controller - Motion Patterns (1-5)")
    parser.add_argument("--mode", choices=["sim", "real", "mock"], default="sim")
    parser.add_argument("--motion", type=int, choices=[1, 2, 3, 4, 5], default=1)
    parser.add_argument("--frequency", type=int, default=100)
    args = parser.parse_args()

    controller = RM75MotionController(
        mode=args.mode,
        motion=args.motion,
        frequency=args.frequency,
    )
    controller.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
