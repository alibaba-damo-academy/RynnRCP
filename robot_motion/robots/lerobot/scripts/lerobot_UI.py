import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import traceback
import yaml
from typing import Dict, Optional
from copy import deepcopy

from robot_devices.motors.feetech import FeetechMotorsBus
from robot_devices.motors.configs import FeetechMotorsBusConfig


class InterfaceController:
    def __init__(self):
        project_path = self.get_project_root_path()
        mjcf_path = project_path + "/models/lerobot/so101/scene/scene.xml"
        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        self.render_freq = 60  # Hz
        self.render_interval = 1.0 / self.render_freq
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        # set initial position to middle position
        self.data.ctrl[:] = [0, -1.57079, 1.57079, 0, 0, 0]
        mujoco.mj_step(self.model, self.data)

        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)
        self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.camera.distance = 3.0
        self.camera.azimuth = 90.0
        self.camera.elevation = -30.0
        self.camera.lookat = np.array([0.0, 0.0, 0.0])

        self.qFb = np.zeros(self.model.nu)
        self.qdFb = np.zeros(self.model.nu)
        self.current_time = time.time()
        self.last_joint_positions_rad = []

    def update_feedback(self):
        self.qFb = self.data.qpos
        self.qdFb = self.data.qvel

    def get_project_root_path(self):
        project_name = "robot_motion"
        current_file_path = str(os.path.abspath(__file__))
        current_path_list = current_file_path.split(project_name)
        project_root_path = current_path_list[0] + project_name
        return project_root_path
    
    def update_control(self):
        ctrl_values = self.data.ctrl.copy()
        return ctrl_values

    def run(self):
        joint_mapping, joint_range, joint_range_rad, motor_bus, config = self.real_robot()
        # input("\nPress Enter to start sine motion, or Ctrl+C to cancel...")
        joint_names = list(config.motors.keys())
        self.enable_torque(motor_bus, joint_names)
        i = 0
        while self.viewer.is_running():
            self.update_feedback()
            ctrl_values = self.update_control()
            if i > 500:
                self.real_robot_control(ctrl_values, joint_mapping, joint_range, joint_range_rad, motor_bus)
            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()
            time.sleep(0.001)
            i += 1
            if i > 100000:
                i = 10000
        self.go_to_rest_position(motor_bus, joint_mapping, joint_range, joint_range_rad)
        self.disconnect_robot(motor_bus, config)

    def load_config(self, config_path):
        """Load configuration from YAML file."""
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        return config


    def create_so100_follower_config(self, port: Optional[str] = None) -> FeetechMotorsBusConfig:
        """Create configuration for SO100 follower arm.

        Args:
            port: USB port for the follower arm (will load from config if None)

        Returns:
            FeetechMotorsBusConfig: Configuration for the follower arm
        """
        # Load port from config if not provided
        if port is None:
            try:
                config_path = "configs/config.yaml"
                config_data = self.load_config(config_path)
                port = config_data.get("robot", {}).get("port", "/dev/ttyACM0")
            except Exception as e:
                print(f"Warning: Could not load config, using default port: {e}")
                port = "/dev/ttyACM0"

        # Ensure port is always a string at this point
        assert isinstance(port, str), "Port must be a string"

        # SO100 follower arm configuration based on robot configs
        motors = {
            "shoulder_pan": (1, "sts3215"),
            "shoulder_lift": (2, "sts3215"),
            "elbow_flex": (3, "sts3215"),
            "wrist_flex": (4, "sts3215"),
            "wrist_roll": (5, "sts3215"),
            "gripper": (6, "sts3215"),
        }

        return FeetechMotorsBusConfig(
            port=port, motors=motors, mock=False  # Set to True for testing without hardware
        )


    def get_joint_positions(
        self, motor_bus: FeetechMotorsBus, joint_names: list, title: str = "Joint Positions"
    ) -> Dict[str, float]:
        """Read and print current joint positions.

        Args:
            motor_bus: Connected FeetechMotorsBus instance
            joint_names: List of joint names to read
            title: Title to display

        Returns:
            Dict mapping joint names to positions
        """
        try:
            # Read present positions for all joints
            positions = motor_bus.read("Present_Position", joint_names)

            # Create a dictionary of joint name -> position
            joint_positions = {}
            # print(f"\n=== {title} ===")

            for i, joint_name in enumerate(joint_names):
                position = positions[i]
                joint_positions[joint_name] = float(position)
            #     print(f"{joint_name:>15}: {position:8.2f}")

            # print("=" * 45)
            return joint_positions

        except Exception as e:
            print(f"Error reading joint positions: {e}")
            return {}


    def enable_torque(self, motor_bus: FeetechMotorsBus, joint_names: list):
        """Enable torque on all motors to allow movement.

        Args:
            motor_bus: Connected FeetechMotorsBus instance
            joint_names: List of joint names to enable torque for
        """
        try:
            print("Enabling torque on all motors...")
            # Enable torque on all motors (1 = enabled, 0 = disabled)
            motor_bus.write("Torque_Enable", np.array([1] * len(joint_names)), joint_names)
            print("✓ Torque enabled on all motors")
        except Exception as e:
            print(f"Error enabling torque: {e}")
            raise


    def disable_torque(self, motor_bus: FeetechMotorsBus, joint_names: list):
        """Disable torque on all motors for safety.

        Args:
            motor_bus: Connected FeetechMotorsBus instance
            joint_names: List of joint names to disable torque for
        """
        try:
            print("Disabling torque on all motors...")
            # Disable torque on all motors (0 = disabled)
            motor_bus.write("Torque_Enable", np.array([0] * len(joint_names)), joint_names)
            print("✓ Torque disabled on all motors")
        except Exception as e:
            print(f"Error disabling torque: {e}")

    def real_robot(self):

        # Joint mapping
        joint_mapping = {
            1: "shoulder_pan",
            2: "shoulder_lift",
            3: "elbow_flex",
            4: "wrist_flex",
            5: "wrist_roll",
            6: "gripper",
        }

        joint_range = {
            "shoulder_pan": (1000, 3000),
            "shoulder_lift": (960, 3200),
            "elbow_flex": (900, 3050),
            "wrist_flex": (800, 3200),
            "wrist_roll": (880, 3100),
            "gripper": (2070, 3480),
        }

        joint_range_rad = {
            "shoulder_pan": (1.92, -1.92),
            "shoulder_lift": (-3.32, 0.0866),
            "elbow_flex": (-0.174, 3.14),
            "wrist_flex": (-1.66, 1.66),
            "wrist_roll": (-2.79, 2.79),
            "gripper": (-0.174, 1.75),
        }

        # Create motor bus configuration
        config = self.create_so100_follower_config()
        motor_bus = None

        try:
            # Create motor bus instance
            motor_bus = FeetechMotorsBus(config)

            # Connect to the arm
            print(f"Attempting to connect to port: {config.port}")
            motor_bus.connect()
            print("✓ Successfully connected to SO100 follower arm!")

            # Get joint names
            joint_names = list(config.motors.keys())
            print(f"Found {len(joint_names)} joints: {', '.join(joint_names)}")

            # Check if motors are responding
            print("\nTesting motor communication...")
            motor_ids = motor_bus.read("ID")
            print(f"Found motor IDs: {motor_ids}")

            # Read initial positions
            print("\nReading initial joint positions...")
            initial_positions = self.get_joint_positions(
                motor_bus, joint_names, "Initial Joint Positions"
            )
            print("\nInitial joint positions read successfully.", initial_positions)
            for joint_name in joint_names:
                current_position_rad = self.real_joint_pos_2_rad(joint_range, joint_range_rad, joint_name, initial_positions)
                self.last_joint_positions_rad.append(current_position_rad)

            if not initial_positions:
                raise Exception("Failed to read initial positions")
            
            # Warning and confirmation
            print("\n" + "⚠️ " * 20)
            print("WARNING: The arm will start moving!")
            print("Make sure the workspace is clear and safe.")
            print("Press Ctrl+C at any time to stop the motion.")
            print("⚠️ " * 20)

            print("\nStarting control real robot move...")
            print("Press Ctrl+C to stop")
            return joint_mapping, joint_range, joint_range_rad, motor_bus, config
            
        except Exception as e:
            print(f"\n❌ Error: {e}")
            print("\nTroubleshooting tips:")
            print("1. Check that the SO100 arm is connected via USB")
            print("2. Verify the USB port (default: /dev/ttyACM0)")
            print("3. Check that you have permission to access the USB port")
            print("4. Ensure the arm is powered on")
            print("5. Make sure the workspace is clear for motion")
            print("\nFull error trace:")
            traceback.print_exc()
            return None, None, None, None, None
        
    def real_joint_pos_2_rad(self, joint_range, joint_range_rad, joint_name, current_positions):
        """Convert target joint positions to radians."""
        joint_range_min, joint_range_max = joint_range[joint_name]
        joint_range_rad_min, joint_range_rad_max = joint_range_rad[joint_name]
        ratio = (current_positions[joint_name] - joint_range_min) / (joint_range_max - joint_range_min)
        current_position_rad = ratio * (joint_range_rad_max - joint_range_rad_min) + joint_range_rad_min
        return current_position_rad

    def real_robot_control(self, target_joint_poses, joint_mapping, joint_range, joint_range_rad, motor_bus, go_to_rest_flag=False):
        # try:
        target_joint_poses_rad = deepcopy(target_joint_poses)
        joint_names = joint_range.keys()
        current_positions = self.get_joint_positions(
            motor_bus, joint_names, "Current Joint Positions"
        )
        
        current_position_rad_all = []
        for joint_name in joint_names:
            current_position_rad = self.real_joint_pos_2_rad(joint_range, joint_range_rad, joint_name, current_positions)
            current_position_rad_all.append(current_position_rad)

        v_max = 1  # Maximum velocity in radians per second
        dt = 0.01  # Time step in seconds
        if time.time() - self.current_time < dt:
            dt = time.time() - self.current_time
        self.current_time = time.time()

        for i in range(len(self.last_joint_positions_rad)):
            angle_error = target_joint_poses_rad[i] - self.last_joint_positions_rad[i]
            if abs(angle_error) > v_max * dt:
                target_joint_poses_rad[i] = self.last_joint_positions_rad[i] + np.sign(angle_error) * v_max * dt

        for i in range(len(target_joint_poses_rad)):
            target_joint = joint_mapping[i + 1]
            joint_range_min, joint_range_max = joint_range[target_joint]
            joint_range_rad_min, joint_range_rad_max = joint_range_rad[target_joint]
            if go_to_rest_flag:
                if target_joint == "wrist_flex":
                    shoulder_lift_angle = self.real_joint_pos_2_rad(joint_range, joint_range_rad, "shoulder_lift", current_positions)
                    if shoulder_lift_angle > -0.5:
                        if target_joint_poses_rad[i] > 0:
                            target_joint_poses_rad[i] = 0
            ratio = (target_joint_poses_rad[i] - joint_range_rad_min) / (joint_range_rad_max - joint_range_rad_min)
            target_position = ratio * (joint_range_max - joint_range_min) + joint_range_min
            target_position = np.clip(target_position, joint_range_min, joint_range_max)
            motor_bus.write("Goal_Position", target_position, target_joint)
        self.last_joint_positions_rad = deepcopy(target_joint_poses_rad)
        # except KeyboardInterrupt:
        #     print("\n\nStopping motion...")
        # except Exception as e:
        #     print(f"\nError in motion loop: {e}")
        #     print("Stopping for safety...")

    def go_to_rest_position(self, motor_bus, joint_mapping, joint_range, joint_range_rad):
        """Move the robot to a predefined rest position."""
        try:
            print("\nMoving to rest position...")
            rest_position = [0, -3.32, 3.11, 1.0, 0, -0.174]
            while True:
                self.real_robot_control(rest_position, joint_mapping, joint_range, joint_range_rad, motor_bus, True)
                joint_names = joint_range.keys()
                current_positions = self.get_joint_positions(
                    motor_bus, joint_names, "Current Joint Positions"
                )
                joint_i = 0
                continue_moving = False
                for joint_name in joint_names:
                    current_position_rad = self.real_joint_pos_2_rad(joint_range, joint_range_rad, joint_name, current_positions)
                    angle_err = abs(current_position_rad - rest_position[joint_i])
                    if angle_err > 0.1:
                        continue_moving = True
                    joint_i += 1
                if not continue_moving:
                    break
            print("✓ Moved to rest position successfully")
        except Exception as e:
            print(f"Error moving to rest position: {e}")

    def disconnect_robot(self, motor_bus: FeetechMotorsBus, config):
        # Safety: Disable torque and clean up connection
        if motor_bus and motor_bus.is_connected:
            try:
                print("\nSafety shutdown...")

                # Get joint names for cleanup
                joint_names = list(config.motors.keys()) if config else []

                # Disable torque for safety
                self.disable_torque(motor_bus, joint_names)

                print("\nDisconnecting from arm...")
                motor_bus.disconnect()
                print("✓ Disconnected successfully")

            except Exception as e:
                print(f"Error during cleanup: {e}")

        print("\nMotion test completed safely.")

def main():
    # try:
    controller = InterfaceController()
    controller.run()
    # except KeyboardInterrupt:
    #     print("\nReceived keyboard interrupt, shutting down...")


if __name__ == "__main__":
    main()
