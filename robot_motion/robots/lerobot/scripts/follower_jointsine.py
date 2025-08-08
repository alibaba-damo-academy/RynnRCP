#!/usr/bin/env python3
"""
Raw Joint Sine Motion Test Script for SO100 Feetech arm - Follower arm only

This script connects to the SO100 follower arm using Feetech motors,
reads initial joint positions, and then applies a sine wave motion
to a specified joint using raw encoder values.

Motion: position = initial_position + 300 * sin(2*pi*f*t)
Where f = 0.2 Hz (adjustable)

Usage:
    cd robots/lerobot
    python -m lerobot.scripts.follower_jointsine --joint 1

Joint mapping:
    1 = shoulder_pan
    2 = shoulder_lift
    3 = elbow_flex
    4 = wrist_flex
    5 = wrist_roll
"""

import argparse
import time
import traceback
import math
import numpy as np
import yaml
import json
from typing import Dict, Optional

from robot_devices.motors.feetech import FeetechMotorsBus
from robot_devices.motors.configs import FeetechMotorsBusConfig
from utils.policy_interpolator import lerp


def load_config(config_path):
    """Load configuration from YAML file."""
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config


def create_so100_follower_config(port: Optional[str] = None) -> FeetechMotorsBusConfig:
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
            config_data = load_config(config_path)
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


def print_joint_positions(
    motor_bus: FeetechMotorsBus, joint_names: list, title: str = "Joint Positions"
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
        print(f"\n=== {title} ===")

        for i, joint_name in enumerate(joint_names):
            position = positions[i]
            joint_positions[joint_name] = float(position)
            print(f"{joint_name:>15}: {position:8.2f}")

        print("=" * 45)
        return joint_positions

    except Exception as e:
        print(f"Error reading joint positions: {e}")
        return {}


def enable_torque(motor_bus: FeetechMotorsBus, joint_names: list):
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


def disable_torque(motor_bus: FeetechMotorsBus, joint_names: list):
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


def generate_sine_position(
    initial_pos: float, amplitude: float, frequency: float, time_elapsed: float
) -> float:
    """Generate sine wave position.

    Args:
        initial_pos: Initial position offset
        amplitude: Amplitude of the sine wave
        frequency: Frequency in Hz
        time_elapsed: Time elapsed since start in seconds

    Returns:
        Target position
    """
    return initial_pos + amplitude * math.sin(2 * math.pi * frequency * time_elapsed)


def main():
    """Main function for SO100 raw joint sine motion test."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="SO100 Raw Joint Sine Motion Test")
    parser.add_argument(
        "--joint",
        type=int,
        choices=[1, 2, 3, 4, 5, 6],
        required=True,
        help="Joint to move: 1=shoulder_pan, 2=shoulder_lift, 3=elbow_flex, 4=wrist_flex, 5=wrist_roll, 6=gripper",
    )
    args = parser.parse_args()

    # Joint mapping
    joint_mapping = {
        1: "shoulder_pan",
        2: "shoulder_lift",
        3: "elbow_flex",
        4: "wrist_flex",
        5: "wrist_roll",
        6: "gripper",
    }

    target_joint = joint_mapping[args.joint]

    # Motion parameters
    AMPLITUDE = 300.0  # Amplitude of sine wave (reduced from 500)
    FREQUENCY = 0.2  # Frequency in Hz (0.2 Hz = 5 second period)
    CONTROL_RATE = 50  # Control frequency in Hz

    with open(".cache/calibration/so100/main_follower.json", "r") as f:
        data = json.load(f)

    start_pos_gripper = data["start_pos"][-1]
    end_pos_gripper = data["end_pos"][-1]
    middle_pos_gripper = (start_pos_gripper + end_pos_gripper) / 2.0
    amplitude_gripper = (end_pos_gripper - start_pos_gripper) / 2.0

    print("SO100 Feetech Arm Raw Joint Sine Motion Test")
    print("=" * 50)
    print(f"This script will apply sine wave motion to {target_joint} joint")
    (
        print(f"Motion: position = initial + {AMPLITUDE} * sin(2*pi*0.2*t)")
        if args.joint != 6
        else print(
            f"Motion: position = {middle_pos_gripper} + {amplitude_gripper} * sin(2*pi*0.2*t)"
        )
    )
    print("=" * 50)

    print(f"Motion parameters:")
    print(f"  Target joint: {target_joint} (joint {args.joint})")
    (
        print(f"  Amplitude: {AMPLITUDE} raw encoder steps")
        if args.joint != 6
        else print(f"  Amplitude: {amplitude_gripper} raw encoder steps")
    )
    print(f"  Frequency: {FREQUENCY} Hz ({1/FREQUENCY:.1f} second period)")
    print(f"  Control rate: {CONTROL_RATE} Hz")
    print()

    # Create motor bus configuration
    config = create_so100_follower_config()
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
        initial_positions = print_joint_positions(
            motor_bus, joint_names, "Initial Joint Positions"
        )

        if not initial_positions:
            raise Exception("Failed to read initial positions")

        # Get initial position of the target joint
        initial_target_position = initial_positions[target_joint]
        print(f"\nTarget joint: {target_joint}")
        print(f"Initial raw position: {initial_target_position:.2f}")

        # Safety check for motion range
        if args.joint == 6:
            print(f"Motion range: {start_pos_gripper} to {end_pos_gripper}")
        else:
            min_pos = initial_target_position - AMPLITUDE
            max_pos = initial_target_position + AMPLITUDE
            print(f"Motion range: {min_pos:.2f} to {max_pos:.2f}")

        # Warning and confirmation
        print("\n" + "⚠️ " * 20)
        print("WARNING: The arm will start moving!")
        print("Make sure the workspace is clear and safe.")
        print("Press Ctrl+C at any time to stop the motion.")
        print("⚠️ " * 20)

        input("\nPress Enter to start sine motion, or Ctrl+C to cancel...")

        # Enable torque for movement
        enable_torque(motor_bus, joint_names)

        print("\nStarting sine wave motion...")
        print("Press Ctrl+C to stop")
        print("-" * 50)

        # Main motion loop
        loop_count = 0
        start_time = time.time()
        dt = 1.0 / CONTROL_RATE  # Time step

        while True:
            try:
                current_time = time.time()
                elapsed_time = current_time - start_time

                # Generate target position for the selected joint
                if args.joint == 6:
                    if elapsed_time <= 2:
                        target_position = lerp(
                            initial_target_position,
                            middle_pos_gripper,
                            elapsed_time,
                            2.0,
                        )
                    else:
                        target_position = generate_sine_position(
                            middle_pos_gripper,
                            amplitude_gripper,
                            FREQUENCY,
                            elapsed_time - 2,
                        )
                else:
                    target_position = generate_sine_position(
                        initial_target_position, AMPLITUDE, FREQUENCY, elapsed_time
                    )

                # Send goal position to the target joint only
                motor_bus.write("Goal_Position", target_position, target_joint)

                # Read current positions for monitoring
                if loop_count % 10 == 0:  # Print every 10 loops (0.2 seconds at 50Hz)
                    current_positions = motor_bus.read(
                        "Present_Position", [target_joint]
                    )

                    print(
                        f"Time: {elapsed_time:6.2f}s | Target: {target_position:7.1f} | "
                        + f"Actual: {current_positions[0]:7.1f} | "
                        + f"Error: {target_position - current_positions[0]:6.1f}"
                    )

                loop_count += 1

                # Control timing
                next_time = start_time + loop_count * dt
                sleep_time = next_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)

            except KeyboardInterrupt:
                print("\n\nStopping motion...")
                break
            except Exception as e:
                print(f"\nError in motion loop: {e}")
                print("Stopping for safety...")
                break

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

    finally:
        # Safety: Disable torque and clean up connection
        if motor_bus and motor_bus.is_connected:
            try:
                print("\nSafety shutdown...")

                # Get joint names for cleanup
                joint_names = list(config.motors.keys()) if config else [target_joint]

                # Disable torque for safety
                disable_torque(motor_bus, joint_names)

                # Read final positions
                print("\nFinal joint positions:")
                print_joint_positions(motor_bus, joint_names, "Final Joint Positions")

                print("\nDisconnecting from arm...")
                motor_bus.disconnect()
                print("✓ Disconnected successfully")

            except Exception as e:
                print(f"Error during cleanup: {e}")

        print("\nMotion test completed safely.")


if __name__ == "__main__":
    main()
