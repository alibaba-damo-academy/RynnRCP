#!/usr/bin/env python3
"""
Test script for SO100 Feetech arm - Follower arm only

This script connects to the SO100 follower arm using Feetech motors
and continuously reads and prints joint positions.

Usage:
    cd robots/lerobot
    python -m lerobot.scripts.follower_show_encoder
"""

import time
import traceback
import numpy as np
import yaml
import os
from typing import Dict, Optional

from robot_devices.motors.feetech import FeetechMotorsBus
from robot_devices.motors.configs import FeetechMotorsBusConfig


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
    motor_bus: FeetechMotorsBus, joint_names: list
) -> Dict[str, float]:
    """Read and print current joint positions.

    Args:
        motor_bus: Connected FeetechMotorsBus instance
        joint_names: List of joint names to read

    Returns:
        Dict mapping joint names to positions
    """
    try:
        # Read present positions for all joints
        positions = motor_bus.read("Present_Position", joint_names)

        # Create a dictionary of joint name -> position
        joint_positions = {}
        print("\n=== SO100 Follower Arm Joint Positions ===")

        for i, joint_name in enumerate(joint_names):
            position = positions[i]
            joint_positions[joint_name] = float(position)
            print(f"{joint_name:>15}: {position:8.2f}")

        print("=" * 45)
        return joint_positions

    except Exception as e:
        print(f"Error reading joint positions: {e}")
        return {}


def main():
    """Main function to test SO100 follower arm."""
    print("SO100 Feetech Arm Test Script")
    print("=" * 40)
    print("Connecting to SO100 follower arm...")

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
        print(f"Monitoring {len(joint_names)} joints: {', '.join(joint_names)}")

        # Check if motors are responding
        print("\nTesting motor communication...")
        motor_ids = motor_bus.read("ID")
        print(f"Found motor IDs: {motor_ids}")

        # Read initial positions
        print("\nReading initial joint positions...")
        print_joint_positions(motor_bus, joint_names)

        print("\nStarting continuous monitoring...")
        print("Press Ctrl+C to stop")
        print("-" * 45)

        # Main monitoring loop
        loop_count = 0
        start_time = time.time()

        while True:
            try:
                # Read and display joint positions
                positions = print_joint_positions(motor_bus, joint_names)

                # Calculate loop frequency
                loop_count += 1
                elapsed_time = time.time() - start_time
                frequency = loop_count / elapsed_time if elapsed_time > 0 else 0

                print(
                    f"Loop: {loop_count:4d} | Freq: {frequency:5.1f} Hz | Time: {elapsed_time:6.1f}s"
                )

                # Wait before next reading
                time.sleep(0.1)  # 10 Hz update rate

            except KeyboardInterrupt:
                print("\n\nStopping monitoring...")
                break
            except Exception as e:
                print(f"\nError in monitoring loop: {e}")
                print("Continuing...")
                time.sleep(1.0)

    except Exception as e:
        print(f"\n❌ Error: {e}")
        print("\nTroubleshooting tips:")
        print("1. Check that the SO100 arm is connected via USB")
        print("2. Verify the USB port in lerobot/configs/config.yaml")
        print("3. Check that you have permission to access the USB port")
        print("4. Ensure the arm is powered on")
        print("5. Try running: ls /dev/tty* to see available ports")
        print("\nFull error trace:")
        traceback.print_exc()

    finally:
        # Clean up connection
        if motor_bus and motor_bus.is_connected:
            try:
                print("\nDisconnecting from arm...")
                motor_bus.disconnect()
                print("✓ Disconnected successfully")
            except Exception as e:
                print(f"Error during disconnect: {e}")


if __name__ == "__main__":
    main()
