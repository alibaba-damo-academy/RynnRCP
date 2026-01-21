#!/usr/bin/env python3
# Copyright 2025 RynnRCP. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Raw Joint Sine Motion Test Script for SO101 Feetech arm - Follower (slave) arm only

This script connects to the SO101 follower arm using Feetech motors,
reads initial joint positions, and then applies a sine wave motion
to a specified joint using raw encoder values.

Motion: position = initial_position + 300 * sin(2*pi*f*t)
Where f = 0.2 Hz (adjustable)

Example:

```shell
so101-jointsine --joint 1
python -m rcp_motion.robots.so101.scripts.follower_jointsine --joint 1
```

Joint mapping:
    1 = shoulder_pan
    2 = shoulder_lift
    3 = elbow_flex
    4 = wrist_flex
    5 = wrist_roll

!!!!!!!!!!!!!!!!! argument --joint 6 is currently not supported !!!!!!!!!!!!!!!
"""

import argparse
import time
import traceback
import math
import numpy as np
import yaml
import json
from pathlib import Path
from typing import Dict

from rcp_motion.robots.so101.hardware.motors.feetech import FeetechMotorsBus
from rcp_motion.robots.so101.hardware.motors import Motor, MotorNormMode

from rcp_motion.utils.policy_interpolator import lerp


def get_config_path():
    """Get the path to the so101.yaml config file."""
    import rcp_motion
    package_dir = Path(rcp_motion.__file__).parent
    return package_dir / "robots" / "so101" / "configs" / "so101.yaml"


def create_motor_bus():
    try:
        config_path = get_config_path()
        config_data = load_config(config_path)
        port = config_data.get("robot", {}).get("port", "/dev/ttyACM0")
    except Exception as e:
        print(f"Warning: Could not load config, using default port: {e}")
        port = "/dev/ttyACM0"

    motors = {
        "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
        "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
        "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
        "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
        "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
        "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_1),
    }
    return FeetechMotorsBus(port=port, motors=motors)


def load_config(config_path):
    """Load configuration from YAML file."""
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config


def print_joint_positions(
    motor_bus: FeetechMotorsBus,
    motors,
    joint_names: list,
    title: str = "Joint Positions",
) -> Dict[str, float]:
    """Read and print current joint positions.

    Args:
        motor_bus: Connected FeetechMotorsBus instance
        motors: Dictionary of motors
        joint_names: List of joint names to read
        title: Title to display

    Returns:
        Dict mapping joint names to positions
    """
    try:
        # Read present positions for all joints
        positions = motor_bus.sync_read(
            "Present_Position", list(motors), normalize=False
        )

        # Create a dictionary of joint name -> position
        joint_positions = {}
        print(f"\n=== {title} ===")

        for joint_name in joint_names:
            position = positions[joint_name]
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
        motor_bus.sync_write("Torque_Enable", 1)
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
        motor_bus.sync_write("Torque_Enable", 0)
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
    """Main function for SO101 raw joint sine motion test."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="SO101 Raw Joint Sine Motion Test")
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

    config_path = get_config_path()
    config_data = load_config(config_path)
    calibration_dir = config_data.get("robot", {}).get("calibration_dir", None)
    default_flag = False
    try:
        with open(calibration_dir) as f:
            data = json.load(f)

        start_pos_gripper = data[target_joint]["range_min"]
        end_pos_gripper = data[target_joint]["range_max"]
        middle_pos_gripper = (start_pos_gripper + end_pos_gripper) / 2.0
        amplitude_gripper = (end_pos_gripper - start_pos_gripper) / 2.0
    except Exception as e:
        print(f"Warning: Could not load calibration data: {e}.")
        default_flag = True

    print("SO101 Feetech Arm Raw Joint Sine Motion Test")
    print("=" * 50)
    print(f"This script will apply sine wave motion to {target_joint} joint")
    (
        print(f"Motion: position = initial + {AMPLITUDE} * sin(2*pi*0.2*t)")
        if default_flag
        else print(
            f"Motion: position = {middle_pos_gripper} + {amplitude_gripper} * sin(2*pi*0.2*t)"
        )
    )
    print("=" * 50)

    print(f"Motion parameters:")
    print(f"  Target joint: {target_joint} (joint {args.joint})")
    (
        print(f"  Amplitude: {AMPLITUDE} raw encoder steps")
        if default_flag
        else print(f"  Amplitude: {amplitude_gripper} raw encoder steps")
    )
    print(f"  Frequency: {FREQUENCY} Hz ({1/FREQUENCY:.1f} second period)")
    print(f"  Control rate: {CONTROL_RATE} Hz")
    print()

    motor_bus = None

    try:
        # Create motor bus instance
        motor_bus = create_motor_bus()

        # Connect to the arm
        print(f"Attempting to connect to port: {motor_bus.port}")
        motor_bus.connect()
        print("✓ Successfully connected to SO101 follower arm!")

        # Get joint names
        joint_names = list(motor_bus.motors.keys())
        print(f"Monitoring {len(joint_names)} joints: {', '.join(joint_names)}")

        # Check if motors are responding
        print("\nTesting motor communication...")
        motor_ids = motor_bus.sync_read("ID", list(motor_bus.motors))
        print(f"Found motor IDs: {motor_ids}")

        # Read initial positions
        print("\nReading initial joint positions...")
        initial_positions = print_joint_positions(
            motor_bus, motor_bus.motors, joint_names, "Initial Joint Positions"
        )

        if not initial_positions:
            raise Exception("Failed to read initial positions")

        # Get initial position of the target joint
        initial_target_position = initial_positions[target_joint]
        print(f"\nTarget joint: {target_joint}")
        print(f"Initial raw position: {initial_target_position:.2f}")

        # Safety check for motion range
        if not default_flag:
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
                if not default_flag:
                    if elapsed_time <= 2:
                        target_position = lerp(
                            initial_target_position,
                            middle_pos_gripper,
                            elapsed_time,
                            2.0,
                        )
                        pass
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
                motor_bus.write(
                    "Goal_Position", target_joint, int(target_position), normalize=False
                )

                # Read current positions for monitoring
                if loop_count % 10 == 0:  # Print every 10 loops (0.2 seconds at 50Hz)
                    current_positions = motor_bus.sync_read(
                        "Present_Position", list(motor_bus.motors), normalize=False
                    )

                    print(
                        f"Time: {elapsed_time:6.2f}s | Target: {target_position:7.1f} | "
                        + f"Actual: {current_positions[target_joint]:7.1f} | "
                        + f"Error: {target_position - current_positions[target_joint]:6.1f}"
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
        print("1. Check that the SO101 arm is connected via USB")
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
                joint_names = (
                    list(motor_bus.motors.keys()) if motor_bus else [target_joint]
                )

                # Disable torque for safety
                disable_torque(motor_bus, joint_names)

                # Read final positions
                print("\nFinal joint positions:")
                print_joint_positions(
                    motor_bus, motor_bus.motors, joint_names, "Final Joint Positions"
                )

                print("\nDisconnecting from arm...")
                motor_bus.disconnect()
                print("✓ Disconnected successfully")

            except Exception as e:
                print(f"Error during cleanup: {e}")

        print("\nMotion test completed safely.")


if __name__ == "__main__":
    main()
