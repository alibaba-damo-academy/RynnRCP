#!/usr/bin/env python3
"""
Calibration Script for SO100 Feetech arm - Follower arm only

This script provides calibration functionality for the SO100 follower arm using Feetech motors.
It supports both automatic and manual calibration modes.

Auto calibration: Automated process that moves the arm to find joint limits
Manual calibration: Manual positioning to zero and rotated positions

Usage:
    cd robots/lerobot
    python -m lerobot.scripts.follower_calibrate [--mode auto|manual]
"""

import argparse
import json
import time
import traceback
from pathlib import Path
from typing import Dict, Optional
import yaml

from robot_devices.motors.feetech import FeetechMotorsBus, TorqueMode
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


def check_torque_disabled(motor_bus: FeetechMotorsBus):
    """Check if torque is disabled on all motors."""
    try:
        torque_status = motor_bus.read("Torque_Enable")
        if (torque_status != TorqueMode.DISABLED.value).any():
            print("❌ Error: Torque must be disabled on all motors before calibration!")
            print("Current torque status:", torque_status)
            print("\nTo disable torque, manually power cycle the arm or run:")
            print(
                'python -c "from robot_devices.motors.feetech import *; '
                "from lerobot.scripts.follower.follower_calibrate import create_so100_follower_config; "
                "config = create_so100_follower_config(); "
                "bus = FeetechMotorsBus(config); bus.connect(); "
                "bus.write('Torque_Enable', [0]*6, list(config.motors.keys())); bus.disconnect()\""
            )
            return False
        return True
    except Exception as e:
        print(f"Error checking torque status: {e}")
        return False


def run_auto_calibration(motor_bus: FeetechMotorsBus) -> Dict:
    """Run automatic calibration for SO100 follower arm.

    Args:
        motor_bus: Connected FeetechMotorsBus instance

    Returns:
        Calibration dictionary
    """
    print("\n" + "🤖" * 20)
    print("AUTOMATIC CALIBRATION MODE")
    print("🤖" * 20)
    print("\nThis will automatically calibrate the SO100 follower arm.")
    print("The arm will move automatically to find joint limits.")
    print("\n⚠️  WARNING: Make sure the workspace is clear!")
    print("⚠️  WARNING: Stay ready to power off if needed!")

    # Import the calibration function
    try:
        from robot_devices.robots.feetech_calibration import (
            run_arm_auto_calibration_so100,
        )
        from robot_devices.motors.utils import MotorsBus
    except ImportError as e:
        raise ImportError(f"Could not import calibration function: {e}")

    input("\nPress Enter to start automatic calibration, or Ctrl+C to cancel...")

    # Run the SO100 auto calibration with type cast
    try:
        # Cast to MotorsBus protocol to satisfy type checker
        calibration = run_arm_auto_calibration_so100(motor_bus, "so100", "main", "follower")  # type: ignore
        return calibration
    except Exception as e:
        print(f"❌ Auto calibration failed: {e}")
        raise


def run_manual_calibration(motor_bus: FeetechMotorsBus) -> Dict:
    """Run manual calibration for SO100 follower arm.

    Args:
        motor_bus: Connected FeetechMotorsBus instance

    Returns:
        Calibration dictionary
    """
    print("\n" + "🔧" * 20)
    print("MANUAL CALIBRATION MODE")
    print("🔧" * 20)
    print("\nThis will manually calibrate the SO100 follower arm.")
    print("You will need to manually position the arm at specific poses.")

    # Import the calibration function
    try:
        from robot_devices.robots.feetech_calibration import (
            run_arm_manual_calibration,
        )
        from robot_devices.motors.utils import MotorsBus
    except ImportError as e:
        raise ImportError(f"Could not import calibration function: {e}")

    input("\nPress Enter to start manual calibration, or Ctrl+C to cancel...")

    # Run the manual calibration with type cast
    try:
        # Cast to MotorsBus protocol to satisfy type checker
        calibration = run_arm_manual_calibration(motor_bus, "so100", "main", "follower")  # type: ignore
        return calibration
    except Exception as e:
        print(f"❌ Manual calibration failed: {e}")
        raise


def save_calibration(
    calibration: Dict, calibration_dir: str = ".cache/calibration/so100"
) -> Path:
    """Save calibration to file.

    Args:
        calibration: Calibration dictionary
        calibration_dir: Directory to save calibration

    Returns:
        Path to saved calibration file
    """
    calib_dir = Path(calibration_dir)
    calib_dir.mkdir(parents=True, exist_ok=True)

    calib_file = calib_dir / "main_follower.json"

    with open(calib_file, "w") as f:
        json.dump(calibration, f, indent=2)

    print(f"✅ Calibration saved to: {calib_file}")
    return calib_file


def load_existing_calibration(
    calibration_dir: str = ".cache/calibration/so100",
) -> Dict | None:
    """Load existing calibration if available.

    Args:
        calibration_dir: Directory containing calibration files

    Returns:
        Calibration dictionary or None if not found
    """
    calib_dir = Path(calibration_dir)
    calib_file = calib_dir / "main_follower.json"

    if calib_file.exists():
        try:
            with open(calib_file, "r") as f:
                calibration = json.load(f)
            print(f"✅ Found existing calibration: {calib_file}")
            return calibration
        except Exception as e:
            print(f"❌ Error loading calibration file {calib_file}: {e}")
    else:
        print(f"❌ Calibration file not found: {calib_file}")
        print(
            "   The calibration file 'main_follower.json' is required for robot operation."
        )
        print("   Please run calibration first:")
        print("   python -m scripts.follower_calibrate --mode manual")

    return None


def apply_calibration_to_bus(motor_bus: FeetechMotorsBus, calibration: Dict):
    """Apply calibration to the motor bus.

    Args:
        motor_bus: Connected FeetechMotorsBus instance
        calibration: Calibration dictionary
    """
    try:
        motor_bus.set_calibration(calibration)
        print("✅ Calibration applied to motor bus")
    except Exception as e:
        print(f"❌ Error applying calibration: {e}")
        raise


def main():
    """Main function for SO100 follower arm calibration."""
    parser = argparse.ArgumentParser(description="SO100 Follower Arm Calibration")
    parser.add_argument(
        "--mode",
        choices=["auto", "manual", "load"],
        default="auto",
        help="Calibration mode: auto (automatic), manual (manual positioning), load (load existing)",
    )
    parser.add_argument(
        "--port",
        default=None,
        help="USB port for follower arm (default: load from config.yaml)",
    )
    parser.add_argument(
        "--calibration-dir",
        default=".cache/calibration/so100",
        help="Directory for calibration files (default: .cache/calibration/so100)",
    )

    args = parser.parse_args()

    # Create motor bus configuration (this will load port from config if needed)
    config = create_so100_follower_config(args.port)

    print("SO100 Feetech Arm Calibration Script")
    print("=" * 40)
    print(f"Mode: {args.mode}")
    print(f"Port: {config.port}")
    print(f"Calibration directory: {args.calibration_dir}")
    print("=" * 40)
    motor_bus = None

    try:
        # Handle load existing calibration mode
        if args.mode == "load":
            existing_calib = load_existing_calibration(args.calibration_dir)
            if existing_calib is None:
                print("❌ No existing calibration found!")
                print("Please run calibration with --mode auto or --mode manual first.")
                return

            print("\nExisting calibration data:")
            print(json.dumps(existing_calib, indent=2))

            # Test the calibration by connecting and applying it
            motor_bus = FeetechMotorsBus(config)
            motor_bus.connect()
            print("✅ Connected to SO100 follower arm")

            apply_calibration_to_bus(motor_bus, existing_calib)

            # Read and display calibrated positions
            joint_names = list(config.motors.keys())
            print_joint_positions(motor_bus, joint_names, "Calibrated Joint Positions")

            return

        # Create motor bus instance for calibration
        motor_bus = FeetechMotorsBus(config)

        # Connect to the arm
        print(f"\nAttempting to connect to port: {config.port}")
        motor_bus.connect()
        print("✅ Successfully connected to SO100 follower arm!")

        # Get joint names
        joint_names = list(config.motors.keys())
        print(f"Found {len(joint_names)} joints: {', '.join(joint_names)}")

        # Check if motors are responding
        print("\nTesting motor communication...")
        motor_ids = motor_bus.read("ID")
        print(f"Found motor IDs: {motor_ids}")

        # Check torque status
        print("\nChecking torque status...")
        if not check_torque_disabled(motor_bus):
            return

        # Read initial positions
        print_joint_positions(motor_bus, joint_names, "Pre-Calibration Joint Positions")

        # Run calibration based on mode
        calibration = None
        if args.mode == "auto":
            calibration = run_auto_calibration(motor_bus)
        elif args.mode == "manual":
            calibration = run_manual_calibration(motor_bus)

        if calibration is None:
            print("❌ Calibration failed - no calibration data returned")
            return

        # Save calibration
        calib_file = save_calibration(calibration, args.calibration_dir)

        # Apply calibration to the motor bus
        apply_calibration_to_bus(motor_bus, calibration)

        # Read final positions with calibration applied
        print_joint_positions(
            motor_bus, joint_names, "Post-Calibration Joint Positions"
        )

        print("\n" + "🎉" * 20)
        print("CALIBRATION COMPLETED SUCCESSFULLY!")
        print("🎉" * 20)
        print(f"✅ Calibration file saved: {calib_file}")
        print("✅ Calibration applied to motor bus")
        print("\nYou can now use the follower arm with calibrated positions!")

    except KeyboardInterrupt:
        print("\n\n⏹️  Calibration cancelled by user")
    except Exception as e:
        print(f"\n❌ Error during calibration: {e}")
        print("\nTroubleshooting tips:")
        print("1. Check that the SO100 arm is connected via USB")
        print("2. Verify the USB port in lerobot/configs/config.yaml")
        print("3. Check that you have permission to access the USB port")
        print("4. Ensure the arm is powered on")
        print("5. Make sure torque is disabled on all motors")
        print("6. Ensure the workspace is clear for arm movement")
        print("\nFull error trace:")
        traceback.print_exc()

    finally:
        # Clean up connection
        if motor_bus and motor_bus.is_connected:
            try:
                print("\nDisconnecting from arm...")
                motor_bus.disconnect()
                print("✅ Disconnected successfully")
            except Exception as e:
                print(f"Error during disconnect: {e}")


if __name__ == "__main__":
    main()
