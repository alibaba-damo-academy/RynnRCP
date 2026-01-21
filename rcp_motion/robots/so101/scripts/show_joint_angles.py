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

"""Display joint angles for SO101 follower or leader.

Shows both raw encoder values and standardized joint feedback in radians.

Usage:
    $ so101-show-joint-angles
"""

import os
import time
import traceback
import yaml
import numpy as np
from pathlib import Path

from rcp_motion.robots.so101.interface.so101_interface import create_robot_interface
from rcp_motion.robots.so101.scripts.lang import select_language, t


def get_config_path():
    """Get the path to the so101.yaml config file."""
    import rcp_motion
    package_dir = Path(rcp_motion.__file__).parent
    return package_dir / "robots" / "so101" / "configs" / "so101.yaml"


def load_config(config_path: str = None):
    """Load configuration from YAML file."""
    if config_path is None:
        config_path = get_config_path()
    try:
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        return config if config else {}
    except FileNotFoundError:
        print(f"Config file {config_path} not found")
        return {}
    except yaml.YAMLError as e:
        print(f"Error parsing YAML file: {e}")
        return {}


def select_device(config: dict) -> str:
    """Prompt user to select follower or leader device.

    Returns:
        str: "follower" or "leader"
    """
    robot_available = config.get("robot", {}).get("port")
    teleop_available = config.get("teleoperate", {}).get("port")

    if not robot_available and not teleop_available:
        print(t("show_angles_no_config"))
        return None

    # Show options
    print(t("show_angles_option_follower"))
    print(t("show_angles_option_leader"))

    choice = input(t("show_angles_select_device")).strip()

    if choice == "2":
        return "leader"
    else:
        return "follower"


def display_positions(raw_positions: dict, radian_positions: np.ndarray,
                     joint_names: list, device_name: str):
    """Display both raw encoder values and radian positions."""
    print(f"\n{t('show_angles_header', device=device_name)}")
    print(t("show_angles_column_headers"))
    print("-" * 50)

    for i, name in enumerate(joint_names):
        # Get raw value (strip .pos suffix for display)
        display_name = name.replace(".pos", "")
        raw_val = raw_positions.get(display_name, 0)

        # Get radian value
        if i < len(radian_positions):
            rad_val = radian_positions[i]
            # Gripper is last joint, display as percentage
            if i == len(joint_names) - 1:
                print(f"{display_name:>15}: {raw_val:8.0f}    {rad_val:7.2f} (%)")
            else:
                print(f"{display_name:>15}: {raw_val:8.0f}    {rad_val:7.4f}")
        else:
            print(f"{display_name:>15}: {raw_val:8.0f}    N/A")

    print("=" * 50)


def main():
    """Main function to display joint angles."""
    # Select language if not set via environment
    select_language()

    print(t("show_angles_title"))
    print("=" * 50)

    config_path = get_config_path()
    config = load_config(config_path)

    # Select device
    device = select_device(config)
    if device is None:
        return 1

    # Get device name for display
    if device == "follower":
        device_name = t("dev_type_follower")
    else:  # leader
        device_name = t("dev_type_leader")

    print(t("show_angles_connecting", device=device_name))

    interface = None

    try:
        # Create interface based on device selection
        if device == "follower":
            interface = create_robot_interface(
                name="so101",
                mode="real",
                config_path=str(config_path),
            )
        else:  # leader
            interface = create_robot_interface(
                name="so101",
                mode="teleop",
                config_path=str(config_path),
            )

        # Connect
        interface.connect()

        # Disable torque so it can be moved freely
        interface.disable_robot_torque()

        print(t("show_angles_connected", device=device_name))

        # Get joint names (without .pos suffix for raw reading)
        joint_names = interface.joint_names
        raw_joint_names = [name.replace(".pos", "") for name in joint_names]

        print(f"\n{t('show_angles_monitoring', count=len(joint_names))}")
        print(t("show_angles_press_ctrl_c"))
        print("-" * 50)

        # Main monitoring loop
        loop_count = 0
        start_time = time.time()

        while True:
            try:
                # Read raw encoder positions (normalize=False)
                raw_positions = {}
                if hasattr(interface, 'robot') and interface.robot is not None:
                    if hasattr(interface.robot, 'bus'):
                        raw_positions = interface.robot.bus.sync_read(
                            "Present_Position",
                            raw_joint_names,
                            normalize=False
                        )

                # Read standardized positions in radians
                radian_positions = interface.get_joint_positions()

                # Display both
                display_positions(raw_positions, radian_positions, joint_names, device_name)

                # Calculate and display loop frequency
                loop_count += 1
                elapsed_time = time.time() - start_time
                frequency = loop_count / elapsed_time if elapsed_time > 0 else 0

                print(f"Loop: {loop_count:4d} | Freq: {frequency:5.1f} Hz | Time: {elapsed_time:6.1f}s")

                # Wait before next reading (10 Hz update rate)
                time.sleep(0.1)

            except KeyboardInterrupt:
                print(f"\n\n{t('show_angles_stopping')}")
                break
            except Exception as e:
                print(f"\n{t('show_angles_loop_error', error=e)}")
                print(t("show_angles_continuing"))
                time.sleep(1.0)

    except Exception as e:
        print(f"\n{t('show_angles_error', error=e)}")
        print(t("show_angles_troubleshooting"))
        print(t("show_angles_tip_1"))
        print(t("show_angles_tip_2"))
        print(t("show_angles_tip_3"))
        print(t("show_angles_tip_4"))
        print(t("show_angles_tip_5"))
        print(f"\n{t('show_angles_full_trace')}")
        traceback.print_exc()
        return 1

    finally:
        if interface and interface.is_connected:
            try:
                print(f"\n{t('show_angles_disconnecting')}")
                interface.disconnect()
                print(t("show_angles_disconnected"))
            except Exception as e:
                print(f"{t('show_angles_disconnect_error', error=e)}")

    # Force clean exit
    os._exit(0)


if __name__ == "__main__":
    main()
