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

"""Robot calibration helper.

Reads configuration from configs/so101.yaml and runs calibration automatically.

Supports selecting which arm(s) to calibrate:
- If not specified, calibrates both sequentially: follower -> leader.
- If language not specified, prompts user to choose.

Usage:
    $ so101-calibrate
    $ so101-calibrate --arm follower
    $ so101-calibrate --arm leader
    $ so101-calibrate --arm both
    $ so101-calibrate --lang zh
    $ so101-calibrate --arm leader --lang en
    $ so101-calibrate --config /path/to/so101.yaml
"""

import argparse
import logging
import os
from pathlib import Path
import yaml

from rcp_motion.robots.so101.hardware.robots.so101_follower.config_so101_follower import (
    SO101FollowerConfig,
)
from rcp_motion.robots.so101.hardware.robots.so101_follower.so101_follower import (
    SO101Follower,
)
from rcp_motion.robots.so101.scripts.lang import select_language, t, set_language

logger = logging.getLogger(__name__)


def parse_args():
    p = argparse.ArgumentParser(description="SO101 calibration helper")
    p.add_argument(
        "--arm",
        choices=["follower", "leader", "both"],
        default=None,
        help="Which arm to calibrate. Default: both (follower -> leader).",
    )
    p.add_argument(
        "--lang",
        choices=["zh", "en"],
        default=None,
        help="Language. If not provided, prompts user to choose.",
    )
    p.add_argument(
        "--config",
        default=None,
        help="Path to so101.yaml (optional). Default: package robots/so101/configs/so101.yaml",
    )
    return p.parse_args()


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
        logger.error(f"Config file {config_path} not found")
        return {}
    except yaml.YAMLError as e:
        logger.error(f"Error parsing YAML file: {e}")
        return {}


def extract_id_from_path(calibration_dir: str) -> str:
    """Extract device ID from calibration directory path."""
    path = Path(os.path.expanduser(calibration_dir))
    return path.name


def make_robot_from_config(
    robot_type: str, port: str = None, calibration_dir: str = None
):
    """Create robot instance based on type."""
    if robot_type != "so101_follower":
        raise ValueError(f"Unsupported robot type: {robot_type}")

    robot_id = extract_id_from_path(calibration_dir) if calibration_dir else robot_type
    config = SO101FollowerConfig(port=port or "/dev/ttyACM0", id=robot_id)

    if calibration_dir:
        calibration_path = Path(os.path.expanduser(calibration_dir))
        config.calibration_dir = calibration_path
        logging.info(f"Set calibration file path: {calibration_path}")

    return SO101Follower(config)


def make_teleoperator_from_config(
    teleop_type: str, port: str = None, calibration_dir: str = None
):
    """Create teleoperator instance based on type."""
    if teleop_type != "so101_leader":
        raise ValueError(f"Unsupported teleoperator type: {teleop_type}")

    teleop_id = (
        extract_id_from_path(calibration_dir) if calibration_dir else teleop_type
    )
    # NOTE: keep existing behavior: leader uses the same SO101FollowerConfig/SO101Follower here.
    config = SO101FollowerConfig(port=port or "/dev/ttyUSB0", id=teleop_id)

    if calibration_dir:
        calibration_path = Path(os.path.expanduser(calibration_dir))
        config.calibration_dir = calibration_path
        logging.info(f"Set calibration file path: {calibration_path}")

    return SO101Follower(config)


def prompt_input(prompt: str = "") -> str:
    if prompt:
        print(prompt, end="", flush=True)
    return input()


def calibrate_device(device):
    """Calibrate the given device."""
    try:
        logging.info("Connecting to device...")
        device.connect(calibrate=False)

        logging.info("Starting calibration...")
        if hasattr(device, "calibrate"):
            device.calibrate()
        else:
            logging.warning("Device does not support calibration method")

        logging.info("Calibration completed successfully")
        return True

    except Exception as e:
        logging.error(f"Calibration failed: {e}")
        return False
    finally:
        if hasattr(device, "disconnect"):
            device.disconnect()
            logging.info("Device disconnected")


def calibrate_follower(config: dict) -> str:
    """Calibrate follower robot. Returns status string."""
    robot_config = config.get("robot", {})
    if not robot_config or not robot_config.get("port"):
        return t("calib_status_no_config")

    # Prompt user
    response = (
        prompt_input(
            t(
                "calib_prompt",
                dev_type=t("dev_type_follower"),
                default="Y/n",
                default_val="Y",
            )
        )
        .strip()
        .lower()
    )
    if not response:
        response = "y"

    if response in ["n", "no"]:
        print(t("calib_skip", dev_type=t("dev_type_follower")))
        return t("calib_status_skipped")

    print(f"\n{'='*60}")
    print(t("calib_header_follower"))
    print(f"{'='*60}")

    robot_port = robot_config.get("port")
    robot_calibration_dir = robot_config.get("calibration_dir")
    robot_type = "so101_follower"
    robot_id = (
        extract_id_from_path(robot_calibration_dir)
        if robot_calibration_dir
        else "unknown"
    )

    logging.info(
        t(
            "calib_info",
            dev_type=t("dev_type_follower"),
            type=robot_type,
            id=robot_id,
            port=robot_port,
        )
    )
    logging.info(t("calib_file_path", path=robot_calibration_dir))

    try:
        robot_device = make_robot_from_config(
            robot_type, robot_port, robot_calibration_dir
        )
        return (
            t("calib_status_done")
            if calibrate_device(robot_device)
            else t("calib_status_failed")
        )
    except Exception as e:
        logging.error(f"Failed to create robot: {e}")
        return t("calib_status_failed")


def calibrate_leader(config: dict) -> str:
    """Calibrate leader teleoperator. Returns status string."""
    teleop_config = config.get("teleoperate", {})
    if not teleop_config or not teleop_config.get("port"):
        return t("calib_status_no_config")

    # Prompt user
    response = (
        prompt_input(
            t(
                "calib_prompt",
                dev_type=t("dev_type_leader"),
                default="Y/n",
                default_val="Y",
            )
        )
        .strip()
        .lower()
    )
    if not response:
        response = "y"

    if response in ["n", "no"]:
        print(t("calib_skip", dev_type=t("dev_type_leader")))
        return t("calib_status_skipped")

    print(f"\n{'='*60}")
    print(t("calib_header_leader"))
    print(f"{'='*60}")

    teleop_port = teleop_config.get("port")
    teleop_calibration_dir = teleop_config.get("calibration_dir")
    teleop_type = "so101_leader"
    teleop_id = (
        extract_id_from_path(teleop_calibration_dir)
        if teleop_calibration_dir
        else "unknown"
    )

    logging.info(
        t(
            "calib_info",
            dev_type=t("dev_type_leader"),
            type=teleop_type,
            id=teleop_id,
            port=teleop_port,
        )
    )
    logging.info(t("calib_file_path", path=teleop_calibration_dir))

    try:
        teleop_device = make_teleoperator_from_config(
            teleop_type, teleop_port, teleop_calibration_dir
        )
        return (
            t("calib_status_done")
            if calibrate_device(teleop_device)
            else t("calib_status_failed")
        )
    except Exception as e:
        logging.error(f"Failed to create teleoperator: {e}")
        return t("calib_status_failed")


def main():
    """Main calibration function."""
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )

    # Language selection
    if args.lang:
        set_language(args.lang)
    else:
        select_language()

    # Load configuration
    config_path = args.config or str(get_config_path())
    config = load_config(config_path)
    logging.info(t("calib_loading_config", config_path=config_path))

    # Decide which arm(s) to calibrate
    arm = args.arm or "both"

    follower_status = None
    leader_status = None

    if arm in ("follower", "both"):
        follower_status = calibrate_follower(config)

    if arm in ("leader", "both"):
        leader_status = calibrate_leader(config)

    # Print summary (only what we ran)
    print(t("calib_summary"))
    if follower_status is not None:
        print(t("calib_follower_status", status=follower_status))
    if leader_status is not None:
        print(t("calib_leader_status", status=leader_status))
    print("=" * 60)

    os._exit(0)


if __name__ == "__main__":
    main()
