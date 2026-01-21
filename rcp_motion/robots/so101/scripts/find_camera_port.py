#!/usr/bin/env python
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
Helper to find the camera devices available in your system.

Example:

```shell
so101-find-camera-port
python -m rcp_motion.robots.so101.scripts.find_camera_port
```
"""

import argparse
import concurrent.futures
import logging
import time
import sys
import platform
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from rcp_motion.robots.so101.hardware.cameras.configs import ColorMode
from rcp_motion.robots.so101.hardware.cameras.opencv.camera_opencv import OpenCVCamera
from rcp_motion.robots.so101.hardware.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from rcp_motion.robots.so101.scripts.lang import select_language, t

logger = logging.getLogger(__name__)


def get_config_path():
    """Get the path to the so101.yaml config file."""
    import rcp_motion
    package_dir = Path(rcp_motion.__file__).parent
    return package_dir / "robots" / "so101" / "configs" / "so101.yaml"


def filter_ubuntu_external_cameras(camera_list: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """
    Filter cameras for Ubuntu laptops to get only external cameras.
    Ubuntu laptops typically have:
    - /dev/video0, /dev/video2: Built-in cameras (webcam, infrared)
    - /dev/video4, /dev/video6: External USB cameras (front, wrist)

    Returns the external cameras (higher numbered video devices).
    """
    if platform.system() != "Linux":
        return camera_list

    external_cameras = []
    for cam in camera_list:
        cam_id = str(cam["id"])
        if "/dev/video" in cam_id:
            # Extract the video device number
            try:
                video_num = int(cam_id.replace("/dev/video", ""))
                # Consider cameras with video4+ as external cameras
                if video_num >= 4:
                    external_cameras.append(cam)
            except ValueError:
                # If we can't parse the number, include it anyway
                external_cameras.append(cam)
        else:
            # Not a /dev/video device, include it
            external_cameras.append(cam)

    logger.info(t("filtered_cameras", filtered=len(external_cameras), total=len(camera_list)))
    return external_cameras


def find_all_opencv_cameras(filter_ubuntu_external: bool = False) -> list[dict[str, Any]]:
    """
    Finds all available OpenCV cameras plugged into the system.

    Args:
        filter_ubuntu_external: If True and on Ubuntu, filter to only external cameras (video4+)

    Returns:
        A list of all available OpenCV cameras with their metadata.
    """
    all_opencv_cameras_info: list[dict[str, Any]] = []
    logger.info(t("searching_opencv"))
    try:
        opencv_cameras = OpenCVCamera.find_cameras()
        for cam_info in opencv_cameras:
            all_opencv_cameras_info.append(cam_info)
        logger.info(t("found_opencv", count=len(opencv_cameras)))

        if filter_ubuntu_external:
            all_opencv_cameras_info = filter_ubuntu_external_cameras(all_opencv_cameras_info)

    except Exception as e:
        logger.error(f"Error finding OpenCV cameras: {e}")

    return all_opencv_cameras_info


def update_camera_config_yaml(front_camera_id=None, wrist_camera_id=None, config_path=None):
    """Update so101.yaml with detected camera ports."""
    import yaml

    if config_path is None:
        config_path = get_config_path()

    try:
        if Path(config_path).exists():
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
        else:
            config = {}

        # Ensure cameras section exists
        if "cameras" not in config or not isinstance(config["cameras"], dict):
            config["cameras"] = {}

        # Update front camera configuration
        if front_camera_id is not None:
            if "front" not in config["cameras"]:
                config["cameras"]["front"] = {
                    "color_mode": "rgb",
                    "fps": 30,
                    "height": 480,
                    "type": "opencv",
                    "width": 640
                }
            config["cameras"]["front"]["index_or_path"] = front_camera_id
            print(t("updated_front_camera", camera_id=front_camera_id))

        # Update wrist camera configuration
        if wrist_camera_id is not None:
            if "wrist" not in config["cameras"]:
                config["cameras"]["wrist"] = {
                    "color_mode": "rgb",
                    "fps": 30,
                    "height": 480,
                    "type": "opencv",
                    "width": 640
                }
            config["cameras"]["wrist"]["index_or_path"] = wrist_camera_id
            print(t("updated_wrist_camera", camera_id=wrist_camera_id))

        with open(config_path, "w") as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)

        print(t("camera_config_saved", config_path=config_path))

    except Exception as e:
        print(t("camera_config_failed", error=e))
        print(t("manual_set_cameras", config_path=config_path))


def find_camera_by_difference(camera_name: str) -> int | None:
    """
    Find a camera by unplugging and plugging it back in.
    Following the same pattern as find_motor_port.py
    """
    print(f"Please make sure the {camera_name} camera is UNPLUGGED and press Enter to continue.")
    input()
    cameras_before = find_all_opencv_cameras()
    camera_ids_before = {cam["id"] for cam in cameras_before}

    time.sleep(0.5)

    print(f"Please connect the {camera_name} camera and press Enter to continue.")
    input()
    cameras_after = find_all_opencv_cameras()
    camera_ids_after = {cam["id"] for cam in cameras_after}
    camera_diff = list(camera_ids_after - camera_ids_before)

    if len(camera_diff) == 1:
        camera_id = camera_diff[0]
        print(f"The camera ID of this {camera_name} camera is '{camera_id}'")
        print(f"DETECTED_{camera_name.upper()}_CAMERA={camera_id}")
        return camera_id
    elif len(camera_diff) == 0:
        raise OSError(
            f"Could not detect the {camera_name} camera. No difference was found ({camera_diff})."
        )
    else:
        raise OSError(
            f"Could not detect the {camera_name} camera. More than one camera was found ({camera_diff})."
        )


def find_cameras_hardcoded():
    """
    Hardcoded camera detection - platform-specific assignment:
    - Ubuntu: Use external cameras (video4+ devices) - typically /dev/video4 (front) and /dev/video6 (wrist)
    - macOS: Use first two available cameras (camera 0 to front, camera 1 to wrist)
    """
    print(t("hardcoded_detection"))

    if platform.system() == "Linux":
        # Ubuntu: Filter for external cameras and use the last two
        all_cameras = find_all_opencv_cameras(filter_ubuntu_external=True)
        print(t("found_cameras_ubuntu", count=len(all_cameras)))

        if len(all_cameras) >= 2:
            # Sort by camera ID and take the last two (highest numbered external cameras)
            all_cameras.sort(key=lambda cam: str(cam["id"]))
            front_camera_id = all_cameras[-2]["id"]   # Second to last (typically /dev/video4)
            wrist_camera_id = all_cameras[-1]["id"]   # Last camera (typically /dev/video6)

            print(t("assigned_front", camera_id=front_camera_id))
            print(t("assigned_wrist", camera_id=wrist_camera_id))

            # Update config file
            update_camera_config_yaml(front_camera_id, wrist_camera_id)

            return {"front": front_camera_id, "wrist": wrist_camera_id}
        else:
            print(t("not_enough_cameras", count=len(all_cameras)))
            return {"front": None, "wrist": None}
    else:
        # macOS: Use first two available cameras
        all_cameras = find_all_opencv_cameras()
        print(t("found_cameras_macos", count=len(all_cameras)))

        if len(all_cameras) >= 2:
            # Sort by camera ID and take the first two (lowest numbered)
            all_cameras.sort(key=lambda cam: str(cam["id"]))
            front_camera_id = all_cameras[0]["id"]   # First camera (typically 0)
            wrist_camera_id = all_cameras[1]["id"]   # Second camera (typically 1)

            print(t("assigned_front", camera_id=front_camera_id))
            print(t("assigned_wrist", camera_id=wrist_camera_id))

            # Update config file
            update_camera_config_yaml(front_camera_id, wrist_camera_id)

            return {"front": front_camera_id, "wrist": wrist_camera_id}
        else:
            print(t("not_enough_cameras", count=len(all_cameras)))
            return {"front": None, "wrist": None}


def find_cameras():
    """
    Find FRONT and WRIST cameras using hardcoded assignment (last two cameras).
    Similar to motor port detection pattern but without plug/unplug.
    """
    print(t("camera_detection"))
    print("=" * 50)

    # First try hardcoded detection (works for most cases)
    result = find_cameras_hardcoded()
    if result["front"] and result["wrist"]:
        print("=" * 50)
        return result

    print(t("camera_detection_done"))
    print("=" * 50)
    return result


def find_and_print_cameras(camera_type_filter: str | None = None) -> list[dict[str, Any]]:
    """
    Finds available cameras based on an optional filter and prints their information.

    Args:
        camera_type_filter: Optional string to filter cameras ("opencv").
                            If None, lists all opencv cameras.

    Returns:
        A list of all available cameras matching the filter, with their metadata.
    """
    all_cameras_info: list[dict[str, Any]] = []

    if camera_type_filter:
        camera_type_filter = camera_type_filter.lower()

    if camera_type_filter is None or camera_type_filter == "opencv":
        all_cameras_info.extend(find_all_opencv_cameras())

    if not all_cameras_info:
        if camera_type_filter:
            logger.warning(f"No {camera_type_filter} cameras were detected.")
        else:
            logger.warning(t("no_cameras_detected"))
    else:
        print(t("detected_cameras"))
        for i, cam_info in enumerate(all_cameras_info):
            print(t("camera_num", num=i))
            for key, value in cam_info.items():
                if key == "default_stream_profile" and isinstance(value, dict):
                    print(f"  {key.replace('_', ' ').capitalize()}:")
                    for sub_key, sub_value in value.items():
                        print(f"    {sub_key.capitalize()}: {sub_value}")
                else:
                    print(f"  {key.replace('_', ' ').capitalize()}: {value}")
            print("-" * 20)
    return all_cameras_info


def save_image(
    img_array: np.ndarray,
    camera_identifier: str | int,
    images_dir: Path,
    camera_type: str,
    sequence_number: int = 1,
):
    """
    Saves a single image to disk using Pillow. Handles color conversion if necessary.
    """
    try:
        img = Image.fromarray(img_array, mode="RGB")

        # Extract camera port number from identifier
        # Linux: /dev/video0 -> 0, macOS: 0 -> 0, Windows: 1 -> 1
        port_number = "unknown"
        if "/dev/video" in str(camera_identifier):
            # Linux format: /dev/video0 -> 0
            port_number = str(camera_identifier).split("video")[-1]
        else:
            # macOS/Windows format: just use the identifier as is
            port_number = str(camera_identifier)

        filename = f"image{port_number}_{sequence_number:02d}.png"

        path = images_dir / filename
        path.parent.mkdir(parents=True, exist_ok=True)
        img.save(str(path))
        logger.info(f"Saved image: {path}")
    except Exception as e:
        logger.error(f"Failed to save image for camera {camera_identifier} (type {camera_type}): {e}")


def create_camera_instance(cam_meta: dict[str, Any]) -> dict[str, Any] | None:
    """Create and connect to a camera instance based on metadata."""
    cam_type = cam_meta.get("type")
    cam_id = cam_meta.get("id")
    instance = None

    logger.info(f"Preparing {cam_type} ID {cam_id} with default profile")

    try:
        if cam_type == "OpenCV":
            cv_config = OpenCVCameraConfig(
                index_or_path=cam_id,
                color_mode=ColorMode.RGB,
            )
            instance = OpenCVCamera(cv_config)
        else:
            logger.warning(f"Unknown camera type: {cam_type} for ID {cam_id}. Skipping.")
            return None

        if instance:
            logger.info(f"Connecting to {cam_type} camera: {cam_id}...")
            instance.connect(warmup=False)
            return {"instance": instance, "meta": cam_meta}
    except Exception as e:
        logger.error(f"Failed to connect or configure {cam_type} camera {cam_id}: {e}")
        if instance and instance.is_connected:
            instance.disconnect()
        return None


def process_camera_image(
    cam_dict: dict[str, Any], output_dir: Path, current_time: float, sequence_number: int = 1
) -> concurrent.futures.Future | None:
    """Capture and process an image from a single camera."""
    cam = cam_dict["instance"]
    meta = cam_dict["meta"]
    cam_type_str = str(meta.get("type", "unknown"))
    cam_id_str = str(meta.get("id", "unknown"))

    try:
        image_data = cam.read()

        return save_image(
            image_data,
            cam_id_str,
            output_dir,
            cam_type_str,
            sequence_number,
        )
    except TimeoutError:
        logger.warning(
            f"Timeout reading from {cam_type_str} camera {cam_id_str} at time {current_time:.2f}s."
        )
    except Exception as e:
        logger.error(f"Error reading from {cam_type_str} camera {cam_id_str}: {e}")
    return None


def cleanup_cameras(cameras_to_use: list[dict[str, Any]]):
    """Disconnect all cameras."""
    logger.info(f"Disconnecting {len(cameras_to_use)} cameras...")
    for cam_dict in cameras_to_use:
        try:
            if cam_dict["instance"] and cam_dict["instance"].is_connected:
                cam_dict["instance"].disconnect()
        except Exception as e:
            logger.error(f"Error disconnecting camera {cam_dict['meta'].get('id')}: {e}")


def save_images_from_all_cameras(
    output_dir: Path,
    record_time_s: float = 2.0,
    camera_type: str | None = None,
):
    """
    Connects to detected cameras (optionally filtered by type) and saves images from each.
    Uses default stream profiles for width, height, and FPS.

    Args:
        output_dir: Directory to save images.
        record_time_s: Duration in seconds to record images.
        camera_type: Optional string to filter cameras ("realsense" or "opencv").
                            If None, uses all detected cameras.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    logger.info(f"Saving images to {output_dir}")
    all_camera_metadata = find_and_print_cameras(camera_type_filter=camera_type)

    if not all_camera_metadata:
        logger.warning("No cameras detected matching the criteria. Cannot save images.")
        return

    cameras_to_use = []
    for cam_meta in all_camera_metadata:
        camera_instance = create_camera_instance(cam_meta)
        if camera_instance:
            cameras_to_use.append(camera_instance)

    if not cameras_to_use:
        logger.warning("No cameras could be connected. Aborting image save.")
        return

    logger.info(f"Starting image capture for {record_time_s} seconds from {len(cameras_to_use)} cameras.")
    start_time = time.perf_counter()

    # Initialize sequence counters for each camera
    camera_counters = {}
    for cam_dict in cameras_to_use:
        cam_id = cam_dict["meta"]["id"]
        camera_counters[cam_id] = 1

    with concurrent.futures.ThreadPoolExecutor(max_workers=len(cameras_to_use) * 2) as executor:
        try:
            while time.perf_counter() - start_time < record_time_s:
                futures = []
                current_capture_time = time.perf_counter()

                for cam_dict in cameras_to_use:
                    cam_id = cam_dict["meta"]["id"]
                    sequence_num = camera_counters[cam_id]

                    # Increment counter for this camera BEFORE submitting the task
                    camera_counters[cam_id] += 1

                    future = process_camera_image(cam_dict, output_dir, current_capture_time, sequence_num)
                    if future:
                        futures.append(future)

                if futures:
                    concurrent.futures.wait(futures)

                # Add a small delay to avoid overloading the cameras
                time.sleep(0.1)

        except KeyboardInterrupt:
            logger.info("Capture interrupted by user.")
        finally:
            print("\nFinalizing image saving...")
            executor.shutdown(wait=True)
            cleanup_cameras(cameras_to_use)
            print(f"Image capture finished. Images saved to {output_dir}")


def main():
    import os

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s"
    )

    parser = argparse.ArgumentParser(
        description="Camera detection and listing utility."
    )

    parser.add_argument(
        "--detect",
        action="store_true",
        help="Detect FRONT and WRIST cameras using plug/unplug method"
    )
    parser.add_argument(
        "--save-images",
        action="store_true",
        help="Save test images from detected cameras"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default="outputs/camera_images",
        help="Directory to save images. Default: outputs/camera_images",
    )
    parser.add_argument(
        "--record-time-s",
        type=float,
        default=2.0,
        help="Time duration to attempt capturing frames. Default: 2 seconds.",
    )

    args = parser.parse_args()

    # Select language if not set via environment
    select_language()

    if args.detect:
        find_cameras()
        # Force clean exit to avoid memory corruption during cleanup
        os._exit(0)
    elif args.save_images:
        save_images_from_all_cameras(
            output_dir=args.output_dir,
            record_time_s=args.record_time_s,
            camera_type="opencv"
        )
        # Force clean exit to avoid memory corruption during cleanup
        os._exit(0)
    else:
        # Default behavior: find and assign cameras, then print results
        find_and_print_cameras()
        print("\n" + "=" * 50)
        find_cameras()
        # Force clean exit to avoid memory corruption during cleanup
        os._exit(0)


if __name__ == "__main__":
    main()
