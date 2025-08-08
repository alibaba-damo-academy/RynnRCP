#!/usr/bin/env python3
"""
Robot Configuration Script

This script allows users to configure robot settings through
interactive prompts and device detection.
"""

import yaml
import os
import time
import cv2
import sys
from pathlib import Path
import subprocess
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

# Configuration file paths - using absolute paths based on script location
SCRIPT_DIR = Path(__file__).parent.absolute()
DEVICE_CONFIG_PATH = str(
    SCRIPT_DIR.parent / "rcp_framework/robots/so100/config/device_config.yaml")
CAMERAS_CONFIG_PATH = str(
    SCRIPT_DIR.parent / "rcp_framework/robots/so100/config/cameras.yaml")
ROBOT_CONFIG_PATH = str(
    SCRIPT_DIR.parent / "robot_motion/robots/lerobot/configs/config.yaml")


def load_yaml_config(filepath):
    """Load YAML configuration file"""
    try:
        logging.info(f"Loading configuration from {filepath}")
        with open(filepath, 'r') as file:
            config = yaml.safe_load(file) or {}
            return config
    except FileNotFoundError:
        logging.warning(f"Configuration file not found: {filepath}")
        return {}
    except yaml.YAMLError as e:
        logging.error(f"Failed to load YAML file {filepath}: {e}")
        sys.exit(1)


def save_yaml_config(filepath, config):
    """Save configuration to YAML file"""
    try:
        logging.info(f"Saving configuration to {filepath}")
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, 'w') as file:
            yaml.dump(config, file, default_flow_style=False,
                      allow_unicode=True)
    except Exception as e:
        logging.error(f"Failed to save configuration to {filepath}: {e}")
        sys.exit(1)


def input_with_default(prompt, default_value):
    """Get user input with a default value"""
    try:
        user_input = input(
            f"{prompt} (default: [{default_value}], Enter to use default): ").strip()
        result = user_input if user_input else default_value
        logging.info(f"User input for '{prompt}': {result}")
        return result
    except KeyboardInterrupt:
        logging.info("Operation cancelled by user")
        sys.exit(1)


def detect_serial_devices():
    """Detect available serial devices"""
    try:
        logging.info("Detecting serial devices...")
        time.sleep(2)
        devices = [str(path) for path in Path("/dev").glob("tty*")]
        return devices
    except Exception as e:
        logging.error(f"Failed to detect serial devices: {e}")
        sys.exit(1)
    return []


def detect_camera_devices():
    """Detect available camera devices"""
    try:
        logging.info("Detecting camera devices...")
        time.sleep(2)
        devices = [str(path) for path in Path("/dev").glob("video*")]
        return devices
    except Exception as e:
        logging.error(f"Failed to detect camera devices: {e}")
        sys.exit(1)
    return []


def wait_for_serial_device_change():
    """Wait for user to plug serial device and detect changes"""

    try:
        logging.info("Starting serial device detection process")
        input("Please unplug the serial device, then press Enter...")
        current_devices = detect_serial_devices()
        logging.info(f"Current serial devices: {current_devices}")

        input("Please plug in the serial device now, then press Enter...")
        after_plug_devices = detect_serial_devices()

        new_devices = list(set(after_plug_devices) - set(current_devices))
        logging.info(f"New devices detected: {new_devices}")

        if len(new_devices) > 1:
            logging.error(
                f"Found {len(new_devices)} new serial devices. Expected only one.")
            logging.error(f"New devices: {new_devices}")
            sys.exit(1)
        elif len(new_devices) == 1:
            return new_devices[0]
        else:
            logging.error("No new serial device detected")
            sys.exit(1)
    except KeyboardInterrupt:
        logging.info("Operation cancelled by user")
        sys.exit(1)


def wait_for_camera_device_change():
    """Wait for user to plug camera device and detect changes"""

    try:
        logging.info("Starting camera device detection process")
        input("Please unplug the camera device, then press Enter...")
        initial_cameras = detect_camera_devices()
        logging.info(f"Current camera devices: {initial_cameras}")

        # Wait for user to plug camera device
        input("Please plug in the camera device now, then press Enter...")
        after_plug_devices = detect_camera_devices()

        # Get devices after plugging
        new_devices = list(set(after_plug_devices) - set(initial_cameras))
        logging.info(f"New devices detected: {new_devices}")

        # Handle multiple new devices
        if not new_devices:
            logging.error("No new camera devices detected")
            sys.exit(1)

        logging.info(
            f"Found {len(new_devices)} new camera devices. Testing each one...")
        for device in new_devices:
            if test_camera_device(device):
                logging.info(
                    f"Using camera device {device} as it successfully captured an image")
                return device
        logging.error("No camera devices could capture images")
        sys.exit(1)
    except KeyboardInterrupt:
        logging.info("Operation cancelled by user")
        sys.exit(1)


def test_camera_device(device_path):
    """Test if camera device can capture images"""
    try:
        logging.info(f"Testing camera device: {device_path}")
        # Try to open the camera
        cap = cv2.VideoCapture(device_path)
        if not cap.isOpened():
            logging.warning(f"Cannot open camera {device_path}")
            return False

        ret, frame = cap.read()
        cap.release()
        if ret:
            logging.info(
                f"Successfully captured frame from {device_path} (shape: {frame.shape})")
            return True
        else:
            logging.warning(f"Cannot read frame from {device_path}")
            return False
    except Exception as e:
        logging.error(f"Error testing camera device {device_path}: {e}")
        sys.exit(1)


def configure_device_settings():
    """Configure device settings"""
    logging.info("Starting device configuration")
    print("\n" + "="*50)
    print("        DEVICE CONFIGURATION")
    print("="*50)
    try:
        config = load_yaml_config(DEVICE_CONFIG_PATH)

        config['http_url'] = input_with_default(
            "Enter HTTP URL",
            config.get('http_url', 'https://robot-access.damo-academy.com')
        )

        config['endpoint_mqtt'] = input_with_default(
            "Enter MQTT endpoint",
            config.get('endpoint_mqtt', '/connect/mqtt')
        )

        config['endpoint_websocket'] = input_with_default(
            "Enter WebSocket endpoint",
            config.get('endpoint_websocket', '/connect/webSocket')
        )

        config['product_key'] = input_with_default(
            "Enter product key",
            config.get('product_key', 'put_product_key_here')
        )

        config['device_name'] = input_with_default(
            "Enter device name",
            config.get('device_name', 'put_device_name_here')
        )

        config['device_secret'] = input_with_default(
            "Enter device secret",
            config.get('device_secret', 'put_device_secret_here')
        )

        save_yaml_config(DEVICE_CONFIG_PATH, config)
        logging.info("Device configuration completed")
    except Exception as e:
        logging.error(f"Error during device configuration: {e}")
        sys.exit(1)


def configure_cameras():
    """Configure camera settings"""
    logging.info("Starting camera configuration")
    print("\n" + "="*50)
    print("        CAMERA CONFIGURATION")
    print("="*50)
    try:
        config = load_yaml_config(CAMERAS_CONFIG_PATH)

        if 'cameras' not in config:
            config['cameras'] = []
            logging.info(
                "No existing camera configuration found. Starting fresh configuration.")

        # Configure each camera
        for i, camera in enumerate(config['cameras']):
            print(
                f"\n--- Configuring Camera {i+1} ({camera.get('name', 'unnamed')}) ---")

            camera['name'] = input_with_default(
                f"Camera {i+1} name",
                camera.get('name', f'camera_{i+1}')
            )

            # Auto-detect camera device by plug
            print(
                f"\n[INSTRUCTION] Please follow the steps to detect camera {i+1}")
            new_camera_device = wait_for_camera_device_change()
            if new_camera_device:
                camera['device'] = new_camera_device
            else:
                logging.error("No new camera device detected")
                sys.exit(1)

            camera['brand'] = input_with_default(
                f"Camera {i+1} brand",
                camera.get('brand', 'Brand Name')
            )

            camera['format'] = input_with_default(
                f"Camera {i+1} format",
                camera.get('format', 'YUYV')
            )

            try:
                width_input = input_with_default(
                    f"Camera {i+1} width",
                    camera.get('width', 640)
                )
                camera['width'] = int(width_input)
                logging.info(
                    f"Camera {i+1} width set to {camera['width']}")
            except ValueError:
                logging.warning("Invalid width, keeping current value")

            try:
                height_input = input_with_default(
                    f"Camera {i+1} height",
                    camera.get('height', 480)
                )
                camera['height'] = int(height_input)
                logging.info(
                    f"Camera {i+1} height set to {camera['height']}")
            except ValueError:
                logging.warning("Invalid height, keeping current value")

            try:
                framerate_input = input_with_default(
                    f"Camera {i+1} framerate",
                    camera.get('framerate', 30)
                )
                camera['framerate'] = int(framerate_input)
                logging.info(
                    f"Camera {i+1} framerate set to {camera['framerate']}")
            except ValueError:
                logging.warning("Invalid framerate, keeping current value")

        save_yaml_config(CAMERAS_CONFIG_PATH, config)
        logging.info("Camera configuration completed")
    except Exception as e:
        logging.error(f"Error during camera configuration: {e}")
        sys.exit(1)


def configure_robot():
    """Configure robot settings"""
    logging.info("Starting robot configuration")
    print("\n" + "="*50)
    print("        ROBOT CONFIGURATION")
    print("="*50)
    try:
        config = load_yaml_config(ROBOT_CONFIG_PATH)

        # Ensure robot section exists
        if 'robot' not in config:
            config['robot'] = {}
            logging.info(
                "No existing robot configuration found. Starting fresh configuration.")

        # Robot configuration - auto-detect serial device by plug/unplug
        new_serial_device = wait_for_serial_device_change()
        if new_serial_device:
            config['robot']['port'] = new_serial_device
            logging.info(f"Using detected serial port: {new_serial_device}")
            try:
                # Change permissions of the robot port
                logging.info(
                    f"Running command: sudo chmod 666 {new_serial_device} to set device permissions")
                subprocess.run(
                    ['sudo', 'chmod', '666', new_serial_device], check=True)
                logging.info(f"Permissions set to 666 for {new_serial_device}")
            except subprocess.CalledProcessError as e:
                logging.error(
                    f"Failed to set permissions for {new_serial_device}: {e}")
                sys.exit(1)
        else:
            logging.error("No new serial device detected")
            sys.exit(1)

        try:
            current_rate = config['robot'].get('inference_rate', 30.0)
            rate_input = input_with_default(
                "Robot inference rate",
                current_rate
            )
            config['robot']['inference_rate'] = float(rate_input)
            logging.info(
                f"Robot inference rate set to {config['robot']['inference_rate']}")
        except ValueError:
            logging.warning("Invalid inference rate, keeping current value")

        try:
            current_timeout = config['robot'].get('timeout_seconds', 30.0)
            timeout_input = input_with_default(
                "Robot timeout seconds",
                current_timeout
            )
            config['robot']['timeout_seconds'] = float(timeout_input)
            logging.info(
                f"Robot timeout seconds set to {config['robot']['timeout_seconds']}")
        except ValueError:
            logging.warning("Invalid timeout, keeping current value")

        save_yaml_config(ROBOT_CONFIG_PATH, config)
        logging.info("Robot configuration completed")
    except Exception as e:
        logging.error(f"Error during robot configuration: {e}")
        sys.exit(1)


def calibrate_robot_arm():
    """Calibrate the robot arm by running the calibration script"""
    logging.info("Starting robot arm calibration")
    print("\n" + "="*50)
    print("        ROBOT ARM CALIBRATION")
    print("="*50)
    try:
        # Confirm with user before starting calibration
        print("The calibration process will check the robot arm properties and generate")
        print("a calibration parameter configuration file specific to the robot arm.")
        print("To avoid calibration failure, please ensure that the robot arm is")
        print("adjusted to the zero position before running the calibration process.")
        confirm = input(
            "Start the calibration process? (y/N): ").strip().lower()
        if confirm not in ['y', 'yes']:
            logging.info("Calibration cancelled by user")
            return

        # Change to the required directory and run the calibration command
        robot_dir = SCRIPT_DIR.parent / "robot_motion/robots/lerobot"
        if not robot_dir.exists():
            logging.error(f"Directory does not exist: {robot_dir}")
            sys.exit(1)

        logging.info(f"Changing to directory: {robot_dir}")
        logging.info(
            "Running command: python -m scripts.follower_calibrate --mode manual")
        print("-" * 50)

        # Run the calibration command with full terminal interaction
        result = subprocess.run(
            ["python", "-m", "scripts.follower_calibrate", "--mode", "manual"],
            cwd=str(robot_dir)
        )

        if result.returncode == 0:
            logging.info("Calibration completed successfully")
        else:
            logging.error(
                f"Calibration failed with return code: {result.returncode}")
            sys.exit(1)

    except KeyboardInterrupt:
        logging.info("Calibration cancelled by user")
        sys.exit(1)
    except Exception as e:
        logging.error(f"Unexpected error during calibration: {e}")
        sys.exit(1)


def main():
    """Main function"""
    logging.info("Robot Configuration Tool started")
    print("="*60)
    print("                  ROBOT CONFIGURATION TOOL")
    print("="*60)
    print("Welcome to the robot configuration tool!")
    print("Follow the on-screen instructions to configure your robot.")
    print("Check the log file for detailed information.")
    print("Please refer to the user guide of RynnBot platform for more details.\n")

    while True:
        print("\n" + "-"*50)
        print("           MAIN MENU")
        print("-"*50)
        print("Select configuration to modify:")
        print("  1. Device settings (MQTT/WebSocket)")
        print("  2. Camera settings")
        print("  3. Robot settings")
        print("  4. Robot arm calibration")
        print("  5. Configure all")
        print("  q. Quit")
        print("-"*50)

        try:
            choice = input("Enter your choice (1-q): ").strip()
            logging.info(f"User selected option: {choice}")

            if choice == "1":
                configure_device_settings()
            elif choice == "2":
                configure_cameras()
            elif choice == "3":
                configure_robot()
            elif choice == "4":
                calibrate_robot_arm()
            elif choice == "5":
                logging.info("Starting full configuration")
                try:
                    configure_device_settings()
                    configure_cameras()
                    configure_robot()
                    calibrate_robot_arm()
                    logging.info("Full configuration completed successfully")
                except Exception as e:
                    logging.error(f"Error during full configuration: {e}")
                    sys.exit(1)
            elif choice == "q":
                logging.info("Exiting configuration tool")
                break
            else:
                logging.warning(f"Invalid choice: {choice}")
        except KeyboardInterrupt:
            logging.info("Operation cancelled by user")
            sys.exit(1)
        except Exception as e:
            logging.error(f"Unexpected error: {e}")
            sys.exit(1)

    logging.info("Robot Configuration Tool finished")


if __name__ == "__main__":
    main()
