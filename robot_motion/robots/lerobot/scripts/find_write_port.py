# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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
import os
import time
import yaml
from pathlib import Path

from serial.tools import list_ports


def find_available_ports():
    if os.name == "nt":
        ports = [port.device for port in list_ports.comports()]
    else:
        ports = [str(path) for path in Path("/dev").glob("tty*")]
    return ports


def update_config_with_port(port_name):
    """Update config.yaml with the detected port."""
    config_path = "configs/config.yaml"

    try:
        if os.path.exists(config_path):
            with open(config_path, "r") as f:
                config = yaml.safe_load(f) or {}
        else:
            config = {}

        if "robot" not in config:
            config["robot"] = {}

        config["robot"]["port"] = port_name
        with open(config_path, "w") as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)

        print(f"✓ Updated {config_path} with port: {port_name}")

    except Exception as e:
        print(f"⚠ Warning: Could not update config file: {e}")
        print(f"  Please manually set port: '{port_name}' in {config_path}")


def find_port():
    print("Finding all available ports for the MotorsBus.")
    ports_before = find_available_ports()
    print("Ports before disconnecting:", ports_before)

    print("Remove the USB cable from your MotorsBus and press Enter when done.")
    input()

    time.sleep(0.5)
    ports_after = find_available_ports()
    ports_diff = list(set(ports_before) - set(ports_after))

    if len(ports_diff) == 1:
        port = ports_diff[0]
        print(f"The port of this MotorsBus is '{port}'")
        print("Reconnect the USB cable.")
        update_config_with_port(port)

    elif len(ports_diff) == 0:
        raise OSError(
            f"Could not detect the port. No difference was found ({ports_diff})."
        )
    else:
        raise OSError(
            f"Could not detect the port. More than one port was found ({ports_diff})."
        )


if __name__ == "__main__":
    find_port()
