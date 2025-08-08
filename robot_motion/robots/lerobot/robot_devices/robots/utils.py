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

from typing import Protocol

from robot_devices.robots.configs import RobotConfig, So100RobotConfig, So101RobotConfig


def get_arm_id(name, arm_type):
    """Return arm identifier for naming conventions."""
    return f"{name}_{arm_type}"


class Robot(Protocol):
    """Protocol defining the interface for robot implementations."""

    robot_type: str
    features: dict

    def connect(self): ...

    def disconnect(self): ...


def make_robot_config(robot_type: str, **kwargs) -> RobotConfig:
    """
    Factory function to create and return a robot configuration.
    Supports so100 and so101 robot types for this deployment.
    """
    if robot_type == "so100":
        return So100RobotConfig(**kwargs)
    elif robot_type == "so101":
        return So101RobotConfig(**kwargs)
    else:
        raise ValueError(
            f"Unsupported robot type: {robot_type}. Supported types: 'so100', 'so101'."
        )


def make_robot_from_config(config: RobotConfig):
    """Create a robot instance from configuration."""
    if config.type == "so100":
        from robot_devices.robots.so100 import So100Robot

        return So100Robot(config=config)
    elif config.type == "so101":
        from robot_devices.robots.so101 import So101Robot

        return So101Robot(config=config)
    else:
        raise ValueError(
            f"Unsupported robot type: {config.type}. Supported types: 'so100', 'so101'."
        )


def make_robot(robot_type: str, **kwargs) -> Robot:
    """
    Factory function to create and return a robot instance.
    Supports so100 and so101 robot types for this deployment.
    """
    config = make_robot_config(robot_type, **kwargs)
    return make_robot_from_config(config)
