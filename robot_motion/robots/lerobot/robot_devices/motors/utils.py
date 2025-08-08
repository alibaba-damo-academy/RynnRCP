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

from robot_devices.motors.configs import MotorsBusConfig


class MotorsBus(Protocol):
    motor_names: list[str]
    motor_models: list[str]

    def connect(self): ...

    def disconnect(self): ...


def make_motors_buses_from_configs(
    motors_bus_configs: dict[str, MotorsBusConfig],
) -> list[MotorsBus]:
    """
    Factory function to create and return a list of motors buses from the
    provided configuration.
    """
    motors_buses = []
    for bus_config in motors_bus_configs.values():
        motors_buses.append(make_motors_bus(bus_config.type, **bus_config.__dict__))
    return motors_buses


def make_motors_bus(motor_type: str, **kwargs) -> MotorsBus:
    """
    Factory function to create and return a motors bus instance.
    Only supports Feetech motors for this deployment.
    """
    if motor_type == "feetech":
        from robot_devices.motors.feetech import FeetechMotorsBus

        return FeetechMotorsBus(**kwargs)
    else:
        raise ValueError(
            f"Unsupported motor type: {motor_type}. Only 'feetech' is supported."
        )
