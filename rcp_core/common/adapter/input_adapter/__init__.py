# rcp_core/common/adapter/__init__.py
from .base_input_adapter import BaseInputAdapter
from .ros2_generic_input_adapter import Ros2GenericInputAdapter
from .ros2_image_input_adapter import Ros2ImageInputAdapter
from .lcm_generic_input_adapter import LcmGenericInputAdapter
from .lcm_image_input_adapter import LcmImageInputAdapter
from .module_image_input_adapter import ModuleImageInputAdapter
from .module_generic_input_adapter import ModuleGenericInputAdapter
from .port_image_input_adapter import PortImageInputAdapter

INPUT_ADAPTER_REGISTRY = {
    "Ros2GenericInputAdapter": Ros2GenericInputAdapter,
    "Ros2ImageInputAdapter": Ros2ImageInputAdapter,
    "LcmGenericInputAdapter": LcmGenericInputAdapter,
    "LcmImageInputAdapter": LcmImageInputAdapter,
    "ModuleImageInputAdapter": ModuleImageInputAdapter,
    "ModuleGenericInputAdapter": ModuleGenericInputAdapter,
    "PortImageInputAdapter": PortImageInputAdapter,
}

__all__ = [
    "BaseInputAdapter",
    "Ros2GenericInputAdapter",
    "Ros2ImageInputAdapter",
    "LcmGenericInputAdapter",
    "LcmImageInputAdapter",
    "ModuleImageInputAdapter",
    "ModuleGenericInputAdapter",
    "PortImageInputAdapter",
    "INPUT_ADAPTER_REGISTRY",
]
