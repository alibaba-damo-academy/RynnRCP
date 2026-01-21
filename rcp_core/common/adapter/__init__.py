# rcp_core/common/adapter/__init__.py
# Input adapters
from .input_adapter import (
    BaseInputAdapter,
    Ros2GenericInputAdapter,
    Ros2ImageInputAdapter,
    LcmGenericInputAdapter,
    LcmImageInputAdapter,
    INPUT_ADAPTER_REGISTRY,
)

# Output adapters
from .output_adapter import (
    GenericStepOutputAdapter,
    GenericChunkOutputAdapter,
    LcmJointStateOutputAdapter,
    LcmJointTrajectoryOutputAdapter,
    Ros2JointStateOutputAdapter,
    Ros2JointTrajectoryOutputAdapter,
    OUTPUT_ADAPTER_REGISTRY,
)

# Unified ADAPTER_REGISTRY: includes both input and output adapters
ADAPTER_REGISTRY = {}
ADAPTER_REGISTRY.update(INPUT_ADAPTER_REGISTRY)
ADAPTER_REGISTRY.update(OUTPUT_ADAPTER_REGISTRY)

__all__ = [
    # input
    "BaseInputAdapter",
    "Ros2GenericInputAdapter",
    "Ros2ImageInputAdapter",
    "LcmGenericInputAdapter",
    "LcmImageInputAdapter",
    "INPUT_ADAPTER_REGISTRY",
    # output
    "GenericStepOutputAdapter",
    "GenericChunkOutputAdapter",
    "LcmJointStateOutputAdapter",
    "LcmJointTrajectoryOutputAdapter",
    "Ros2JointStateOutputAdapter",
    "Ros2JointTrajectoryOutputAdapter",
    # unified
    "ADAPTER_REGISTRY",
]
