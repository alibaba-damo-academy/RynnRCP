# rcp_core/common/adapter/output_adapter/__init__.py
from .generic_step_output_adapter import GenericStepOutputAdapter
from .generic_chunk_output_adapter import GenericChunkOutputAdapter

from .lcm_joint_state_output_adapter import LcmJointStateOutputAdapter
from .lcm_joint_trajectory_output_adapter import LcmJointTrajectoryOutputAdapter

from .ros2_joint_state_output_adapter import Ros2JointStateOutputAdapter
from .ros2_joint_trajectory_output_adapter import Ros2JointTrajectoryOutputAdapter

from .module_step_output_adapter import ModuleStepOutputAdapter
from .module_chunk_output_adapter import ModuleChunkOutputAdapter

OUTPUT_ADAPTER_REGISTRY = {
    "GenericStepOutputAdapter": GenericStepOutputAdapter,
    "GenericChunkOutputAdapter": GenericChunkOutputAdapter,
    "LcmJointStateOutputAdapter": LcmJointStateOutputAdapter,
    "LcmJointTrajectoryOutputAdapter": LcmJointTrajectoryOutputAdapter,
    "Ros2JointStateOutputAdapter": Ros2JointStateOutputAdapter,
    "Ros2JointTrajectoryOutputAdapter": Ros2JointTrajectoryOutputAdapter,
    "ModuleStepOutputAdapter": ModuleStepOutputAdapter,
    "ModuleChunkOutputAdapter": ModuleChunkOutputAdapter,
}

__all__ = [
    "GenericStepOutputAdapter",
    "GenericChunkOutputAdapter",
    "LcmJointStateOutputAdapter",
    "LcmJointTrajectoryOutputAdapter",
    "Ros2JointStateOutputAdapter",
    "Ros2JointTrajectoryOutputAdapter",
    "ModuleStepOutputAdapter",
    "ModuleChunkOutputAdapter",
    "OUTPUT_ADAPTER_REGISTRY",
]
