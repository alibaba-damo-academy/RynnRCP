# rcp_core/common/adapter/output_adapter/lcm_joint_trajectory_output_adapter.py

"""
LCM JointTrajectory chunk-mode output adapter.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.adapter.output_adapter.lcm_joint_trajectory_output_adapter.LcmJointTrajectoryOutputAdapter`,
a chunk-mode adapter that converts an action chunk into a single ``lcm_msgs.msg.JointTrajectory``
message.

Behavior:
- reads ``action_chunk[key]`` (typically ``key="action"``) as a sequence of N joint frames
- for each frame, builds a JointState-like point and assigns joint values to ``out_field``
  (commonly ``position``), while also populating the corresponding ``<out_field>_length``
- clears other JointState array fields (name/velocity/effort) to avoid stale values
- packs all points into ``traj_msg.joints`` and sets ``joints_length``

The adapter emits exactly one publish frame with:
- ``interval = N / fps``
- ``step = N``
"""

from typing import Any, Dict, List, Tuple
import numpy as np

from .base_output_adapter import BaseOutputAdapter
from ...utils.get_message_class import get_message_class
from rcp_core.common.utils.logger import server_logger

logger = server_logger()


class LcmJointTrajectoryOutputAdapter(BaseOutputAdapter):
    """
    lcm_msgs.msg.JointTrajectory chunk adapter.

    Design:
      - action_chunk[key] is N frames, each frame is a joint positions vector.
      - Outputs a single JointTrajectory message:
          joints: N JointState-like points
              joints[i].<out_field>_length = number of joints in frame i
              joints[i].<out_field>        = joint values for frame i (usually positions)
    """

    def build_frames(
        self,
        action_chunk: Dict[str, Any],
        outputs: List[Dict[str, Any]],
        fps: int,
    ) -> List[List[Tuple[str, str, Any, float, int]]]:
        """Build a JointTrajectory chunk message from action_chunk and outputs."""

        if not outputs:
            logger.warning("[LcmJointTrajectoryOutputAdapter] outputs is empty")
            return []

        out = outputs[0]
        protocol = out.get("protocol")
        topic = out.get("topic")
        type_str = out.get("type")  # "lcm_msgs.msg.JointTrajectory"
        mappings = out.get("mappings", [])

        if not protocol or not topic or not type_str:
            logger.warning(
                f"[LcmJointTrajectoryOutputAdapter] invalid output config: {out}"
            )
            return []

        if not mappings:
            logger.warning("[LcmJointTrajectoryOutputAdapter] mappings is empty")
            return []

        mapping = mappings[0]
        key = mapping.get("key")  # "action"
        out_field = mapping.get("out_field")  # "position"

        if key not in action_chunk:
            logger.warning(
                f"[LcmJointTrajectoryOutputAdapter] '{key}' not in action_chunk"
            )
            return []

        frames = action_chunk[key]
        if not isinstance(frames, (list, tuple)) or not frames:
            logger.warning(
                f"[LcmJointTrajectoryOutputAdapter] action_chunk['{key}'] "
                f"is empty or not list/tuple"
            )
            return []

        # JointTrajectory & JointState types
        TrajMsg = get_message_class(type_str)
        js_type_str = type_str.rsplit(".", 1)[0] + ".JointState"
        JointStateMsg = get_message_class(js_type_str)

        traj_msg = TrajMsg()
        joints: List[Any] = []

        for frame_data in frames:
            # Convert to list[float]
            if isinstance(frame_data, np.ndarray):
                frame_list = frame_data.astype(float).tolist()
            elif isinstance(frame_data, (list, tuple)):
                frame_list = [float(v) for v in frame_data]
            else:
                logger.error(
                    "[LcmJointTrajectoryOutputAdapter] unsupported frame_data type:",
                    type(frame_data),
                )
                continue

            js = JointStateMsg()

            js.header.stamp_sec = 0
            js.header.stamp_nanosec = 0
            js.header.frame_id = ""

            if out_field and hasattr(js, out_field):
                setattr(js, out_field, frame_list)
            else:
                logger.error(
                    f"[LcmJointTrajectoryOutputAdapter] invalid out_field '{out_field}' "
                    f"for JointState type {JointStateMsg}"
                )
                continue

            # Mechanically fill the corresponding *_length field
            length_field = out_field + "_length"
            if hasattr(js, length_field):
                setattr(js, length_field, len(frame_list))

            # Clear other fields
            if hasattr(js, "name_length"):
                js.name_length = 0
                js.name = []
            if hasattr(js, "velocity_length"):
                js.velocity_length = 0
                js.velocity = []
            if hasattr(js, "effort_length"):
                js.effort_length = 0
                js.effort = []

            joints.append(js)

        traj_msg.joints = joints
        traj_msg.joints_length = len(joints)

        num_frames = len(joints)
        interval_chunk = float(num_frames) / fps if fps > 0 else 0.0
        step_chunk = num_frames

        return [[(protocol, topic, traj_msg, interval_chunk, step_chunk)]]
