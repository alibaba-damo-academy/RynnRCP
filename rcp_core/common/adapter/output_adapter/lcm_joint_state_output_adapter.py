# rcp_core/common/adapter/output_adapter/lcm_joint_state_output_adapter.py

"""
LCM JointState step-mode output adapter.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.adapter.output_adapter.lcm_joint_state_output_adapter.LcmJointStateOutputAdapter`,
a specialized output adapter for publishing ``lcm_msgs.msg.JointState`` messages from
high-level action chunks.

Unlike the generic step adapter, this implementation handles LCM-specific conventions
by:
- converting each action frame into a ``list[float]`` and assigning it to the mapped
  ``out_field`` (typically ``position``)
- automatically populating the corresponding ``<out_field>_length`` field when present
- clearing other JointState array fields (``name``, ``velocity``, ``effort``) and their
  length fields to avoid sending stale/garbage values

The adapter emits one message per logical frame with ``interval = 1/fps`` and ``step = 1``.
"""

from typing import Any, Dict, List, Tuple
import numpy as np

from .base_output_adapter import BaseOutputAdapter
from ...utils.get_message_class import get_message_class
from rcp_core.common.utils.logger import server_logger

logger = server_logger()


class LcmJointStateOutputAdapter(BaseOutputAdapter):
    """
    Step-mode adapter specifically for lcm_msgs.msg.JointState.

    Assumes:
      - action_chunk['action'] is a list of frames
      - Each frame is a 1D array/list: [j0, j1, ..., jN-1]
      - In the config, the mapping's out_field is usually "position"
        Example:
          outputs:
            - protocol: lcm
              topic: /so100/act_cmd
              type: lcm_msgs.msg.JointState
              adapter: LcmJointStateOutputAdapter
              mappings:
                - key: action
                  out_field: position
    """

    def build_frames(
        self,
        action_chunk: Dict[str, Any],
        outputs: List[Dict[str, Any]],
        fps: int,
    ) -> List[List[Tuple[str, str, Any, float, int]]]:
        """Build per-step JointState messages from action_chunk and outputs."""

        frames = action_chunk["action"]
        if not isinstance(frames, (list, tuple)) or not frames:
            logger.warning(
                "[LcmJointStateOutputAdapter] 'action' not in action_chunk, not list/tuple"
            )
            return []

        out = outputs[0]
        protocol = out.get("protocol")
        topic = out.get("topic")
        type_str = out.get("type")  # "lcm_msgs.msg.JointState"
        mappings = out.get("mappings", [])

        if not protocol or not topic or not type_str:
            logger.warning(
                "[LcmJointStateOutputAdapter] invalid output config: %s", out
            )
            return []

        if not mappings:
            logger.warning("[LcmJointStateOutputAdapter] mappings is empty")
            return []

        mapping = mappings[0]
        key = mapping.get("key")
        out_field = mapping.get("out_field")  # "position"

        if key != "action":
            logger.warning(
                f"[LcmJointStateOutputAdapter] only support key 'action', got '{key}'"
            )
            return []

        Msg = get_message_class(type_str)
        interval = 1.0 / fps if fps > 0 else 0.0

        frames_msgs: List[List[Tuple[str, str, Any, float, int]]] = []

        for frame_data in frames:
            # Convert to list[float]
            if isinstance(frame_data, np.ndarray):
                frame_list = frame_data.tolist()
            elif isinstance(frame_data, (list, tuple)):
                frame_list = list(frame_data)
            else:
                logger.error(
                    "[LcmJointStateOutputAdapter] unsupported frame_data type:",
                    type(frame_data),
                )
                continue

            msg = Msg()

            # Mechanically fill the corresponding *_length field
            setattr(msg, out_field, frame_list)
            length_field = out_field + "_length"
            if hasattr(msg, length_field):
                setattr(msg, length_field, len(frame_list))

            # Clear other fields to avoid garbage values on the decoding side
            if hasattr(msg, "name_length"):
                msg.name_length = 0
                msg.name = []
            if hasattr(msg, "velocity_length"):
                msg.velocity_length = 0
                msg.velocity = []
            if hasattr(msg, "effort_length"):
                msg.effort_length = 0
                msg.effort = []

            frames_msgs.append([(protocol, topic, msg, interval, 1)])

        return frames_msgs
