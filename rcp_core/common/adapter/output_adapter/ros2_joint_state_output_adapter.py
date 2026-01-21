# rcp_core/common/adapter/output_adapter/ros2_joint_state_output_adapter.py

"""
ROS 2 JointState step-mode output adapter.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.adapter.output_adapter.ros2_joint_state_output_adapter.Ros2JointStateOutputAdapter`,
a dedicated output adapter for publishing ``sensor_msgs.msg.JointState`` messages from
high-level action chunks.

Compared to the generic step adapter, it:
- converts each frame to a Python ``list`` (from ``numpy.ndarray`` or sequence)
- assigns it to the configured ``out_field`` (typically ``position``)
- clears ``velocity`` and ``effort`` arrays to avoid sending unintended/stale values

The adapter emits one message per logical frame with ``interval = 1/fps`` and ``step = 1``.
"""

from typing import Any, Dict, List, Tuple
import numpy as np

from .base_output_adapter import BaseOutputAdapter
from ...utils.get_message_class import get_message_class
from rcp_core.common.utils.logger import server_logger

logger = server_logger()


class Ros2JointStateOutputAdapter(BaseOutputAdapter):
    """
    Step adapter dedicated to ROS2 sensor_msgs.msg.JointState.

    Assumes:
      - action_chunk['action'] is a list of frames
      - Each frame is a 1D array/list: [j0, j1, ..., jN-1]
      - mappings:
          - key: action
            out_field: position
    """

    def build_frames(
        self,
        action_chunk: Dict[str, Any],
        outputs: List[Dict[str, Any]],
        fps: int,
    ) -> List[List[Tuple[str, str, Any, float, int]]]:
        """Build per-step ROS2 JointState messages from action_chunk and outputs."""

        frames = action_chunk["action"]
        if not isinstance(frames, (list, tuple)) or not frames:
            logger.warning(
                "[Ros2JointStateOutputAdapter] action is empty or not list/tuple"
            )
            return []

        out = outputs[0]
        protocol = out.get("protocol")
        topic = out.get("topic")
        type_str = out.get("type")  # "sensor_msgs.msg.JointState"
        mappings = out.get("mappings", [])

        if not protocol or not topic or not type_str:
            logger.error(f"[LcmJointStateOutputAdapter] invalid output config: {out}")
            return []

        if not mappings:
            logger.warning("[Ros2JointStateOutputAdapter] mappings is empty")
            return []

        mapping = mappings[0]
        key = mapping.get("key")
        out_field = mapping.get("out_field")  # "position"

        if key != "action":
            logger.warning(
                f"[Ros2JointStateOutputAdapter] only support key 'action', got '{key}'"
            )
            return []

        Msg = get_message_class(type_str)
        interval = 1.0 / fps if fps > 0 else 0.0

        frames_msgs: List[List[Tuple[str, str, Any, float, int]]] = []

        for frame_data in frames:
            if isinstance(frame_data, np.ndarray):
                frame_list = frame_data.tolist()
            elif isinstance(frame_data, (list, tuple)):
                frame_list = list(frame_data)
            else:
                logger.error(
                    "[Ros2JointStateOutputAdapter] unsupported frame_data type:",
                    type(frame_data),
                )
                continue

            msg = Msg()

            setattr(msg, out_field, frame_list)
            msg.velocity = []
            msg.effort = []

            frames_msgs.append([(protocol, topic, msg, interval, 1)])

        return frames_msgs
