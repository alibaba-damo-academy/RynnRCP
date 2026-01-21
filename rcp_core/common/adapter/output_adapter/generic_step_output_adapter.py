# rcp_core/common/adapter/output_adapter/generic_step_output_adapter.py

"""
Generic step-mode output adapter.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.adapter.output_adapter.generic_step_output_adapter.GenericStepOutputAdapter`,
an output adapter that publishes an action chunk frame-by-frame.

Input:
    ``action_chunk = {"action": [frame0, frame1, ..., frameN-1]}``, where each
    frame is a 1-D sequence.

Behavior:
- expects a single output configuration with a single mapping whose key is
  ``"action"``
- constructs one concrete message per frame using :func:`~rcp_core.common.utils.get_message_class.get_message_class`
- assigns the frame data to the configured ``out_field``
- returns ``N`` frames with ``interval = 1/fps`` and ``step = 1``

Note:
    This adapter does not handle message types that require explicit ``*_length``
    fields or other special encoding rules; use a dedicated adapter for such types.
"""

from typing import Any, Dict, List, Tuple
from .base_output_adapter import BaseOutputAdapter
from ...utils.get_message_class import get_message_class
from rcp_core.common.utils.logger import server_logger

logger = server_logger()


class GenericStepOutputAdapter(BaseOutputAdapter):
    """
    Step mode:
    - Input:
        action_chunk = { "action": [frame0, frame1, ..., frameN-1] }
      where each frame_i is a 1D array/list.
    - Config (outputs) only supports:
        - a single output
        - a single mapping
        - mapping key must be "action"
    - Output:
        frames_msgs: List[List[(protocol, topic, msg, interval, step)]]
        length N:
          one (protocol, topic, msg, interval, step) per frame
          interval = 1.0 / fps
          step = 1

    Note:
      For messages like LCM JointState that have *_length fields, this generic
      adapter will NOT automatically set the length; use a specific adapter
      such as LcmJointStateOutputAdapter instead.
    """

    def build_frames(
        self,
        action_chunk: Dict[str, Any],
        outputs: List[Dict[str, Any]],
        fps: int,
    ) -> List[List[Tuple[str, str, Any, float, int]]]:
        """Build per-step frames from action_chunk using a single output mapping."""

        frames = action_chunk["action"]
        if not isinstance(frames, (list, tuple)) or not frames:
            logger.warning(
                "[GenericStepOutputAdapter] 'action' in action_chunk is empty or not list/tuple"
            )
            return []

        out = outputs[0]
        protocol = out.get("protocol")
        topic = out.get("topic")
        type_str = out.get("type")
        mappings = out.get("mappings", [])

        if not protocol or not topic or not type_str:
            logger.warning(f"[GenericStepOutputAdapter] invalid output config: {out}")
            return []

        if not mappings:
            logger.warning(f"[GenericStepOutputAdapter] mappings is empty: {out}")
            return []

        mapping = mappings[0]
        key = mapping.get("key")
        out_field = mapping.get("out_field")

        if key != "action":
            logger.warning(
                f"[GenericStepOutputAdapter] only support key 'action', got '{key}'"
            )
            return []

        Msg = get_message_class(type_str)
        interval = 1.0 / fps if fps > 0 else 0.0

        frames_msgs: List[List[Tuple[str, str, Any, float, int]]] = []

        for frame_data in frames:
            msg = Msg()
            setattr(msg, out_field, frame_data)
            frames_msgs.append([(protocol, topic, msg, interval, 1)])

        return frames_msgs
