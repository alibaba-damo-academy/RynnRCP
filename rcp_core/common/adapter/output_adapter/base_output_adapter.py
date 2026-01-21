# rcp_core/common/adapter/output_adapter/base_output_adapter.py

"""
Base interface for output adapters.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.adapter.output_adapter.base_output_adapter.BaseOutputAdapter`,
the contract used by action execution components to turn a high-level ``action_chunk``
into concrete middleware/protocol messages.

An output adapter implements :meth:`BaseOutputAdapter.build_frames`, which returns a
nested list describing what to publish for each logical frame (or chunk). Each inner
entry is a tuple ``(protocol, topic, msg, interval, step)`` where ``interval`` is the
desired time gap to the next publish and ``step`` indicates how many logical steps the
frame represents (useful for accounting/statistics).

Concrete output adapters live under ``rcp_core.common.adapter.output_adapter``.
"""

from typing import Any, Dict, List, Tuple


class BaseOutputAdapter:
    """
    Base class for all OutputAdapters, defining a unified interface:
      build_frames(action_chunk, outputs, fps) -> frames_msgs

    Returns:
      frames_msgs: List[List[(protocol, topic, msg, interval, step)]]
        - Outer list: logical frame / chunk count
        - Inner list: one frame may contain multiple messages (usually just one)
        - Tuple:
            protocol: str (e.g. 'lcm', 'ros2', 'module')
            topic:    str
            msg:      concrete message object (already constructed)
            interval: float, time interval in seconds from the previous frame
            step:     int, how many logical "steps" this frame represents (for statistics)
    """

    def build_frames(
        self,
        action_chunk: Dict[str, Any],
        outputs: List[Dict[str, Any]],
        fps: int,
    ) -> List[List[Tuple[str, str, Any, float, int]]]:
        """Build a nested list of frame messages from action_chunk and outputs."""
        raise NotImplementedError
