# rcp_core/common/adapter/output_adapter/module_chunk_output_adapter.py

"""
Module chunk-mode output adapter.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.adapter.output_adapter.module_chunk_output_adapter.ModuleChunkOutputAdapter`,
an output adapter for the ``module`` protocol that publishes an entire action chunk in
one call/message.

It expects ``action_chunk["action"]`` to be a sequence of frames and forwards that
2-D structure as the outgoing ``msg`` (i.e., no per-frame message construction). The
mapping is controlled by ``outputs[0].params.arg_mapping.from_key`` (default: ``"action"``),
and currently only ``"action"`` is supported.

The adapter emits exactly one publish frame with:
- ``interval = N / fps``
- ``step = N``
where ``N`` is the number of logical frames in the chunk.
"""

from typing import Any, Dict, List, Tuple

from .base_output_adapter import BaseOutputAdapter
from rcp_core.common.utils.logger import server_logger

logger = server_logger()


class ModuleChunkOutputAdapter(BaseOutputAdapter):
    """
    Chunk mode module output adapter:
      - action_chunk: { "action": [frame0, frame1, ..., frameN-1] }
      - The msg = the entire action_chunk[from_key] two-dimensional array at once.
    """

    def build_frames(
        self,
        action_chunk: Dict[str, Any],
        outputs: List[Dict[str, Any]],
        fps: int,
    ) -> List[List[Tuple[str, Dict[str, Any], Any, float, int]]]:

        out = outputs[0]
        protocol = out.get("protocol")
        params = out.get("params", {}) or {}

        arg_mapping = params.get("arg_mapping") or {}
        from_key = arg_mapping.get("from_key") or "action"

        if from_key != "action":
            logger.warning(
                f"[ModuleChunkOutputAdapter] only support key 'action', got '{from_key}'"
            )
            return []

        if from_key not in action_chunk:
            logger.warning(
                f"[ModuleChunkOutputAdapter] '{from_key}' not in action_chunk"
            )
            return []

        frames = action_chunk[from_key]

        msg = frames
        num_frames = len(frames)
        interval = float(num_frames) / fps if fps > 0 else 0.0
        step_chunk = num_frames

        return [[(protocol, params, msg, interval, step_chunk)]]
