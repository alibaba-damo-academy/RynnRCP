# rcp_core/common/adapter/output_adapter/module_step_output_adapter.py

"""
Module step-mode output adapter.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.adapter.output_adapter.module_step_output_adapter.ModuleStepOutputAdapter`,
an output adapter for the ``module`` protocol that publishes an action chunk frame-by-frame.

It expects a module output configuration that uses ``params.dynamic_arg`` in *list* form
(e.g. ``[{ "from_key": "action", "arg": "positions" }]``). For each frame in
``action_chunk["action"]``, the adapter produces a message dict of the form
``{"action": frame_i}`` and returns ``N`` frames with ``interval = 1/fps`` and ``step = 1``.

Assembly of actual Python function arguments from ``dynamic_arg`` / ``static_args`` is
handled downstream by the module protocol publisher (``ModuleAdapter.pub``).
"""

from typing import Any, Dict, List, Tuple

from .base_output_adapter import BaseOutputAdapter
from rcp_core.common.utils.logger import server_logger

logger = server_logger()


class ModuleStepOutputAdapter(BaseOutputAdapter):
    """
    Step mode module output adapter (only supports dynamic_arg in list format):

      Configuration example:
        dynamic_arg:
          - from_key: action
            arg: positions
        static_args:
          - radians: true

      Requirements:
        action_chunk[from_key] = [frame0, frame1, ..., frameN-1]

      Output:
        Generates one (protocol, params, msg_dict, interval, step) for each frame
        where msg_dict must at least contain:
          {from_key: frame_i}
        How to assemble function parameters based on dynamic_arg/static_args
        is managed by ModuleAdapter.pub.
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

        # Only supports list format for dynamic_arg: List[ {from_key, arg} ]
        dyn_list = params.get("dynamic_arg") or []
        if not isinstance(dyn_list, list) or not dyn_list:
            raise TypeError(
                "[ModuleStepOutputAdapter] params.dynamic_arg must be a non-empty list"
            )

        first = dyn_list[0]
        if not isinstance(first, dict):
            raise TypeError(
                "[ModuleStepOutputAdapter] params.dynamic_arg[0] must be a dict"
            )

        from_key = first.get("from_key") or "action"

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

        interval = 1.0 / fps if fps > 0 else 0.0

        frames_msgs: List[List[Tuple[str, Dict[str, Any], Any, float, int]]] = []

        for frame_data in frames:
            # msg_dict will be passed to ModuleAdapter.pub as msg_dict
            msg_dict = {from_key: frame_data}
            frames_msgs.append([(protocol, params, msg_dict, interval, 1)])

        return frames_msgs
