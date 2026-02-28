# rcp_core/action_server/action_server.py

"""
Action execution server.
~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.action_server.action_server.ActionServer`, a
:class:`~rcp_core.common.server.base_server.BaseServer` implementation responsible
for executing high-level action chunks.

The server exposes two main capabilities:
- ``get_state()``: snapshots buffered observations and aligns them by trigger time
  (via :func:`~rcp_core.common.utils.sync_frames.sync_by_trigger_time`) to produce a
  consistent state dict.
- ``run_action_chunk()``: validates and publishes a sequence of action frames at a
  target FPS using the configured output adapter from :data:`~rcp_core.common.adapter.ADAPTER_REGISTRY`.

When bound to an :class:`~rcp_core.common.bus.rcp_bus.RcpBus`, these capabilities are
registered as tools (``get_state`` / ``run_action_chunk``) for other components to call.
"""

import time
from array import array
from typing import Dict, Any, List

from ..common.server.base_server import BaseServer
from ..common.bus.rcp_bus import RcpBus

from ..common.adapter import ADAPTER_REGISTRY

from ..common.utils.sync_frames import sync_by_trigger_time
from rcp_core.common.utils.logger import server_logger

logger = server_logger()


class ActionServer(BaseServer):
    """Action execution server that converts high-level action chunks into middleware messages."""

    def __init__(self, config: Dict[str, Any]):
        """Initialize the action server with the given configuration."""
        super().__init__(config, "action_server")

    def get_state(self) -> Dict[str, Any]:
        """Synchronize all keys in the local buffer and return the state."""
        snap = self.get_buffer()  # Dict[str, Deque[(ts, value)]]

        aligned = sync_by_trigger_time(buffers=snap)

        if aligned is None:
            return self.bus.make_result(
                success=False,
                result={},
                message="state sync failed: no aligned frame",
            )

        aligned_dict = {k: v for k, ts, v in aligned}

        composed: Dict[str, Any] = {}
        if self.compose_cfg:
            composed = BaseServer.run_compose_with_cfg(
                aligned,
                self.compose_cfg,
            )
            aligned_dict.update(composed)

        def to_jsonable(v: Any) -> Any:
            try:
                import numpy as np

                if isinstance(v, np.ndarray):
                    return v.tolist()
            except Exception:
                pass
            if isinstance(v, array):
                return list(v)
            return v

        aligned_dict = {k: to_jsonable(v) for k, v in aligned_dict.items()}

        return self.bus.make_result(
            success=True,
            result=aligned_dict,
            message="OK",
        )

    def _make_result(
        self,
        success: bool,
        frames_sent: int = 0,
        expect_frames: int = 0,
        message: str | None = None,
    ) -> Dict[str, Any]:
        """Construct the standard return structure for run_action_chunk."""
        result_payload = {
            "frames_sent": frames_sent,
            "expect_frames": expect_frames,
        }
        return self.bus.make_result(
            success=success,
            result=result_payload,
            message=message,
        )

    def run_action_chunk(
        self,
        action_chunk: Dict[str, Any],
        fps: int = 30,
        post_delay_s: float | None = None,
    ) -> Dict[str, Any]:
        """
        Publish a sequence of actions (an action chunk) at a given frame rate.

        Args:
            action_chunk: Dict with a single key:
                {
                    "action": [frame0, frame1, ..., frameN-1]
                }
                - Only one key is allowed and it must be "action".
                - Each frame_i is a single action frame (e.g., joint + gripper control).
            fps: Target publishing rate in frames per second.

        Adapter convention:
            - GenericStepOutputAdapter:
                * build_frames returns N frames
                * Each frame is (protocol, params, msg, interval, step)
                  where interval = 1/fps and step = 1
            - GenericChunkOutputAdapter:
                * build_frames returns 1 frame
                * Frame is (protocol, params, msg, interval, step)
                  where interval = N/fps and step = N

        Returns:
            {
                "success": bool,
                "frames_sent": int,
                "expect_frames": int,
                "message": str | None,
            }
        """
        # Strict validation of action_chunk
        if not isinstance(action_chunk, dict):
            msg = "[ActionServer] action_chunk must be a dict"
            logger.error(msg)
            return self._make_result(False, message=msg)

        keys = list(action_chunk.keys())
        if len(keys) != 1 or keys[0] != "action":
            msg = f"[ActionServer] action_chunk can and must contain only one key='action', current keys={keys}"
            logger.error(msg)
            return self._make_result(False, message=msg)

        if not isinstance(action_chunk["action"], (list, tuple)):
            msg = "[ActionServer] action_chunk['action'] must be a list/tuple of frames"
            logger.error(msg)
            return self._make_result(False, message=msg)

        outputs = self.server_config.get("outputs", [])
        if not outputs:
            msg = "[ActionServer] outputs configuration is empty"
            logger.error(msg)
            return self._make_result(False, message=msg)

        # Logical frame count: number of frames in action
        logical_frames = len(action_chunk["action"])

        adapter_name = outputs[0].get("adapter", "GenericStepOutputAdapter")
        Adapter = ADAPTER_REGISTRY.get(adapter_name)
        if Adapter is None:
            msg = f"[ActionServer] Unknown adapter: {adapter_name}"
            logger.error(msg)
            return self._make_result(False, message=msg)

        adapter = Adapter()
        frames_msgs = adapter.build_frames(action_chunk, outputs, fps)
        # frames_msgs: List[List[(protocol, topic, msg, interval, step)]]

        if not frames_msgs:
            msg = "[ActionServer] adapter did not generate any messages"
            logger.error(msg)
            return self._make_result(False, message=msg)

        errors = ""
        frames_sent = 0

        start_time = time.time()

        for frame_idx, triplets in enumerate(frames_msgs):
            try:
                protocol, params, msg, interval, step = triplets[0]

                self.protocol_factory.pub(protocol, params, msg)
                logger.info(f"Sent frame {frame_idx}: {msg}")

                frames_sent += step

                if interval > 0:
                    sleep_duration = (
                        start_time + (frame_idx + 1) * interval - time.time()
                    )
                    if sleep_duration > 0:
                        time.sleep(sleep_duration)

            except Exception as e:
                err = (
                    f"[ActionServer] Exception while publishing frame {frame_idx}: {e}"
                )
                logger.error(err)
                errors = err

        end_time = time.time()
        logger.info(
            f"[ActionServer] Publishing completed, time elapsed: {end_time - start_time}s"
        )

        outputs_cfg = self.server_config.get("outputs", []) or []
        out0_params = (outputs_cfg[0].get("params", {}) if outputs_cfg else {}) or {}
        cfg_delay = float(out0_params.get("post_chunk_delay_s", 0.0))

        delay = cfg_delay if post_delay_s is None else float(post_delay_s)

        if delay > 0:
            time.sleep(delay)

        success = (frames_sent == logical_frames) and errors == ""

        logger.info(
            f"[ActionServer] Execution result: {success}, "
            f"frames_sent: {frames_sent}, expect_frames: {logical_frames}, errors: {errors}"
        )

        return self._make_result(
            success=success,
            frames_sent=frames_sent,
            expect_frames=logical_frames,
            message="OK" if success else errors,
        )

    def bind_bus(self, bus: RcpBus):
        """Register get_state and run_action_chunk tools on the bus."""
        super().bind_bus(bus)
        bus.add_tool(
            "get_state",
            self.get_state,
            input_schema=None,
            output_schema={
                "success": "bool",
                "message": "str",
                "result": {
                    "<buffer_key>": "Any  # aligned value from buffer (dict/list/number/...)",
                },
            },
            description="Get an aligned observation snapshot from internal buffer.",
        )
        bus.add_tool(
            "run_action_chunk",
            self.run_action_chunk,
            input_schema={
                "action_chunk": {
                    "action": [
                        "[j0, j1, j2, j3, j4, j5]  # one frame: joint positions (list[float])",
                        "[j0, j1, j2, j3, j4, j5]",
                    ]
                },
                "fps": "int  # Publishing frame rate (frames per second), e.g. 30",
                "post_delay_s": "float | None  # Optional delay after chunk; e.g. 0.0",
            },
            output_schema={
                "success": "bool",
                "message": "str | None",
                "result": {
                    "frames_sent": "int  # frames actually sent",
                    "expect_frames": "int  # expected frames (len(action_chunk['action']))",
                },
            },
            description="Publish an action chunk at a given FPS using the configured output adapter.",
        )
