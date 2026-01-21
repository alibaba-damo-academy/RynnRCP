# rcp_core/common/adapter/output_adapter/ros2_joint_trajectory_output_adapter.py

"""
ROS 2 JointTrajectory chunk-mode output adapter.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.adapter.output_adapter.ros2_joint_trajectory_output_adapter.Ros2JointTrajectoryOutputAdapter`,
a chunk-mode adapter that converts an action chunk into a single
``trajectory_msgs.msg.JointTrajectory`` message.

Behavior:
- reads ``action_chunk[key]`` (typically ``key="action"``) as a sequence of N joint frames
- creates N ``JointTrajectoryPoint`` messages
- writes each frame’s joint vector to the configured ``out_field`` (typically ``positions``)
- sets ``point.time_from_start`` to ``(i+1)/fps`` seconds (when available)
- packs points into ``traj_msg.points``

The adapter emits exactly one publish frame with:
- ``interval = N / fps``
- ``step = N``
where ``N`` is the number of frames in the chunk.
"""

from typing import Any, Dict, List, Tuple
import numpy as np

from .base_output_adapter import BaseOutputAdapter
from ...utils.get_message_class import get_message_class
from rcp_core.common.utils.logger import server_logger

logger = server_logger()


class Ros2JointTrajectoryOutputAdapter(BaseOutputAdapter):
    """
    Chunk adapter dedicated to ROS2 trajectory_msgs.msg.JointTrajectory.

    Design:
      - action_chunk[key] is N frames, each frame is a joint positions vector.
      - Outputs a single JointTrajectory message:
          points: N JointTrajectoryPoint
              points[i].<out_field>     = joint values for frame i (usually positions)
              points[i].time_from_start = (i+1) / fps
    """

    def build_frames(
        self,
        action_chunk: Dict[str, Any],
        outputs: List[Dict[str, Any]],
        fps: int,
    ) -> List[List[Tuple[str, str, Any, float, int]]]:
        """Build a JointTrajectory chunk message from action_chunk and outputs."""

        if not outputs:
            logger.warning("[Ros2JointTrajectoryOutputAdapter] outputs is empty")
            return []

        out = outputs[0]
        protocol = out.get("protocol")
        topic = out.get("topic")
        type_str = out.get("type")  # "sensor_msgs.msg.JointState"
        mappings = out.get("mappings", [])

        if not protocol or not topic or not type_str:
            logger.error(f"[Ros2JointStateOutputAdapter] invalid output config: {out}")
            return []

        if not mappings:
            logger.warning("[Ros2JointStateOutputAdapter] mappings is empty")
            return []

        mapping = mappings[0]
        key = mapping.get("key")  # "action"
        out_field = mapping.get("out_field")  # "positions"

        if key not in action_chunk:
            logger.warning(f"[Ros2JointStateOutputAdapter] '{key}' not in action_chunk")
            return []

        frames = action_chunk[key]
        if not isinstance(frames, (list, tuple)) or not frames:
            logger.warning(
                f"[Ros2JointStateOutputAdapter] action_chunk['{key}'] is empty or not list/tuple"
            )
            return []

        # JointTrajectory and JointTrajectoryPoint types
        Msg = get_message_class(type_str)
        point_type_str = type_str.rsplit(".", 1)[0] + ".JointTrajectoryPoint"
        PointMsg = get_message_class(point_type_str)

        traj_msg = Msg()

        points: List[Any] = []

        for i, frame_data in enumerate(frames):

            # Convert to list[float]
            if isinstance(frame_data, np.ndarray):
                frame_list = frame_data.astype(float).tolist()
            elif isinstance(frame_data, (list, tuple)):
                frame_list = [float(v) for v in frame_data]
            else:
                logger.error(
                    "[Ros2JointTrajectoryOutputAdapter] unsupported frame_data type:",
                    type(frame_data),
                )
                continue

            p = PointMsg()

            # Set the out_field on the point
            if out_field and hasattr(p, out_field):
                setattr(p, out_field, frame_list)
            else:
                logger.error(
                    f"[Ros2JointTrajectoryOutputAdapter] invalid out_field '{out_field}' "
                    f"for PointMsg type {PointMsg}"
                )
                continue

            # time_from_start: (i+1)/fps seconds
            if fps > 0 and hasattr(p, "time_from_start"):
                total_sec = float(i + 1) / float(fps)
                sec = int(total_sec)
                nsec = int((total_sec - sec) * 1e9)
                try:
                    p.time_from_start.sec = sec
                    p.time_from_start.nanosec = nsec
                except Exception as e:
                    logger.error(
                        "[Ros2JointTrajectoryOutputAdapter] set time_from_start failed:",
                        e,
                    )

            points.append(p)

        traj_msg.points = points

        num_frames = len(frames)
        interval_chunk = float(num_frames) / fps if fps > 0 else 0.0
        step_chunk = num_frames

        return [[(protocol, topic, traj_msg, interval_chunk, step_chunk)]]
