# rcp_core/common/adapter/ros2_image_input_adapter.py

"""
ROS 2 image input adapter.
~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.adapter.ros2_image_input_adapter.Ros2ImageInputAdapter`,
an input adapter that normalizes ROS 2 image messages into a single structured dict
suitable for buffering and downstream consumers.

Supported message types:
- ``sensor_msgs.msg.Image`` (uncompressed): extracts geometry/encoding metadata and
  the raw pixel byte buffer.
- ``sensor_msgs.msg.CompressedImage``: extracts the compression format and the
  compressed byte buffer.

The adapter uses ``msg.header.stamp.sec`` / ``msg.header.stamp.nanosec`` as the
timestamp and writes the parsed image dict under a configured ``out_key`` (typically
via ``params.mappings[0].out_key``).
"""

from typing import Any, Dict, Tuple

from .base_input_adapter import BaseInputAdapter


class Ros2ImageInputAdapter(BaseInputAdapter):
    """
    ROS2 image input adapter:
      - Supports two message types:
          * sensor_msgs/msg/Image           (uncompressed image)
          * sensor_msgs/msg/CompressedImage (compressed image)
      - Extracts timestamp ts (float seconds) from msg.header.stamp.
      - Replaces the original msg with a structured dict under a single out_key, for example:
          out_key = "observation.images.cam0"

        For Image, the final buffer[out_key] value is:
          {
            "type": "image",
            "frame_id": <str | None>,
            "height": <int>,
            "width": <int>,
            "encoding": <str>,
            "is_bigendian": <int>,
            "step": <int>,
            "data": <bytes>,  # raw uncompressed pixel buffer
          }

        For CompressedImage:
          {
            "type": "compressed",
            "frame_id": <str | None>,
            "format": <str>,  # jpeg/png
            "data": <bytes>,  # compressed image buffer
          }
    """

    @staticmethod
    def _extract_timestamp_from_header(msg: Any) -> float:
        """Extract timestamp in seconds from ROS2 msg.header.stamp.sec / nanosec."""
        try:
            header = getattr(msg, "header")
            stamp = getattr(header, "stamp")
            sec = getattr(stamp, "sec")
            nanosec = getattr(stamp, "nanosec")
        except AttributeError as e:
            raise ValueError(
                f"[Ros2ImageInputAdapter] Missing header.stamp.sec / nanosec in message: {e}"
            )
        return float(sec) + float(nanosec) * 1e-9

    def parse(self, msg: Any) -> Tuple[float, Dict[str, Any]]:
        """Parse a ROS2 image message into (timestamp_seconds, {out_key: structured_image_dict})."""
        ts = self._extract_timestamp_from_header(msg)

        params = self.adapter_cfg.get("params", self.adapter_cfg)
        mappings = params.get("mappings") or []
        out_key = None
        if mappings:
            first = mappings[0]
            out_key = first.get("out_key")

        if not out_key:
            raise ValueError(
                "[Ros2ImageInputAdapter] adapter_cfg/params must provide out_key or mappings[0].out_key"
            )

        frame_id = getattr(getattr(msg, "header", None), "frame_id", None)
        typename = type(msg).__name__

        img_dict: Dict[str, Any] = {
            "frame_id": frame_id,
        }

        if typename == "Image":
            # sensor_msgs/msg/Image (uncompressed)
            try:
                height = int(getattr(msg, "height"))
                width = int(getattr(msg, "width"))
                encoding = str(getattr(msg, "encoding"))
                is_bigendian = int(getattr(msg, "is_bigendian"))
                step = int(getattr(msg, "step"))
                data_seq = getattr(msg, "data")
            except Exception as e:
                raise ValueError(f"[Ros2ImageInputAdapter] 解析 Image 字段失败: {e}")

            img_dict.update(
                {
                    "type": "image",
                    "height": height,
                    "width": width,
                    "encoding": encoding,
                    "is_bigendian": is_bigendian,
                    "step": step,
                    "data": bytes(data_seq),  # uint8[] -> bytes
                }
            )

        elif typename == "CompressedImage":
            # sensor_msgs/msg/CompressedImage (compressed)
            try:
                fmt = str(getattr(msg, "format"))
                data_seq = getattr(msg, "data")
            except Exception as e:
                raise ValueError(
                    f"[Ros2ImageInputAdapter] 解析 CompressedImage 字段失败: {e}"
                )

            img_dict.update(
                {
                    "type": "compressed",
                    "format": fmt,
                    "data": bytes(data_seq),  # uint8[] -> bytes
                }
            )

        else:
            raise TypeError(
                f"[Ros2ImageInputAdapter] Unsupported message type: {type(msg)} "
                "(only supports sensor_msgs.msg.Image and sensor_msgs.msg.CompressedImage)"
            )

        out: Dict[str, Any] = {out_key: img_dict}
        return ts, out
