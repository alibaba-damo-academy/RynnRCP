# rcp_core/common/adapter/lcm_image_input_adapter.py

"""
LCM image input adapter.
~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.adapter.lcm_image_input_adapter.LcmImageInputAdapter`,
an input adapter that normalizes LCM image messages into a single structured dict
suitable for buffering and downstream consumers.

Supported message types:
- ``lcm_msgs.Image`` (uncompressed): extracts geometry/encoding metadata and the raw
  pixel byte buffer.
- ``lcm_msgs.CompressedImage``: extracts the compression format and the compressed
  byte buffer.

The adapter uses ``msg.header.stamp_sec`` / ``msg.header.stamp_nanosec`` as the
timestamp and writes the parsed image dict under a configured ``out_key`` (typically
via ``params.mappings[0].out_key``).
"""

from typing import Any, Dict, Tuple

from .base_input_adapter import BaseInputAdapter


class LcmImageInputAdapter(BaseInputAdapter):
    """
    LCM image input adapter:
      - Supports two message types:
          * lcm_msgs.Image           (uncompressed image)
          * lcm_msgs.CompressedImage (compressed image)
      - Extracts timestamp ts (float seconds) from msg.header.stamp_sec / stamp_nanosec.
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
            "format": <str>,  # jpeg/png/tiff
            "data": <bytes>,  # compressed image buffer
          }
    """

    @staticmethod
    def _extract_timestamp_from_header(msg: Any) -> float:
        """Extract timestamp in seconds from msg.header.stamp_sec / stamp_nanosec."""
        try:
            header = getattr(msg, "header")
            sec = getattr(header, "stamp_sec")
            nanosec = getattr(header, "stamp_nanosec")
        except AttributeError as e:
            raise ValueError(
                f"[LcmImageInputAdapter] Missing header.stamp_sec / stamp_nanosec in message: {e}"
            )
        return float(sec) + float(nanosec) * 1e-9

    def parse(self, msg: Any) -> Tuple[float, Dict[str, Any]]:
        """Return (ts, out_dict) where fields are extracted per mappings."""
        ts = self._extract_timestamp_from_header(msg)
        params = self.adapter_cfg.get("params", self.adapter_cfg)
        mappings = params.get("mappings") or []
        out_key = None
        if mappings:
            first = mappings[0]
            out_key = first.get("out_key")

        if not out_key:
            raise ValueError(
                "[LcmImageInputAdapter] adapter_cfg/params must provide out_key or mappings[0].out_key"
            )

        frame_id = getattr(getattr(msg, "header", None), "frame_id", None)
        typename = type(msg).__name__

        img_dict: Dict[str, Any] = {
            "frame_id": frame_id,
        }

        if typename == "Image":
            # lcm_msgs.Image (uncompressed)
            try:
                height = int(getattr(msg, "height"))
                width = int(getattr(msg, "width"))
                encoding = str(getattr(msg, "encoding"))
                is_bigendian = int(getattr(msg, "is_bigendian"))
                step = int(getattr(msg, "step"))
                data_seq = getattr(msg, "data")
            except Exception as e:
                raise ValueError(
                    f"[LcmImageInputAdapter] Failed to parse Image fields: {e}"
                )

            img_dict.update(
                {
                    "type": "image",
                    "height": height,
                    "width": width,
                    "encoding": encoding,
                    "is_bigendian": is_bigendian,
                    "step": step,
                    "data": bytes(data_seq),  # byte[] -> bytes
                }
            )

        elif typename == "CompressedImage":
            # lcm_msgs.CompressedImage (compressed)
            try:
                fmt = str(getattr(msg, "format"))
                data_seq = getattr(msg, "data")
            except Exception as e:
                raise ValueError(
                    f"[LcmImageInputAdapter] Parse CompressedImage failed: {e}"
                )

            img_dict.update(
                {
                    "type": "compressed",
                    "format": fmt,
                    "data": bytes(data_seq),  # byte[] -> bytes
                }
            )

        else:
            raise TypeError(
                f"[LcmImageInputAdapter] Unsupported message type: {type(msg)} "
                "(only supports lcm_msgs.Image and lcm_msgs.CompressedImage)"
            )

        out: Dict[str, Any] = {out_key: img_dict}
        return ts, out
