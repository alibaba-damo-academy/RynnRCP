"""
Protocol image input adapter.

Normalizes image data from connector-specific sources into the RCP image value:
``{width, height, encoding, image}``.

Supports:
- Camera 5-tuple: ``(ok, width, height, encoding, raw_data)``
- Dict with image fields
- ROS2-style Image/CompressedImage messages
"""

from __future__ import annotations

import time
from typing import Any, Dict, Tuple

from .base_input_adapter import BaseInputAdapter


class ProtocolImageInputAdapter(BaseInputAdapter):
    """Adapter that normalizes image data into an RCP image value.

    Configuration (via *params*):
      object_name : str – key for the image dict in the output (default ``image``)
    """

    def __init__(self, params: Dict[str, Any] | None = None) -> None:
        if params is None:
            raise ValueError("ProtocolImageInputAdapter requires params")
        self._object_name = str(params["object_name"])
        self._width: int | None = None
        self._height: int | None = None
        self._encoding: str | None = None
        self._width = params.get("width")
        self._height = params.get("height")
        self._encoding = params.get("encoding")

    def parse(self, msg: Any) -> Tuple[float, Dict[str, Any]]:
        ts = time.time()

        # 5-tuple from BaseCamera.read_frame(): (ok, width, height, encoding, data)
        if isinstance(msg, (tuple, list)) and len(msg) == 5:
            ok, width, height, encoding, data = msg
            if not ok:
                return ts, {}

            raw = data

            return ts, {
                self._object_name: {
                    "width": int(width),
                    "height": int(height),
                    "encoding": str(encoding),
                    "image": raw,
                }
            }

        # Dict with image fields
        if isinstance(msg, dict):
            if "image" in msg:
                return ts, {self._object_name: msg}

        # ROS2 CompressedImage message (has header, format, data; no width/height).
        if hasattr(msg, "format") and hasattr(msg, "data"):
            ts = self._timestamp_from_header(msg, default=ts)
            width, height, encoding = self._compressed_meta(msg.data, str(getattr(msg, "format", "")))
            return ts, {
                self._object_name: {
                    "width": int(width),
                    "height": int(height),
                    "encoding": encoding,
                    "image": msg.data,
                }
            }

        # ROS2 Image message (has header, height, width, encoding, data)
        if hasattr(msg, "height") and hasattr(msg, "width") and hasattr(msg, "data"):
            ts = self._timestamp_from_header(msg, default=ts)

            return ts, {
                self._object_name: {
                    "width": int(msg.width),
                    "height": int(msg.height),
                    "encoding": str(msg.encoding),
                    "image": msg.data,
                }
            }

        return ts, {}

    def _compressed_meta(self, data: Any, msg_format: str) -> tuple[int, int, str]:
        if self._width is None or self._height is None or self._encoding is None:
            raise ValueError("Compressed image inputs require width, height, and encoding in config")
        return int(self._width), int(self._height), str(self._encoding)

    @staticmethod
    def _timestamp_from_header(msg: Any, default: float) -> float:
        try:
            header = getattr(msg, "header", None)
            if header:
                stamp = getattr(header, "stamp", None)
                if stamp:
                    return float(stamp.sec) + float(stamp.nanosec) * 1e-9
                sec = getattr(header, "stamp_sec", None)
                nsec = getattr(header, "stamp_nanosec", None)
                if sec is not None:
                    return float(sec) + float(nsec or 0) * 1e-9
        except Exception:
            pass
        return default
