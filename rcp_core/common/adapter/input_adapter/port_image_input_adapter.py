# rcp_core/common/adapter/port_image_input_adapter.py

"""
Image input adapter for port-based camera sources.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.adapter.port_image_input_adapter.PortImageInputAdapter`,
an input adapter that normalizes frames read from a camera "port" into the standard
image dict representation used across the project.

The adapter expects the input message to be a 5-tuple:

    ``(ok, width, height, encoding, frame)``

It timestamps frames using the current wall-clock time and stores the parsed image
under ``params.out_key``. Both uncompressed (e.g. ``rgb8``/``bgr8``) and compressed
(e.g. ``jpeg``/``png``) encodings are supported.
"""

import time
import numpy as np
from typing import Any, Dict, Tuple
from .base_input_adapter import BaseInputAdapter


class PortImageInputAdapter(BaseInputAdapter):
    """
    Port image input adapter:
    - Input is the return value of USBCamera.read() (ok, width, height, encoding, data)
    - Output structure is consistent with other input adapters:
        out[out_key] = {
          "type": "image",
          "frame_id": <str | None>,
          "height": int,
          "width": int,
          "encoding": <str>,  # for uncompressed images
          "is_bigendian": 0,
          "step": int,
          "data": <bytes>,  # uncompressed pixel buffer
        }
        OR
        {
          "type": "compressed",
          "frame_id": <str | None>,
          "format": <str>,  # jpeg/png/tiff
          "data": <bytes>,  # compressed image buffer
        }
    """

    def parse(self, msg: Any) -> Tuple[float, Dict[str, Any]]:
        """Return (ts, out_dict) where fields are extracted from the input message."""

        # Validate the input message structure
        if not isinstance(msg, (tuple, list)) or len(msg) != 5:
            raise TypeError(
                f"[PortImageInputAdapter] Expected msg to be (ok, width, height, encoding, data), got: {type(msg)}"
            )

        ok, width, height, encoding, frame = msg
        if not ok or frame is None:
            return -1, {}  # Return invalid state

        ts = time.time()

        # Prepare the image output dictionary
        img_dict: Dict[str, Any] = {
            "frame_id": None,  # Optionally set if frame_id is defined in your context
            "height": height,
            "width": width,
            "is_bigendian": 0,  # Assuming by default for simplicity
        }

        # Determine if the image is compressed or not
        if encoding in ("rgb8", "bgr8"):
            img_dict["type"] = "image"
            img_dict["encoding"] = encoding
            img_dict["data"] = frame.tobytes()
            img_dict["step"] = len(img_dict["data"]) // max(
                height, 1
            )  # Avoid division by zero
        elif encoding in ("jpeg", "jpg", "png"):
            img_dict["type"] = "compressed"
            img_dict["format"] = encoding

            if not isinstance(frame, (bytes, bytearray)):
                raise TypeError(
                    "[PortImageInputAdapter] Frame must be bytes for compressed image."
                )
            img_dict["data"] = bytes(frame)  # Ensure frame is in bytes
        else:
            raise ValueError(
                f"[PortImageInputAdapter] Unsupported encoding: {encoding}"
            )

        return ts, {self.adapter_cfg["params"]["out_key"]: img_dict}
