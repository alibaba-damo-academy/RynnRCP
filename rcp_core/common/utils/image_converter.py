# rcp_core/common/utils/image_converter.py

"""
Structured image dict → encoded bytes converter.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.utils.image_converter.ImageConverter`, a
small utility that converts *structured* image dictionaries (already parsed from
ROS2/LCM/module/port inputs) into compressed image bytes (JPEG/PNG/etc.), optionally
resizing along the way. The actual decode/encode/resize work is delegated to a
pluggable :class:`~rcp_core.common.utils.image_backend.ImageBackendBase` backend
(OpenCV/cv2 by default).

Input format
------------
``msg_to_img`` expects a dict of the form:

- Raw image:
  ``{"type": "image", "data": bytes, "height": int, "width": int, "encoding": str}``
  where encoding is typically ``bgr8`` or ``rgb8``.

- Compressed image:
  ``{"type": "compressed", "data": bytes, "format": str}``
  where format is something like ``jpeg``/``png``.

Behavior
--------
- Normalizes ``img_type="jpg"`` to ``"jpeg"``.
- If ``type == "image"``:
  raw bytes → ndarray → optional resize → encode to requested format.
- If ``type == "compressed"``:
  - passthrough if no resize is requested and input format already matches output format
  - otherwise decode → optional resize → re-encode to requested format.
- Logs useful size/format/shape information for debugging.

Public API
----------
- :meth:`ImageConverter.msg_to_img(img_msg, img_type="jpeg", width=None, height=None)`:
  main entry point.
"""
from __future__ import annotations

from .image_backend import ImageBackendBase
from .image_backend_cv2 import ImageBackendCv2
from rcp_core.common.utils.logger import server_logger

logger = server_logger()


class ImageConverter:
    """
    Generic image converter with pluggable backend.

    Input is a structured dict (not raw ROS2/LCM messages), and the backend
    (OpenCV by default) handles actual decode/encode/resize.
    """

    def __init__(self, backend: ImageBackendBase | None = None):
        """Initialize the converter with a given image backend (default: OpenCV)."""
        self.backend = backend or ImageBackendCv2()

    def msg_to_img(
        self,
        img_msg: dict,
        img_type: str = "jpeg",
        width: int | None = None,
        height: int | None = None,
    ) -> bytes:
        """
        Convert a structured image dict to encoded bytes (optionally resized).

        img_msg: {
          "type": "image" | "compressed",
          "data": bytes,
          # if type == "image":
          "height": int,
          "width": int,
          "encoding": str,
          # if type == "compressed":
          "format": str,
        }
        """
        if not isinstance(img_msg, dict):
            raise TypeError(
                f"[ImageConverter] Expected img_msg to be dict, actual type={type(img_msg)}"
            )

        logger.info(
            f"[ImageConverter] in: type={img_type}, "
            f"encoding={img_msg.get('encoding', '')}, "
            f"format={img_msg.get('format', '')}, "
            f"size={len(img_msg['data'])} bytes"
        )

        img_type = "jpeg" if img_type == "jpg" else img_type

        img_kind = img_msg.get("type")
        if img_kind == "image":
            return self._dict_image_to_bytes(img_msg, img_type, width, height)
        elif img_kind == "compressed":
            return self._dict_compressed_to_bytes(img_msg, img_type, width, height)
        else:
            raise ValueError(f"[ImageConverter] Unknown image type: {img_kind}")

    def _dict_image_to_bytes(
        self,
        img_msg: dict,
        img_type: str,
        width: int | None,
        height: int | None,
    ) -> bytes:
        """Handle type='image': raw fields -> ndarray -> (optional resize) -> bytes."""
        h = int(img_msg["height"])
        w = int(img_msg["width"])
        encoding = str(img_msg["encoding"]).lower()
        data = img_msg["data"]

        img = self.backend.raw_image_to_ndarray(h, w, encoding, data)

        if width is not None and height is not None:
            img = self.backend.resize(img, width, height)

        out_bytes = self.backend.encode_image(img, img_type)

        out_h, out_w = img.shape[:2]
        logger.info(
            f"[DictImage] in: {w}x{h}, encoding={encoding} -> "
            f"out: {out_w}x{out_h}, format={img_type}, size={len(out_bytes)} bytes"
        )
        return out_bytes

    def _dict_compressed_to_bytes(
        self,
        img_msg: dict,
        img_type: str,
        width: int | None,
        height: int | None,
    ) -> bytes:
        """Handle type='compressed': decode -> (optional resize) -> re-encode."""
        fmt = str(img_msg.get("format", "")).lower()
        data = img_msg["data"]

        # If already in target format and no resize is needed, passthrough
        if (width is None or height is None) and fmt == img_type:
            logger.info(
                f"[DictCompressed] passthrough: format={fmt}, size={len(data)} bytes (no resize)"
            )
            return data

        img = self.backend.compressed_to_ndarray(data, fmt)

        if width is not None and height is not None:
            img = self.backend.resize(img, width, height)

        out_bytes = self.backend.encode_image(img, img_type)

        h, w = img.shape[:2]
        logger.info(
            f"[DictCompressed] in: format={fmt}, decoded={w}x{h} -> "
            f"out: format={img_type}, size={len(out_bytes)} bytes"
        )
        return out_bytes
