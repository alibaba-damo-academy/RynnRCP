# rcp_core/common/utils/image_backend_cv2.py

"""
OpenCV-based image backend.
~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.utils.image_backend_cv2.ImageBackendCv2`,
an :class:`~rcp_core.common.utils.image_backend.ImageBackendBase` implementation that
performs common image conversions using OpenCV (cv2) and NumPy.

Provided operations
-------------------
- :meth:`raw_image_to_ndarray(h, w, encoding, data)`:
  Converts raw pixel bytes into a ``(h, w, 3)`` ``np.uint8`` ndarray.
  Supports ``bgr8`` and ``rgb8`` (RGB is converted to BGR for OpenCV consistency) and
  validates that ``len(data) == h*w*3``.

- :meth:`compressed_to_ndarray(data, fmt)`:
  Decodes compressed image bytes (e.g. JPEG/PNG) into a BGR ndarray using
  ``cv2.imdecode``. Raises if decoding fails.

- :meth:`encode_image(img, img_type)`:
  Encodes a BGR ndarray into compressed bytes (e.g. ``jpg``/``png``) via
  ``cv2.imencode`` and returns the resulting byte buffer.

- :meth:`resize(img, width, height)`:
  Resizes using ``cv2.resize`` and chooses interpolation automatically:
  ``INTER_AREA`` for downscaling, ``INTER_LINEAR`` for upscaling.
"""

from __future__ import annotations

import cv2
import numpy as np
from .image_backend import ImageBackendBase


class ImageBackendCv2(ImageBackendBase):
    """Image backend implementation based on OpenCV."""

    def raw_image_to_ndarray(
        self, h: int, w: int, encoding: str, data: bytes
    ) -> np.ndarray:
        """Convert raw image bytes (bgr8/rgb8) to an ndarray."""
        encoding = encoding.lower()

        if encoding in ("bgr8", "rgb8"):
            arr = np.frombuffer(data, dtype=np.uint8)
            if arr.size != h * w * 3:
                raise ValueError(
                    f"[ImageBackendCv2] Data length mismatch: {arr.size} != {h}*{w}*3 for encoding={encoding}"
                )
            cv_img = arr.reshape((h, w, 3))
            if encoding == "rgb8":
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
            return cv_img

        raise ValueError(f"[ImageBackendCv2] Unsupported image encoding: {encoding}")

    def compressed_to_ndarray(self, data: bytes, fmt: str) -> np.ndarray:
        """Decode compressed image bytes (e.g. JPEG/PNG) to a BGR ndarray."""
        np_arr = np.frombuffer(data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_img is None:
            raise RuntimeError(
                "[ImageBackendCv2] cv2.imdecode failed, unable to decode from compressed data"
            )
        return cv_img

    def encode_image(self, img: np.ndarray, img_type: str) -> bytes:
        """Encode a BGR ndarray to compressed image bytes (e.g. jpg/png)."""
        ok, buf = cv2.imencode(f".{img_type}", img)
        if not ok:
            raise RuntimeError(
                f"[ImageBackendCv2] cv2.imencode('.{img_type}', ...) failed"
            )
        return buf.tobytes()

    def resize(self, img: np.ndarray, width: int, height: int) -> np.ndarray:
        """Resize an image ndarray with OpenCV using suitable interpolation."""
        h, w = img.shape[:2]
        interpolation = cv2.INTER_AREA if width < w or height < h else cv2.INTER_LINEAR
        return cv2.resize(img, (width, height), interpolation=interpolation)
