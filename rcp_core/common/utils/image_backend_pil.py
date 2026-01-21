# rcp_core/common/utils/image_backend_pil.py

"""
PIL-based image backend.
~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.utils.image_backend_pil.ImageBackendPIL`,
an :class:`~rcp_core.common.utils.image_backend.ImageBackendBase` implementation using
Pillow (PIL) + NumPy.

Color conventions
-----------------
This backend uses **RGB ndarrays** as its in-memory convention.

Provided operations
-------------------
- :meth:`raw_image_to_ndarray(h, w, encoding, data)`:
  Converts raw bytes into an ``(h, w, 3)`` ``np.uint8`` ndarray.
  Supports ``rgb8`` and ``bgr8``; for ``bgr8`` it swaps channels (BGR→RGB).
  Validates ``len(data) == h*w*3``.

- :meth:`compressed_to_ndarray(data, fmt)`:
  Decodes compressed bytes (e.g. JPEG/PNG) to an RGB ndarray using
  ``Image.open(...).convert("RGB")``.

- :meth:`encode_image(img, img_type)`:
  Encodes an RGB ndarray to compressed bytes via Pillow.

- :meth:`resize(img, width, height)`:
  Resizes using Pillow bilinear resampling and returns an RGB ndarray.
"""

from __future__ import annotations

import io
import numpy as np
from PIL import Image

from .image_backend import ImageBackendBase


class ImageBackendPIL(ImageBackendBase):
    """Image backend implementation based on PIL."""

    def raw_image_to_ndarray(self, h, w, encoding, data):
        """Convert raw image bytes (rgb8/bgr8) to an RGB ndarray."""
        encoding = encoding.lower()
        arr = np.frombuffer(data, dtype=np.uint8)

        if encoding in ("rgb8", "bgr8"):
            if arr.size != h * w * 3:
                raise ValueError(
                    f"[ImageBackendPIL] size mismatch: {arr.size} != {h}*{w}*3 for {encoding}"
                )
            img = arr.reshape((h, w, 3))
            # PIL uses RGB; bgr8 needs conversion
            if encoding == "bgr8":
                img = img[:, :, ::-1]  # BGR -> RGB
            return img

        raise ValueError(f"[ImageBackendPIL] unsupported encoding: {encoding}")

    def compressed_to_ndarray(self, data: bytes, fmt: str) -> np.ndarray:
        """Decode compressed image bytes to an RGB ndarray with PIL."""
        img = Image.open(io.BytesIO(data)).convert("RGB")
        return np.array(img)

    def encode_image(self, img: np.ndarray, img_type: str) -> bytes:
        """Encode an RGB ndarray to compressed image bytes (e.g. jpg/png)."""
        pil_img = Image.fromarray(img)
        buf = io.BytesIO()
        pil_img.save(buf, format=img_type.upper())
        return buf.getvalue()

    def resize(self, img: np.ndarray, width: int, height: int) -> np.ndarray:
        """Resize an RGB image ndarray using PIL."""
        pil_img = Image.fromarray(img)
        pil_img = pil_img.resize((width, height), Image.BILINEAR)
        return np.array(pil_img)
