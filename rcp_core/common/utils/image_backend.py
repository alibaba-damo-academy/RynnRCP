# rcp_core/common/utils/image_backend.py

"""
Abstract image backend interface.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.utils.image_backend.ImageBackendBase`, an
abstract base class that standardizes image conversions used across rcp_core.

Concrete backends (e.g. OpenCV/cv2 or Pillow/PIL) implement four operations:
- :meth:`raw_image_to_ndarray`:
  convert raw pixel buffers (with known height/width/encoding) into a NumPy ndarray
- :meth:`compressed_to_ndarray`:
  decode compressed image bytes (JPEG/PNG, etc.) into a NumPy ndarray
- :meth:`encode_image`:
  encode an ndarray into compressed image bytes in a requested format
- :meth:`resize`:
  resize an image ndarray to a target width/height

The goal is to allow the rest of the system to perform image handling without being
tied to a specific imaging library.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
import numpy as np


class ImageBackendBase(ABC):
    """
    Abstract interface for image backends:
      - raw bytes <-> ndarray
      - compressed bytes <-> ndarray
      - resize
    """

    @abstractmethod
    def raw_image_to_ndarray(
        self, h: int, w: int, encoding: str, data: bytes
    ) -> np.ndarray:
        """Convert raw image fields (size + encoding + buffer) to an ndarray."""
        ...

    @abstractmethod
    def compressed_to_ndarray(self, data: bytes, fmt: str) -> np.ndarray:
        """Decode compressed image bytes (e.g. JPEG/PNG) to an ndarray."""
        ...

    @abstractmethod
    def encode_image(self, img: np.ndarray, img_type: str) -> bytes:
        """Encode an image ndarray to compressed bytes in the given format."""
        ...

    @abstractmethod
    def resize(self, img: np.ndarray, width: int, height: int) -> np.ndarray:
        """Resize an image ndarray to the specified width and height."""
        ...
