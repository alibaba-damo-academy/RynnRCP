# rcp_sensor/camera/camera_base.py

"""
Abstract camera driver interface.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_sensor.camera.camera_base.BaseCamera`, an abstract base
class that standardizes how camera drivers in ``rcp_sensor`` are started, read, and
stopped.

Purpose
-------
- Provides a transport-agnostic API (USB, network, etc.).
- Encapsulates only camera capture concerns (no middleware/buffering).

Constructor parameters
----------------------
- ``device_id``: device identifier (index, string ID, or device path)
- ``width`` / ``height``: expected capture resolution
- ``encoding``: expected output encoding:
  - uncompressed: ``"rgb8"``, ``"bgr8"``
  - compressed: ``"jpeg"``, ``"png"``
- ``fps``: desired capture rate
- ``brand``: informational camera brand/model label
- ``rotate``: rotation in degrees (0/90/180/270)

Required methods
----------------
Concrete implementations must implement:
- :meth:`start` — initialize and begin capture
- :meth:`read` — return ``(success, width, height, encoding, image)``, where ``image`` is
  a NumPy array (or ``None`` on failure)
- :meth:`stop` — release resources and stop capture
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Tuple, Optional

import numpy as np


class BaseCamera(ABC):
    """
    Abstract camera base class.

    - Does not care whether it's USB, network, or other types; only defines a unified interface.
    - Responsible only for "capturing images + encoding."
    """

    def __init__(
        self,
        device_id: str | int,
        width: int,
        height: int,
        encoding: str,
        fps: float,
        brand: str | None = None,
        rotate: int = 0,
    ):
        """
        :param device_id: Camera device ID, such as 0 / "0" / "/dev/video0"
        :param width:  Image width
        :param height: Image height
        :param encoding: Image encoding format:
                         - Uncompressed: "rgb8", "bgr8"
                         - Compressed:   "jpeg", "png"
        :param fps: Frame rate
        :param brand: Camera brand/model (for informational display only)
        :param rotate: Image rotation degrees (0/90/180/270). Default 0 (no rotation).
        """
        self.device_id = device_id
        self.width = width
        self.height = height
        self.encoding = encoding
        self.fps = fps
        self.brand = brand or "Unknown"
        self.rotate = rotate

    @abstractmethod
    def start(self) -> None:
        """Start the camera."""
        raise NotImplementedError("Abstract method not implemented")

    @abstractmethod
    def read(self) -> Tuple[bool, int, int, str, Optional[np.ndarray]]:
        """Read an image from the camera.

        :return: (success, width, height, encoding, image)
        """
        raise NotImplementedError("Abstract method not implemented")

    @abstractmethod
    def stop(self) -> None:
        """Stop the camera."""
        raise NotImplementedError("Abstract method not implemented")
