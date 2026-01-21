# rcp_sensor/camera/usb_camera.py

"""
OpenCV USB camera driver.
~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_sensor.camera.usb_camera.USBCamera`, a concrete
implementation of :class:`~rcp_sensor.camera.base_camera.BaseCamera` using OpenCV’s
:class:`cv2.VideoCapture`.

Platform/backends
-----------------
The driver selects a capture backend based on OS:
- Linux: ``cv2.CAP_V4L2``
- macOS: ``cv2.CAP_AVFOUNDATION``
- Windows: ``cv2.CAP_DSHOW``
- otherwise: ``cv2.CAP_ANY``

Device ID handling
------------------
``device_id`` may be:
- an integer index (``0``, ``1`` …)
- a numeric string (``"0"`` → index 0)
- a device path (Linux, e.g. ``/dev/video0``)
- macOS AVFoundation uniqueID (UUID/hex-like), optionally prefixed with ``"avf:"``

On macOS, uniqueID values are mapped to a stable AVFoundation device index via PyObjC
(``pip install pyobjc``) by enumerating available ``AVCaptureDevice`` instances.

Capture configuration
---------------------
:meth:`start` opens the device and applies:
- Linux: forces MJPG FOURCC to reduce USB bandwidth
- requested frame width/height/fps via OpenCV capture properties

Frame acquisition and encoding
------------------------------
:meth:`read`:
- grabs a frame from ``VideoCapture.read()``
- optionally rotates by 0/90/180/270 degrees
- returns ``(success, width, height, encoding, payload)`` where payload depends on
  ``self.encoding``:
  - ``bgr8``: returns the BGR ndarray as captured by OpenCV
  - ``rgb8``: converts BGR→RGB and returns an RGB ndarray
  - ``jpeg/jpg/png``: encodes the frame via ``cv2.imencode`` and returns compressed bytes

Shutdown
--------
:meth:`stop` stops capture, waits briefly, and releases the underlying OpenCV capture handle.
"""

from __future__ import annotations

import platform
import re
import cv2
import numpy as np
from time import sleep
from typing import Tuple, Optional, Union

from .base_camera import BaseCamera
from rcp_core.common.utils.logger import server_logger

logger = server_logger()


def _pick_cv_backend() -> int:
    sysname = platform.system()
    if sysname == "Linux":
        return cv2.CAP_V4L2
    if sysname == "Darwin":
        return cv2.CAP_AVFOUNDATION
    if sysname == "Windows":
        return cv2.CAP_DSHOW
    return cv2.CAP_ANY


def _is_probably_macos_uniqueid(s: str) -> bool:
    """
    macOS AVFoundation uniqueID examples:
      - 0x213000010bb2b08  (hex-like)
      - 1FD4B3A2-236E-492B-8CE5-255DD288CE50 (UUID)
    """
    s = s.strip()
    if s.startswith("avf:"):
        s = s[4:]

    if re.fullmatch(r"0x[0-9a-fA-F]+", s):
        return True
    if re.fullmatch(r"[0-9A-Fa-f]{8}-([0-9A-Fa-f]{4}-){3}[0-9A-Fa-f]{12}", s):
        return True
    return False


def _macos_uniqueid_to_index(unique_id: str) -> int:
    """
    Map AVFoundation uniqueID -> index in AVFoundation device list.
    """
    try:
        import AVFoundation
    except ModuleNotFoundError as e:
        raise RuntimeError(
            "AVFoundation not available. On macOS install: pip install pyobjc"
        ) from e

    uid = unique_id.strip()
    if uid.startswith("avf:"):
        uid = uid[4:]

    devices = AVFoundation.AVCaptureDevice.devicesWithMediaType_(
        AVFoundation.AVMediaTypeVideo
    )

    for i, d in enumerate(devices):
        if str(d.uniqueID()) == uid:
            return i

    available = [
        (i, str(d.localizedName()), str(d.uniqueID())) for i, d in enumerate(devices)
    ]
    raise KeyError(f"uniqueID not found: {uid}. Available: {available}")


class USBCamera(BaseCamera):
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
        super().__init__(device_id, width, height, encoding, fps, brand, rotate)
        self._cap: Optional[cv2.VideoCapture] = None
        self._running: bool = False

    def start(self) -> None:
        """
        Open the USB camera device and set the resolution.

        Supported device_id:
          - int / "0" / "1" (index)
          - Linux: "/dev/video0" or "/dev/v4l/by-id/..."
          - macOS: AVFoundation uniqueID (e.g. "0x213..." or UUID), optionally prefixed with "avf:"
        """
        sysname = platform.system()
        backend = _pick_cv_backend()

        dev: Union[int, str]

        if isinstance(self.device_id, int):
            dev = self.device_id
        else:
            s = self.device_id.strip()

            # macOS uniqueID -> index
            if sysname == "Darwin" and _is_probably_macos_uniqueid(s):
                idx = _macos_uniqueid_to_index(s)
                logger.info(f"macOS uniqueID mapped to index: {s} -> {idx}")
                dev = idx
            else:
                # "0" -> 0, otherwise keep string path
                try:
                    dev = int(s)
                except ValueError:
                    dev = s

        cap = cv2.VideoCapture(dev, backend)
        if not cap.isOpened():
            raise RuntimeError(
                f"Unable to open camera device: {self.device_id} "
                f"(resolved={dev}, platform={sysname}, backend={backend})"
            )

        # Linux: Force MJPG to reduce USB bandwidth
        if sysname == "Linux":
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            cap.set(cv2.CAP_PROP_FOURCC, fourcc)

        if self.width is not None:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height is not None:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps is not None:
            cap.set(cv2.CAP_PROP_FPS, self.fps)

        self._cap = cap
        self._running = True

    def read(self) -> Tuple[bool, int, int, str, Optional[np.ndarray]]:
        if self._cap is None:
            raise RuntimeError("Camera not opened, please call start() first.")

        if not self._running:
            return False, 0, 0, self.encoding, None

        ok, frame = self._cap.read()
        if not ok:
            logger.warning(f"{self.device_id}: Failed to read a frame")
            return False, 0, 0, self.encoding, None

        if self.rotate == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif self.rotate == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        h, w = frame.shape[:2]

        if self.encoding == "bgr8":
            return True, w, h, self.encoding, frame

        if self.encoding == "rgb8":
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return True, w, h, self.encoding, rgb

        if self.encoding in ("jpeg", "jpg", "png"):
            ext = "." + ("jpeg" if self.encoding == "jpg" else self.encoding)
            ok, buf = cv2.imencode(ext, frame)
            if not ok:
                return False, 0, 0, self.encoding, None
            return True, w, h, self.encoding, buf.tobytes()

        raise ValueError(f"USBCamera does not support encoding: {self.encoding}")

    def stop(self) -> None:
        logger.info("Shutting down USB camera")
        self._running = False
        sleep(0.1)
        if self._cap is not None:
            self._cap.release()
            self._cap = None
