"""
OpenCV USB camera driver.
~~~~~~~~~~~~~~~~~~~~~~~~~

Concrete implementation of :class:`~rynnkit.cameras.base.BaseCamera` using
OpenCV's :class:`cv2.VideoCapture`.

Ported from RynnRCP ``rcp_sensor/camera/usb_camera.py`` with the following
adaptations:

- Inherits rynnkit ``BaseCamera`` (which itself extends ``BaseSensor``).
- ``read()`` and ``read_frame()`` return the RynnRCP image tuple
  ``(success, width, height, encoding, image)`` for callers that need
  the decoded image directly.
- ``rcp_core.common.utils.logger`` replaced with stdlib ``logging``.
- ``cv2`` / ``numpy`` are optional imports (try-import).

Platform / backends
-------------------
- Linux:   ``cv2.CAP_GSTREAMER`` when available, falling back to ``cv2.CAP_V4L2``
- macOS:   ``cv2.CAP_AVFOUNDATION``
- Windows: ``cv2.CAP_DSHOW``
- other:   ``cv2.CAP_ANY``

Device ID handling
------------------
``device_id`` may be:
- an integer index (``0``, ``1`` ...)
- a numeric string (``"0"`` -> index 0)
- a device path (Linux, e.g. ``/dev/video0``)
- macOS AVFoundation uniqueID (UUID/hex-like), optionally prefixed ``"avf:"``
"""

from __future__ import annotations

import importlib
import logging
import re
import shutil
from time import sleep
from typing import Any, Tuple, Union

# Use importlib to avoid shadowing by rynnrcp's own ``platform`` package.
_stdlib_platform = importlib.import_module("platform")

from rynnkit.cameras.base import BaseCamera, SensorState

try:
    import cv2

    _HAS_CV2 = True
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore[assignment]
    _HAS_CV2 = False

logger = logging.getLogger(__name__)


# ------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------

def _pick_cv_backend() -> int:
    """Select OpenCV capture backend based on OS."""
    sysname = _stdlib_platform.system()
    if sysname == "Linux":
        return cv2.CAP_V4L2
    if sysname == "Darwin":
        return cv2.CAP_AVFOUNDATION
    if sysname == "Windows":
        return cv2.CAP_DSHOW
    return cv2.CAP_ANY


def _is_probably_macos_uniqueid(s: str) -> bool:
    """Check whether *s* looks like a macOS AVFoundation uniqueID.

    Examples::

        0x213000010bb2b08          (hex-like)
        1FD4B3A2-236E-492B-8CE5-255DD288CE50  (UUID)
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
    """Map AVFoundation uniqueID -> index in AVFoundation device list."""
    try:
        import AVFoundation  # type: ignore[import-untyped]
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


# ------------------------------------------------------------------
# USBCamera
# ------------------------------------------------------------------

class USBCamera(BaseCamera):
    """OpenCV-based USB camera driver.

    Extends rynnrcp ``BaseCamera`` with OpenCV ``VideoCapture``.

    Parameters match ``BaseCamera.__init__`` plus the ``name`` field
    required by rynnrcp's ``BaseSensor`` hierarchy.
    """

    def __init__(
        self,
        name: str,
        device_id: Union[str, int],
        width: int = 640,
        height: int = 480,
        encoding: str = "bgr8",
        fps: float = 30.0,
        brand: str = "Unknown",
        rotate: int = 0,
        native_compressed: bool = False,
        capture_backend: str = "auto",
    ) -> None:
        super().__init__(
            name=name,
            device_id=device_id,
            width=width,
            height=height,
            encoding=encoding,
            fps=fps,
            brand=brand,
            rotate=rotate,
        )
        self._cap: Any = None  # cv2.VideoCapture or None
        self._running: bool = False
        self.native_compressed = bool(native_compressed)
        self.capture_backend = str(capture_backend or "auto").strip().lower()
        if self.capture_backend not in {"auto", "opencv", "v4l2", "gstreamer"}:
            raise ValueError("capture_backend must be one of: auto, opencv, v4l2, gstreamer")
        self._native_compressed_active = False
        self._native_compressed_warning_logged = False
        self._capture_backend_active = "none"
        self._failed_reads = 0
        self._reopen_after_failed_reads = 5

    # ----------------------------------------------------------
    # Lifecycle
    # ----------------------------------------------------------

    def start(self) -> None:
        """Open the USB camera device and set the resolution.

        Raises ``RuntimeError`` if OpenCV is not installed or the device
        cannot be opened.
        """
        if not _HAS_CV2:
            raise RuntimeError(
                "OpenCV (cv2) is not installed. "
                "Install with: pip install opencv-python"
            )

        sysname = _stdlib_platform.system()
        backend = _pick_cv_backend()

        dev = _resolve_device_id(self.device_id, sysname)
        self._native_compressed_active = False

        cap = None
        if self._should_try_gstreamer(sysname):
            cap = self._try_open_gstreamer(dev)
            if cap is None and self.capture_backend == "gstreamer":
                raise RuntimeError(
                    f"Unable to open camera device with GStreamer: {self.device_id} "
                    f"(resolved={dev}, platform={sysname})"
                )

        if cap is None:
            cap = cv2.VideoCapture(dev, backend)
            self._capture_backend_active = "opencv"
        if not cap.isOpened():
            raise RuntimeError(
                f"Unable to open camera device: {self.device_id} "
                f"(resolved={dev}, platform={sysname}, backend={backend})"
            )

        # Linux: Force MJPG to reduce USB bandwidth
        if sysname == "Linux" and self._capture_backend_active != "gstreamer":
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            cap.set(cv2.CAP_PROP_FOURCC, fourcc)

        if self.width is not None and self._capture_backend_active != "gstreamer":
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height is not None and self._capture_backend_active != "gstreamer":
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.frequency_hz is not None and self._capture_backend_active != "gstreamer":
            cap.set(cv2.CAP_PROP_FPS, self.frequency_hz)

        if (
            sysname == "Linux"
            and self.native_compressed
            and str(self.encoding).lower() in ("jpg", "jpeg")
            and int(self.rotate or 0) == 0
            and self._capture_backend_active != "gstreamer"
        ):
            convert_rgb_prop = getattr(cv2, "CAP_PROP_CONVERT_RGB", None)
            if convert_rgb_prop is not None:
                cap.set(convert_rgb_prop, 0)
                self._native_compressed_active = True

        self._cap = cap
        self._running = True
        self.state = SensorState.RUNNING

    def stop(self) -> None:
        """Stop capture and release resources."""
        logger.info("Shutting down USB camera: %s", self.name)
        self._running = False
        sleep(0.1)
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self.state = SensorState.STOPPED
        self._capture_backend_active = "none"

    # ----------------------------------------------------------
    # read_frame — original RynnRCP-style return
    # ----------------------------------------------------------

    def read_frame(self) -> Tuple[bool, int, int, str, Any]:
        """Read one frame with the original RynnRCP return signature.

        Returns:
            ``(success, width, height, encoding, image)`` where *image*
            is a NumPy array (bgr8/rgb8) or ``bytes`` (jpeg/png), or
            ``None`` on failure.
        """
        if self._cap is None:
            raise RuntimeError("Camera not opened, please call start() first.")

        if not self._running:
            return False, 0, 0, self.encoding, None

        ok, frame = self._cap.read()
        if not ok:
            self._failed_reads += 1
            logger.warning("%s: Failed to read a frame", self.device_id)
            if self._failed_reads >= self._reopen_after_failed_reads:
                self._reopen()
            return False, 0, 0, self.encoding, None
        self._failed_reads = 0

        if self._native_compressed_active and str(self.encoding).lower() in ("jpg", "jpeg"):
            native_bytes = _extract_native_jpeg_bytes(frame)
            if native_bytes is not None:
                return True, int(self.width), int(self.height), self.encoding, native_bytes
            if not self._native_compressed_warning_logged:
                logger.warning(
                    "%s: native compressed mode did not return JPEG bytes; falling back to OpenCV encode",
                    self.name,
                )
                self._native_compressed_warning_logged = True
            self._native_compressed_active = False

        # Rotation
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
            ok_enc, buf = cv2.imencode(ext, frame)
            if not ok_enc:
                return False, 0, 0, self.encoding, None
            return True, w, h, self.encoding, buf.tobytes()

        raise ValueError(f"USBCamera does not support encoding: {self.encoding}")

    # ----------------------------------------------------------
    # read — RynnRCP image tuple contract
    # ----------------------------------------------------------

    def read(self) -> Tuple[bool, int, int, str, Any]:
        """Read one image sample in the RynnRCP image tuple format."""
        return self.read_frame()

    def _should_try_gstreamer(self, sysname: str) -> bool:
        if sysname != "Linux":
            return False
        if self.capture_backend not in {"auto", "gstreamer"}:
            return False
        return _has_gstreamer()

    def _try_open_gstreamer(self, dev: Union[int, str]) -> Any | None:
        cap_gstreamer = getattr(cv2, "CAP_GSTREAMER", None)
        if cap_gstreamer is None:
            return None

        compressed = (
            self.native_compressed
            and str(self.encoding).lower() in ("jpg", "jpeg")
            and int(self.rotate or 0) == 0
        )
        device_path = _device_to_linux_video_path(dev)
        pipelines = [
            _build_gstreamer_pipeline(
                device_path=device_path,
                width=int(self.width),
                height=int(self.height),
                fps=float(self.frequency_hz),
                compressed=compressed,
            )
        ]
        if compressed:
            pipelines.append(
                _build_gstreamer_pipeline(
                    device_path=device_path,
                    width=int(self.width),
                    height=int(self.height),
                    fps=float(self.frequency_hz),
                    compressed=False,
                )
            )

        for index, pipeline in enumerate(pipelines):
            cap = cv2.VideoCapture(pipeline, cap_gstreamer)
            if cap.isOpened():
                self._capture_backend_active = "gstreamer"
                self._native_compressed_active = compressed and index == 0
                logger.info(
                    "USB camera %s opened with GStreamer%s",
                    self.name,
                    " native JPEG passthrough" if self._native_compressed_active else "",
                )
                return cap
            cap.release()

        return None

    def _reopen(self) -> None:
        logger.warning("%s: reopening camera after %d failed reads", self.device_id, self._failed_reads)
        self._failed_reads = 0
        self._running = False
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        sleep(0.2)
        self.start()


def _resolve_device_id(device_id: Union[str, int], sysname: str) -> Union[int, str]:
    if isinstance(device_id, int):
        return device_id

    s = str(device_id).strip()
    if sysname == "Darwin" and _is_probably_macos_uniqueid(s):  # pragma: no cover
        idx = _macos_uniqueid_to_index(s)
        logger.info("macOS uniqueID mapped to index: %s -> %d", s, idx)
        return idx

    try:
        return int(s)
    except ValueError:
        return s


def _has_gstreamer() -> bool:
    return shutil.which("gst-launch-1.0") is not None


def _device_to_linux_video_path(dev: Union[int, str]) -> str:
    if isinstance(dev, int):
        return f"/dev/video{dev}"
    text = str(dev).strip()
    if text.isdigit():
        return f"/dev/video{text}"
    return text


def _build_gstreamer_pipeline(
    *,
    device_path: str,
    width: int,
    height: int,
    fps: float,
    compressed: bool,
) -> str:
    fps_num = max(1, int(round(fps)))
    caps = f"image/jpeg,width={width},height={height},framerate={fps_num}/1"
    sink = "appsink drop=true max-buffers=1 sync=false"
    if compressed:
        return f"v4l2src device={device_path} ! {caps} ! {sink}"
    return (
        f"v4l2src device={device_path} ! {caps} ! "
        f"jpegparse ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! {sink}"
    )


def _extract_native_jpeg_bytes(frame: Any) -> bytes | None:
    try:
        if isinstance(frame, bytes):
            data = frame
        elif isinstance(frame, bytearray):
            data = bytes(frame)
        elif isinstance(frame, memoryview):
            data = frame.cast("B").tobytes()
        else:
            data = memoryview(frame).cast("B").tobytes()
    except (TypeError, ValueError):
        return None
    if len(data) >= 4 and data[:2] == b"\xff\xd8":
        return data
    return None
