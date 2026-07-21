"""Intel RealSense color camera driver."""

from __future__ import annotations

import logging
from time import sleep
from typing import Any, Tuple

from rynnkit.cameras.base import BaseCamera, SensorState

logger = logging.getLogger(__name__)


class RealSenseCamera(BaseCamera):
    """RealSense color stream driver.

    The driver returns the standard RCP image tuple:
    ``(success, width, height, encoding, image)``.
    """

    def __init__(
        self,
        name: str,
        serial: str | None = None,
        device_id: str | None = None,
        width: int = 640,
        height: int = 480,
        encoding: str = "jpg",
        fps: float = 30.0,
        brand: str = "Intel RealSense",
        rotate: int = 0,
        warmup_frames: int = 60,
        jpeg_quality: int = 85,
    ) -> None:
        dev = serial or device_id or ""
        super().__init__(
            name=name,
            device_id=dev,
            width=width,
            height=height,
            encoding=encoding,
            fps=fps,
            brand=brand,
            rotate=rotate,
        )
        self.serial = dev
        self.warmup_frames = int(warmup_frames)
        self.jpeg_quality = int(jpeg_quality)
        self._pipeline: Any = None
        self._profile: Any = None
        self._running = False

    def start(self) -> None:
        try:
            import pyrealsense2 as rs
        except ImportError as e:  # pragma: no cover
            raise RuntimeError(
                "pyrealsense2 is not installed. Install Intel librealsense "
                "and its Python binding on the robot."
            ) from e

        pipeline = rs.pipeline()
        config = rs.config()
        if self.serial:
            config.enable_device(str(self.serial))
        config.enable_stream(
            rs.stream.color,
            int(self.width),
            int(self.height),
            rs.format.bgr8,
            int(self.frequency_hz),
        )

        profile = pipeline.start(config)
        self._enable_auto_controls(profile)
        for _ in range(max(0, self.warmup_frames)):
            pipeline.wait_for_frames()

        self._pipeline = pipeline
        self._profile = profile
        self._running = True
        self.state = SensorState.RUNNING

    def stop(self) -> None:
        logger.info("Shutting down RealSense camera: %s", self.name)
        self._running = False
        sleep(0.1)
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
            self._profile = None
        self.state = SensorState.STOPPED

    def read_frame(self) -> Tuple[bool, int, int, str, Any]:
        if self._pipeline is None:
            raise RuntimeError("Camera not opened, please call start() first.")
        if not self._running:
            return False, 0, 0, self.encoding, None

        frames = self._pipeline.wait_for_frames()
        color = frames.get_color_frame()
        if not color:
            return False, 0, 0, self.encoding, None

        import cv2
        import numpy as np

        frame = np.asanyarray(color.get_data())
        if self.rotate == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif self.rotate == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        h, w = frame.shape[:2]
        encoding = str(self.encoding).lower()
        if encoding == "bgr8":
            return True, w, h, "bgr8", frame
        if encoding == "rgb8":
            return True, w, h, "rgb8", cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if encoding in ("jpg", "jpeg"):
            params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            ok, buf = cv2.imencode(".jpg", frame, params)
            return (True, w, h, "jpg", buf.tobytes()) if ok else (False, 0, 0, "jpg", None)
        if encoding == "png":
            ok, buf = cv2.imencode(".png", frame)
            return (True, w, h, "png", buf.tobytes()) if ok else (False, 0, 0, "png", None)
        raise ValueError(f"RealSenseCamera does not support encoding: {self.encoding}")

    def read(self) -> Tuple[bool, int, int, str, Any]:
        return self.read_frame()

    @staticmethod
    def _enable_auto_controls(profile: Any) -> None:
        try:
            import pyrealsense2 as rs

            sensor = profile.get_device().first_color_sensor()
            for option in (rs.option.enable_auto_exposure, rs.option.enable_auto_white_balance):
                if sensor.supports(option):
                    sensor.set_option(option, 1)
        except Exception:
            logger.debug("Unable to enable RealSense auto controls", exc_info=True)
