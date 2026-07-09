"""Simulation camera for RynnRCP.

Connects to Isaac Sim's FrameManager via ZeroMQ to fetch rendered camera frames.
Follows the BaseCamera interface expected by the PortConnector.
"""

from __future__ import annotations

import logging
import os
import time
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np

from rynnkit.cameras.base import BaseCamera, SensorState
from .zmq_clients import FrameClient

logger = logging.getLogger(__name__)


def _resolve_port(port: int) -> int:
    """If port is 0, read from the PORT environment variable."""
    if int(port) != 0:
        return int(port)
    env_val = os.getenv("PORT")
    if not env_val:
        raise RuntimeError("server_port=0, but environment variable PORT is not set")
    return int(env_val)


class SimCamera(BaseCamera):
    """Simulation camera that fetches frames from Isaac Sim's FrameManager via ZMQ.

    The PortConnector calls:
      - start() → initialize ZMQ connection
      - read() → fetch latest frame
      - stop() → close ZMQ connection

    Constructor args (passed from robot_integration.yaml via port_type init):
      - name: camera identifier for protocol (e.g. "front_camera")
      - device_id: camera name in the simulation (e.g. "camera_front")
      - width, height: expected resolution
      - encoding: pixel format (default "bgr8")
      - fps: capture rate hint
      - server_host: ZMQ server host (default "localhost")
      - server_port: base port of the simulation (default 8080)
                     FrameManager runs on server_port + 3 = 8083
    """

    def __init__(
        self,
        name: str,
        device_id: str = "camera_front",
        width: int = 640,
        height: int = 360,
        encoding: str = "jpg",
        fps: float = 30.0,
        brand: str = "sim_camera",
        rotate: int = 0,
        server_host: str = "localhost",
        server_port: int = 8080,
        native_compressed: bool = True,
        **kwargs: Any,
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
        self.native_compressed = native_compressed
        self.server_host = server_host
        self.server_port = _resolve_port(server_port)
        self._frame_port = self.server_port + 3  # FrameManager offset

        self._frame_client: FrameClient | None = None
        self._blank_frame = np.zeros((int(height), int(width), 3), dtype=np.uint8)

    def start(self) -> None:
        """Connect to FrameManager and mark sensor as running."""
        self._frame_client = FrameClient(
            server_host=self.server_host,
            server_port=self._frame_port,
        )
        self.state = SensorState.RUNNING
        logger.info(
            "SimCamera started: name=%s, device_id=%s, size=(%dx%d), server=%s:%s",
            self.name,
            self.device_id,
            self.width,
            self.height,
            self.server_host,
            self._frame_port,
        )

    def read(self) -> Tuple[bool, int, int, str, Optional[bytes | np.ndarray]]:
        """Fetch the latest frame from the simulation.

        Returns:
            (success, width, height, encoding, image_data)
            When native_compressed=True, returns JPEG bytes.
        """
        if self.state != SensorState.RUNNING or self._frame_client is None:
            return False, 0, 0, self.encoding, None

        frame = self._frame_client.get_frame(str(self.device_id))
        if frame is not None:
            if self.native_compressed and self.encoding == "jpg":
                # Encode BGR8 numpy array to JPEG bytes
                _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                return True, int(self.width), int(self.height), "jpg", buf.tobytes()
            return True, int(self.width), int(self.height), self.encoding, frame
        else:
            return False, int(self.width), int(self.height), self.encoding, None

    def stop(self) -> None:
        """Close the ZMQ connection and mark sensor as stopped."""
        if self._frame_client is not None:
            self._frame_client.close()
            self._frame_client = None
        self.state = SensorState.STOPPED
        logger.info("SimCamera stopped: name=%s, device_id=%s", self.name, self.device_id)
