"""
Reusable sensor base classes for RynnKit drivers.

These classes are intentionally outside the protocol/runtime core.  The core
PortConnector only requires a driver object with start() / read() / stop().

Design:
  - BaseSensor: generic sensor with start/read/stop lifecycle
  - BaseCamera: specialized sensor for image capture (resolution, encoding, fps)
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum
from typing import Any, Dict, Tuple


# ======================================================================
# Sensor state
# ======================================================================

class SensorState(Enum):
    IDLE = "idle"
    RUNNING = "running"
    ERROR = "error"
    STOPPED = "stopped"


# ======================================================================
# BaseSensor — generic sensor lifecycle
# ======================================================================

class BaseSensor(ABC):
    """Abstract base class for all sensors.

    Lifecycle: __init__ → start() → [read() loop] → stop()

    Implementations must override start(), read(), stop().
    """

    def __init__(self, name: str, frequency_hz: float = 30.0, **kwargs: Any) -> None:
        self.name = name
        self.frequency_hz = frequency_hz
        self.state = SensorState.IDLE
        self._extra_config = kwargs

    @abstractmethod
    def start(self) -> None:
        """Initialize hardware/driver and begin capture.

        Must set self.state = SensorState.RUNNING on success.
        """
        ...

    @abstractmethod
    def read(self) -> Tuple[bool, float, bytes]:
        """Read one sample from the sensor.

        Returns:
            (success, timestamp_sec, data_bytes)
        """
        ...

    @abstractmethod
    def stop(self) -> None:
        """Release resources and stop capture.

        Must set self.state = SensorState.STOPPED.
        """
        ...

    def get_info(self) -> Dict[str, Any]:
        """Return metadata about this sensor."""
        return {
            "name": self.name,
            "frequency_hz": self.frequency_hz,
            "state": self.state.value,
            **self._extra_config,
        }


# ======================================================================
# BaseCamera — image sensor (references RynnRCP BaseCamera)
# ======================================================================

class BaseCamera(BaseSensor):
    """Abstract camera sensor with resolution and encoding.

    Follows RynnRCP BaseCamera pattern:
      - device_id, width, height, encoding, fps, brand, rotate
      - start() → read() → stop()
      - read() returns (success, width, height, encoding, image)
    """

    def __init__(
        self,
        name: str,
        device_id: Any,
        width: int = 640,
        height: int = 480,
        encoding: str = "bgr8",
        fps: float = 30.0,
        brand: str = "Unknown",
        rotate: int = 0,
    ) -> None:
        super().__init__(name=name, frequency_hz=fps)
        self.device_id = device_id
        self.width = width
        self.height = height
        self.encoding = encoding
        self.brand = brand
        self.rotate = rotate

    def get_info(self) -> Dict[str, Any]:
        info = super().get_info()
        info.update({
            "device_id": self.device_id,
            "width": self.width,
            "height": self.height,
            "encoding": self.encoding,
            "brand": self.brand,
            "rotate": self.rotate,
        })
        return info
