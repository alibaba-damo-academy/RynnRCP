"""Tests for shared sensor base contracts."""

from __future__ import annotations

import struct
import time

from rynnkit.cameras.base import BaseCamera, BaseSensor, SensorState


class MockIMU(BaseSensor):
    def __init__(self) -> None:
        super().__init__(name="mock_imu", frequency_hz=100.0, axis_count=3)

    def start(self) -> None:
        self.state = SensorState.RUNNING

    def read(self) -> tuple[bool, float, bytes]:
        return True, time.monotonic(), struct.pack("<3f", 0.0, 0.0, 9.81)

    def stop(self) -> None:
        self.state = SensorState.STOPPED


class MockCamera(BaseCamera):
    def __init__(self) -> None:
        super().__init__(
            name="mock_cam",
            device_id=0,
            width=4,
            height=4,
            encoding="rgb8",
            fps=30.0,
            brand="MockBrand",
            rotate=90,
        )

    def start(self) -> None:
        self.state = SensorState.RUNNING

    def read(self) -> tuple[bool, float, bytes]:
        return True, time.monotonic(), bytes(self.width * self.height * 3)

    def stop(self) -> None:
        self.state = SensorState.STOPPED


def test_base_sensor_lifecycle_and_info() -> None:
    sensor = MockIMU()

    assert sensor.get_info() == {
        "name": "mock_imu",
        "frequency_hz": 100.0,
        "state": "idle",
        "axis_count": 3,
    }
    sensor.start()
    ok, _timestamp, data = sensor.read()
    sensor.stop()

    assert ok is True
    assert len(data) == 12
    assert sensor.state is SensorState.STOPPED


def test_base_camera_info_includes_camera_metadata() -> None:
    camera = MockCamera()

    assert camera.get_info() == {
        "name": "mock_cam",
        "frequency_hz": 30.0,
        "state": "idle",
        "device_id": 0,
        "width": 4,
        "height": 4,
        "encoding": "rgb8",
        "brand": "MockBrand",
        "rotate": 90,
    }
