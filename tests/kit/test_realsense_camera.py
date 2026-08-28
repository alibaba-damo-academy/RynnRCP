"""Tests for the RealSense color camera driver with fake pyrealsense2/cv2."""

from __future__ import annotations

import sys
import types
from typing import Any

import numpy as np
import pytest

from rynnkit.cameras.base import SensorState
from rynnkit.cameras.realsense_camera import RealSenseCamera


class _FakeColorFrame:
    def __init__(self, frame: np.ndarray) -> None:
        self._frame = frame

    def __bool__(self) -> bool:
        return True

    def get_data(self) -> np.ndarray:
        return self._frame


class _FakeFrames:
    def __init__(self, color: Any) -> None:
        self._color = color

    def get_color_frame(self) -> Any:
        return self._color


class _FakeSensor:
    def __init__(self) -> None:
        self.options: list[Any] = []

    def supports(self, option: Any) -> bool:
        return True

    def set_option(self, option: Any, value: float) -> None:
        self.options.append((option, value))


class _FakeProfile:
    def __init__(self) -> None:
        self.sensor = _FakeSensor()

    def get_device(self) -> "_FakeProfile":
        return self

    def first_color_sensor(self) -> _FakeSensor:
        return self.sensor


class _FakePipeline:
    def __init__(self) -> None:
        self.started_with: Any = None
        self.stopped = False
        self.wait_calls = 0
        self.frames: Any = _FakeFrames(None)

    def start(self, config: Any) -> _FakeProfile:
        self.started_with = config
        return _FakeProfile()

    def stop(self) -> None:
        self.stopped = True

    def wait_for_frames(self) -> Any:
        self.wait_calls += 1
        return self.frames


class _FakeConfig:
    def __init__(self) -> None:
        self.enabled_device: str | None = None
        self.streams: list[tuple] = []

    def enable_device(self, serial: str) -> None:
        self.enabled_device = serial

    def enable_stream(self, *args: Any) -> None:
        self.streams.append(args)


def _install_fake_rs(monkeypatch: pytest.MonkeyPatch) -> types.ModuleType:
    rs = types.ModuleType("pyrealsense2")
    pipeline = _FakePipeline()
    rs.pipeline = lambda: pipeline
    rs.config = _FakeConfig
    rs.stream = types.SimpleNamespace(color="color")
    rs.format = types.SimpleNamespace(bgr8="bgr8")
    rs.option = types.SimpleNamespace(
        enable_auto_exposure="auto_exposure",
        enable_auto_white_balance="auto_white_balance",
    )
    rs._pipeline = pipeline
    monkeypatch.setitem(sys.modules, "pyrealsense2", rs)
    return rs


def _install_fake_cv2(monkeypatch: pytest.MonkeyPatch) -> types.ModuleType:
    cv2 = types.ModuleType("cv2")
    cv2.ROTATE_90_CLOCKWISE = 0
    cv2.ROTATE_180 = 1
    cv2.ROTATE_90_COUNTERCLOCKWISE = 2
    cv2.COLOR_BGR2RGB = 4
    cv2.IMWRITE_JPEG_QUALITY = 1

    def rotate(frame: np.ndarray, code: int) -> np.ndarray:
        if code in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
            return np.transpose(frame, (1, 0, 2))
        return frame[::-1]

    cv2.rotate = rotate
    cv2.cvtColor = lambda frame, code: frame[..., ::-1]

    class _Buffer:
        def __init__(self, payload: bytes) -> None:
            self._payload = payload

        def tobytes(self) -> bytes:
            return self._payload

    cv2.imencode = lambda ext, frame, params=None: (True, _Buffer(b"enc" + ext.encode()))
    monkeypatch.setitem(sys.modules, "cv2", cv2)
    return cv2


@pytest.fixture
def fake_backends(monkeypatch: pytest.MonkeyPatch):
    rs = _install_fake_rs(monkeypatch)
    cv2 = _install_fake_cv2(monkeypatch)
    return rs, cv2


def _frame(width: int = 4, height: int = 2) -> np.ndarray:
    return np.zeros((height, width, 3), dtype=np.uint8)


def test_init_prefers_serial_over_device_id() -> None:
    cam = RealSenseCamera(name="rs", serial="S123", device_id="ignored")
    assert cam.serial == "S123"
    assert cam.device_id == "S123"

    fallback = RealSenseCamera(name="rs", device_id="D456")
    assert fallback.serial == "D456"


def test_start_configures_stream_and_warms_up(fake_backends) -> None:
    rs, _cv2 = fake_backends
    cam = RealSenseCamera(
        name="rs", serial="S1", width=320, height=240, fps=15, warmup_frames=3
    )
    cam.start()

    pipeline = rs._pipeline
    assert pipeline.started_with.enabled_device == "S1"
    assert pipeline.started_with.streams == [("color", 320, 240, "bgr8", 15)]
    assert pipeline.wait_calls == 3
    assert cam.state == SensorState.RUNNING
    profile = cam._profile
    assert ("auto_exposure", 1) in profile.sensor.options
    assert ("auto_white_balance", 1) in profile.sensor.options


def test_start_without_serial_skips_enable_device(fake_backends) -> None:
    rs, _cv2 = fake_backends
    cam = RealSenseCamera(name="rs", warmup_frames=0)
    cam.start()
    assert rs._pipeline.started_with.enabled_device is None


def test_stop_shuts_down_pipeline(fake_backends) -> None:
    rs, _cv2 = fake_backends
    cam = RealSenseCamera(name="rs", warmup_frames=0)
    cam.start()
    cam.stop()
    assert rs._pipeline.stopped is True
    assert cam._pipeline is None
    assert cam.state == SensorState.STOPPED


def test_read_frame_requires_start() -> None:
    cam = RealSenseCamera(name="rs")
    with pytest.raises(RuntimeError, match="call start"):
        cam.read_frame()


def test_read_frame_after_stop_flag_returns_failure(fake_backends) -> None:
    cam = RealSenseCamera(name="rs", warmup_frames=0)
    cam.start()
    cam._running = False
    ok, w, h, encoding, data = cam.read_frame()
    assert (ok, w, h, data) == (False, 0, 0, None)
    assert encoding == "jpg"


def test_read_frame_without_color_frame(fake_backends) -> None:
    rs, _cv2 = fake_backends
    cam = RealSenseCamera(name="rs", warmup_frames=0)
    cam.start()
    rs._pipeline.frames = _FakeFrames(None)
    ok, *_rest = cam.read_frame()
    assert ok is False


@pytest.mark.parametrize(
    "encoding, expected_encoding, expects_bytes",
    [
        ("bgr8", "bgr8", False),
        ("rgb8", "rgb8", False),
        ("jpg", "jpg", True),
        ("png", "png", True),
    ],
)
def test_read_frame_encodings(fake_backends, encoding, expected_encoding, expects_bytes) -> None:
    rs, _cv2 = fake_backends
    cam = RealSenseCamera(name="rs", encoding=encoding, warmup_frames=0)
    cam.start()
    rs._pipeline.frames = _FakeFrames(_FakeColorFrame(_frame()))

    ok, w, h, out_encoding, data = cam.read()
    assert ok is True
    assert (w, h) == (4, 2)
    assert out_encoding == expected_encoding
    if expects_bytes:
        assert isinstance(data, bytes)
    else:
        assert isinstance(data, np.ndarray)


def test_read_frame_rotation_changes_dimensions(fake_backends) -> None:
    rs, _cv2 = fake_backends
    cam = RealSenseCamera(name="rs", encoding="bgr8", rotate=90, warmup_frames=0)
    cam.start()
    rs._pipeline.frames = _FakeFrames(_FakeColorFrame(_frame(width=4, height=2)))
    _ok, w, h, _encoding, _data = cam.read_frame()
    assert (w, h) == (2, 4)

    cam_180 = RealSenseCamera(name="rs", encoding="bgr8", rotate=180, warmup_frames=0)
    cam_180.start()
    _ok, w, h, _encoding, _data = cam_180.read_frame()
    assert (w, h) == (4, 2)


def test_read_frame_rejects_unknown_encoding(fake_backends) -> None:
    rs, _cv2 = fake_backends
    cam = RealSenseCamera(name="rs", encoding="yuv", warmup_frames=0)
    cam.start()
    rs._pipeline.frames = _FakeFrames(_FakeColorFrame(_frame()))
    with pytest.raises(ValueError, match="does not support encoding"):
        cam.read_frame()
