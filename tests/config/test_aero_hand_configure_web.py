"""Checks for the Aero Hand browser configure helper."""

from __future__ import annotations

import sys
import time
from typing import Any

import pytest

from rynnrcp_robot_aero_hand import configure_aero_hand_web as web
from rynnrcp_robot_aero_hand.aero_open_sdk.aero_hand import HOMING_MODE
from rynnrcp.utils.web_urls import primary_browser_url


class _Serial:
    def __init__(self) -> None:
        self.reset_count = 0

    def reset_input_buffer(self) -> None:
        self.reset_count += 1


class _Hand:
    def __init__(self) -> None:
        self.ser = _Serial()
        self.sent: list[int] = []
        self.closed = False

    def _send_data(self, header: int) -> None:
        self.sent.append(header)

    def close(self) -> None:
        self.closed = True


def test_fallback_serial_devices_only_include_mapped_windows_com_ports(monkeypatch: Any) -> None:
    monkeypatch.setattr(web.glob, "glob", lambda pattern: [])
    monkeypatch.setattr(web.os, "name", "nt")
    monkeypatch.setattr(web, "_windows_serial_devices", lambda: ["COM3", "COM4"])

    assert web._fallback_serial_devices() == ["COM3", "COM4"]


def test_windows_fallback_does_not_manufacture_ports(monkeypatch: Any) -> None:
    monkeypatch.setattr(web.glob, "glob", lambda pattern: [])
    monkeypatch.setattr(web.os, "name", "nt")
    monkeypatch.setattr(web, "_windows_serial_devices", lambda: [])

    assert web._fallback_serial_devices() == []


def test_windows_serial_devices_reads_only_mapped_com_ports(monkeypatch: Any) -> None:
    class FakeWinreg:
        HKEY_LOCAL_MACHINE = object()
        values = [
            (r"\Device\Serial2", "COM10", 1),
            (r"\Device\Serial0", "COM3", 1),
            (r"\Device\Other", "not-a-com-port", 1),
        ]

        @staticmethod
        def OpenKey(root: object, path: str) -> object:
            assert root is FakeWinreg.HKEY_LOCAL_MACHINE
            assert path == r"HARDWARE\DEVICEMAP\SERIALCOMM"
            return object()

        @classmethod
        def EnumValue(cls, _key: object, index: int) -> tuple[str, str, int]:
            if index >= len(cls.values):
                raise OSError
            return cls.values[index]

        @staticmethod
        def CloseKey(_key: object) -> None:
            return None

    monkeypatch.setitem(sys.modules, "winreg", FakeWinreg)

    assert web._windows_serial_devices() == ["COM3", "COM10"]


def test_windows_com_ports_are_deduplicated_and_sorted_naturally() -> None:
    ports = [
        {"device": "COM10", "description": "ten", "hwid": ""},
        {"device": "com3", "description": "CH343", "hwid": "USB VID:PID=1A86:55D3"},
        {"device": "COM4", "description": "four", "hwid": ""},
        {"device": "COM3", "description": "fallback", "hwid": ""},
    ]

    result = web._dedupe_serial_ports(ports)

    assert [port["device"] for port in result] == ["COM3", "COM4", "COM10"]
    assert result[0]["description"] == "CH343"


def test_configure_ui_prints_browser_safe_localhost_for_wildcard_bind() -> None:
    assert primary_browser_url("0.0.0.0", 28411) == "http://127.0.0.1:28411/"
    assert primary_browser_url("", 28411) == "http://127.0.0.1:28411/"
    assert primary_browser_url("192.168.1.2", 28411) == "http://192.168.1.2:28411/"


def test_config_ui_includes_single_and_dual_hand_master_settings() -> None:
    assert 'id="master-camera-index"' in web.HTML
    assert "observation.images.front" in web.HTML
    assert "同一个摄像头实例同时提供手势识别画面" in web.HTML
    assert 'id="master-product-key"' in web.HTML
    assert "扫描可用摄像头" in web.HTML
    assert "用哪只手做动作" not in web.HTML
    assert "高级识别设置" not in web.HTML
    assert "本地手势遥操 / Teleop / 数采" in web.HTML
    assert "填写 0 套凭据" in web.HTML
    assert "填写 2 套凭据" not in web.HTML
    assert "真机执行端凭据（接入 RynnBot 时必填）" in web.HTML
    assert "摄像手势控制端凭据（控制端接入 RynnBot 时必填）" in web.HTML
    assert "手势控制端控制仿真目标" in web.HTML
    assert "填写“摄像手势控制端”" in web.HTML
    assert "手势控制端控制真实 Aero Hand" not in web.HTML
    assert "这套凭据用于摄像手势控制服务" in web.HTML
    assert "请根据本次使用方式填写其中一组" in web.HTML
    assert web.HTML.index("摄像手势控制端凭据（控制端接入 RynnBot 时必填）") < web.HTML.index("手势摄像头与采集画面")
    assert 'id="camera-teleop-start"' in web.HTML
    assert 'id="camera-teleop-stop"' in web.HTML
    assert 'id="camera-teleop-preview-image"' in web.HTML
    assert "CAMERA_TELEOP_PREVIEW_INTERVAL_MS = 33" in web.HTML
    assert "aero_hand_single_hand_master_server.yaml" in str(web.CAMERA_MASTER_CONFIG_PATHS["single"])
    assert "aero_hand_dual_hand_master_server.yaml" in str(web.CAMERA_MASTER_CONFIG_PATHS["dual"])


def test_camera_master_profile_ids_are_distinct() -> None:
    assert web._camera_master_robot_id("single") == "aero_hand_single_hand_master"
    assert web._camera_master_robot_id("dual") == "aero_hand_dual_hand_master"
    assert web._camera_master_rynnbot_app_id("single") == "aero_hand_single_hand_master_rynnbot_app"
    assert web._camera_master_rynnbot_app_id("dual") == "aero_hand_dual_hand_master_rynnbot_app"
    assert "Leader" in web._camera_master_robot_name("single")
    assert "Leader" in web._camera_master_robot_name("dual")


def test_camera_probe_returns_preview_and_releases_device() -> None:
    class Frame:
        shape = (240, 320, 3)

    class EncodedImage:
        @staticmethod
        def tobytes() -> bytes:
            return b"jpeg"

    class Capture:
        released = False

        @staticmethod
        def isOpened() -> bool:
            return True

        @staticmethod
        def read() -> tuple[bool, Frame]:
            return True, Frame()

        @classmethod
        def release(cls) -> None:
            cls.released = True

    class Cv2:
        IMWRITE_JPEG_QUALITY = 1

        @staticmethod
        def VideoCapture(index: int) -> Capture:
            assert index == 3
            return Capture()

        @staticmethod
        def imencode(extension: str, frame: Frame, options: list[int]) -> tuple[bool, EncodedImage]:
            assert extension == ".jpg"
            assert options == [1, 75]
            return True, EncodedImage()

    result = web._probe_camera_with_cv2(Cv2, 3)

    assert result == {
        "ok": True,
        "camera": {
            "index": 3,
            "width": 320,
            "height": 240,
            "image": "data:image/jpeg;base64,anBlZw==",
        },
    }
    assert Capture.released is True


def test_camera_teleop_job_uses_smoothed_controller(monkeypatch: Any) -> None:
    class Vision:
        started = False
        stopped = False

        def start(self) -> None:
            self.started = True

        def get_joint_positions(self) -> dict[str, list[float]]:
            return {"joint_positions": [0.1] * 7}

        def shutdown(self) -> None:
            self.stopped = True

        @staticmethod
        def get_preview_jpeg() -> bytes:
            return b"jpeg-preview"

    class Controller:
        def __init__(self) -> None:
            self.targets: list[list[float]] = []
            self.started = False
            self.stopped = False

        def start(self) -> None:
            self.started = True

        def set_joint_positions(self, value: dict[str, list[float]]) -> None:
            self.targets.append(value["joint_positions"])

        def shutdown(self) -> None:
            self.stopped = True

    vision = Vision()
    controller = Controller()
    monkeypatch.setattr(web, "_new_camera_teleop_vision", lambda profile, payload: vision)
    monkeypatch.setattr(
        web,
        "_new_camera_teleop_controller",
        lambda profile, payload: controller,
    )
    job = web.CameraTeleopJob()

    job.start({"profile": "single"})
    deadline = time.monotonic() + 1.0
    while not controller.targets and time.monotonic() < deadline:
        time.sleep(0.01)
    status = job.stop()

    assert vision.started is True
    assert vision.stopped is True
    assert controller.started is True
    assert controller.stopped is True
    assert controller.targets
    assert controller.targets[0] == pytest.approx([0.1] * 7)
    assert status["active"] is False
    assert status["state"] == "stopped"
    assert job.preview_jpeg() == b"jpeg-preview"


def test_homing_sends_all_hands_without_reading_state(monkeypatch: Any) -> None:
    hands = {"left": _Hand(), "right": _Hand()}
    sleeps: list[float] = []

    monkeypatch.setattr(web, "_new_hands", lambda payload: (hands, 921600))
    monkeypatch.setattr(
        web,
        "_load_config",
        lambda profile: {"components": {"robot": {"homing_settle_s": 0.25}}},
    )
    monkeypatch.setattr(web.time, "sleep", sleeps.append)

    result = web._homing({"profile": "dual"})

    assert result == {"homed": ["left", "right"], "settle_s": 0.25}
    assert sleeps == [0.25]
    assert [hand.sent for hand in hands.values()] == [[HOMING_MODE], [HOMING_MODE]]
    assert all(hand.closed for hand in hands.values())
