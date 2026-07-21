from __future__ import annotations

import sys
from typing import Any

from rynnrcp_robot_lekiwi import configure_lekiwi_web as web


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


def test_apply_snapshot_uses_one_machine_suffix_for_all_ids(monkeypatch: Any) -> None:
    monkeypatch.setattr(web, "machine_mac_suffix", lambda: "1234abcd")
    configs = {
        "follower_server": {
            "manifest": {},
            "components": {
                "robot": {},
                "front_camera": {},
                "wrist_camera": {},
            },
        },
        "leader_server": {"manifest": {}, "components": {"robot": {}}},
        "rynnbot_app": {"app": {"app_id": "lekiwi_rynnbot_app"}},
    }
    snapshot = {
        "server": {"follower_id": "lekiwi_follower", "leader_id": "lekiwi_leader"},
        "hardware": {
            "follower_port": "/dev/follower",
            "leader_port": "/dev/leader",
            "front_camera": 0,
            "wrist_camera": 1,
            "front_rotate": 180,
            "wrist_rotate": 180,
        },
        "rynnbot": {
            "product_key": "",
            "device_name": "",
            "device_secret": "",
            "http_url": "https://example.com",
            "image_upload_codec": "jpeg",
        },
    }

    web.apply_snapshot(configs, snapshot)
    web.apply_snapshot(configs, snapshot)

    assert configs["follower_server"]["manifest"]["robot_id"] == "lekiwi_follower_1234abcd"
    assert configs["leader_server"]["manifest"]["robot_id"] == "lekiwi_leader_1234abcd"
    assert configs["rynnbot_app"]["app"]["app_id"] == "lekiwi_rynnbot_app_1234abcd"


def test_config_ui_displays_generated_app_id() -> None:
    assert 'id="app-id" readonly' in web.HTML
    assert "本地 Teleop、MCP、Protocol Debug" in web.HTML
    assert "保留两个 Robot ID，然后继续相机配置" in web.HTML
    assert "主臂由本地 Teleop App 连接，页面仅提供从臂云端凭据" in web.HTML
    assert "从臂 RynnBot App ID（自动生成）" in web.HTML


def test_build_snapshot_exposes_generated_app_id(monkeypatch: Any) -> None:
    monkeypatch.setattr(web, "machine_mac_suffix", lambda: "1234abcd")
    configs = {
        "follower_server": {
            "manifest": {"robot_id": "lekiwi_follower"},
            "components": {
                "robot": {"port": "/dev/follower"},
                "front_camera": {"device_id": 0, "rotate": 180},
                "wrist_camera": {"device_id": 1, "rotate": 180},
            },
        },
        "leader_server": {
            "manifest": {"robot_id": "lekiwi_leader"},
            "components": {"robot": {"port": "/dev/leader"}},
        },
        "rynnbot_app": {"app": {"app_id": "lekiwi_rynnbot_app"}},
    }

    snapshot = web.build_snapshot(configs)

    assert snapshot["server"] == {
        "follower_id": "lekiwi_follower_1234abcd",
        "leader_id": "lekiwi_leader_1234abcd",
    }
    assert snapshot["rynnbot"]["app_id"] == "lekiwi_rynnbot_app_1234abcd"
