from __future__ import annotations

import sys
from typing import Any

from rynnrcp_robot_so101 import configure_so101_web as web


def test_fallback_serial_devices_only_include_mapped_windows_com_ports(monkeypatch: Any) -> None:
    monkeypatch.setattr(web.glob, "glob", lambda pattern: [])
    monkeypatch.setattr(web.os, "name", "nt")
    monkeypatch.setattr(web, "_windows_serial_devices", lambda: ["COM3", "COM4"])

    devices = web._fallback_serial_devices()

    assert devices == ["COM3", "COM4"]


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


def test_windows_com_ports_are_deduplicated_case_insensitively_and_sorted_naturally() -> None:
    ports = [
        {"device": "COM10", "description": "ten", "hwid": ""},
        {"device": "com3", "description": "CH343", "hwid": "USB VID:PID=1A86:55D3"},
        {"device": "COM4", "description": "four", "hwid": ""},
        {"device": "COM3", "description": "fallback", "hwid": ""},
    ]

    result = web._dedupe_serial_ports(ports)

    assert [port["device"] for port in result] == ["COM3", "COM4", "COM10"]
    assert result[0]["description"] == "CH343"


def test_fallback_serial_devices_include_unix_usb_devices(monkeypatch: Any) -> None:
    monkeypatch.setattr(web.glob, "glob", lambda pattern: [pattern.replace("*", "0")])
    monkeypatch.setattr(web.os, "name", "posix")

    devices = web._fallback_serial_devices()

    assert "/dev/ttyACM0" in devices
    assert "/dev/cu.usbmodem0" in devices


def test_debug_joint_slider_clamps_and_preserves_other_joints() -> None:
    class FakeController:
        positions = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]

        def get_joint_positions(self):
            return {"joint_positions": list(self.positions)}

        def set_joint_positions(self, value):
            self.positions = list(value["joint_positions"])

    job = web.PresetMotionJob()
    job._controller = FakeController()

    status = job.set_joint(5, 2.0)

    assert status["positions"] == [0.0, 0.1, 0.2, 0.3, 0.4, 1.0]
    assert job._debug_targets == [0.0, 0.1, 0.2, 0.3, 0.4, 1.0]


def test_debug_slider_does_not_feed_joint_state_noise_back_into_other_targets() -> None:
    class NoisyController:
        positions = [0.0] * 6
        commands = []

        def get_joint_positions(self):
            self.positions[0] += 0.01
            return {"joint_positions": list(self.positions)}

        def set_joint_positions(self, value):
            self.commands.append(list(value["joint_positions"]))

    job = web.PresetMotionJob()
    controller = NoisyController()
    job._controller = controller
    job._debug_targets = [0.0] * 6

    job.set_joint(5, 0.4)
    job.set_joint(5, 0.6)

    assert controller.commands == [[0.0, 0.0, 0.0, 0.0, 0.0, 0.4], [0.0, 0.0, 0.0, 0.0, 0.0, 0.6]]


def test_apply_snapshot_writes_follower_and_master_rynnbot_credentials_separately() -> None:
    configs = {
        "follower_server": {
            "manifest": {},
            "integration": {},
            "components": {"robot": {}, "front_camera": {}, "wrist_camera": {}},
        },
        "leader_server": {"manifest": {}, "integration": {}, "components": {"robot": {}}},
        "rynnbot_app": {"app": {"app_id": "so101_rynnbot_app"}},
        "master_rynnbot_app": {
            "app": {
                "app_id": "so101_master_rynnbot_app",
                "role": "controller",
                "image_upload_codec": "jpeg",
            },
            "master_source": {"enabled": True},
        },
    }
    snapshot = {
        "server": {"id": "so101_follower"},
        "hardware": {
            "follower_port": "/dev/follower",
            "leader_port": "/dev/leader",
            "front_camera": 0,
            "wrist_camera": 1,
        },
        "rynnbot": {
            "product_key": "follower-key",
            "device_name": "follower-device",
            "device_secret": "follower-secret",
            "http_url": "https://follower.example.com",
            "image_upload_codec": "jpeg",
        },
        "master_rynnbot": {
            "product_key": "leader-key",
            "device_name": "leader-device",
            "device_secret": "leader-secret",
            "http_url": "https://leader.example.com",
        },
    }

    web.apply_snapshot_to_configs(configs, snapshot)

    assert configs["rynnbot_app"]["app"]["device_name"] == "follower-device"
    assert configs["master_rynnbot_app"]["app"]["device_name"] == "leader-device"
    assert configs["master_rynnbot_app"]["app"]["role"] == "controller"
    assert "image_upload_codec" not in configs["master_rynnbot_app"]["app"]
    assert configs["master_rynnbot_app"]["master_source"] == {"enabled": True}


def test_config_ui_explains_when_master_rynnbot_credentials_are_needed() -> None:
    assert "本地动作测试、本地 Teleop、MCP" in web.HTML_TEMPLATE
    assert "0 套。保留 Robot ID，然后继续下一步" in web.HTML_TEMPLATE
    assert "从臂执行端凭据（从臂本体接入 RynnBot 时必填）" in web.HTML_TEMPLATE
    assert "主臂控制端凭据（主臂接入 RynnBot 时必填）" in web.HTML_TEMPLATE
    assert "真实主臂通过 RynnBot 控制仿真从臂" in web.HTML_TEMPLATE
    assert "1 套，填写“主臂控制端”" in web.HTML_TEMPLATE
    assert "主臂控制端的用途" in web.HTML_TEMPLATE
    assert "用于控制仿真从臂" in web.HTML_TEMPLATE
    assert 'id="device-app-id" readonly' in web.HTML_TEMPLATE
    assert 'id="master-device-app-id" readonly' in web.HTML_TEMPLATE


def test_apply_snapshot_uses_one_machine_suffix_for_all_ids(monkeypatch: Any) -> None:
    monkeypatch.setattr(web, "machine_mac_suffix", lambda: "1234abcd")
    configs = {
        "follower_server": {
            "manifest": {},
            "integration": {},
            "components": {"robot": {}, "front_camera": {}, "wrist_camera": {}},
        },
        "leader_server": {"manifest": {}, "integration": {}, "components": {"robot": {}}},
        "rynnbot_app": {"app": {"app_id": "so101_rynnbot_app"}},
        "master_rynnbot_app": {"app": {"app_id": "so101_master_rynnbot_app"}},
    }
    snapshot = {
        "server": {"id": "so101_follower"},
        "hardware": {
            "follower_port": "/dev/follower",
            "leader_port": "/dev/leader",
            "front_camera": 0,
            "wrist_camera": 1,
        },
        "rynnbot": {
            "product_key": "",
            "device_name": "",
            "device_secret": "",
            "http_url": "https://example.com",
            "image_upload_codec": "jpeg",
        },
        "master_rynnbot": {
            "product_key": "",
            "device_name": "",
            "device_secret": "",
            "http_url": "https://example.com",
        },
    }

    web.apply_snapshot_to_configs(configs, snapshot)
    web.apply_snapshot_to_configs(configs, snapshot)

    assert configs["follower_server"]["manifest"]["robot_id"] == "so101_follower_1234abcd"
    assert configs["leader_server"]["manifest"]["robot_id"] == "so101_leader_1234abcd"
    assert configs["rynnbot_app"]["app"]["app_id"] == "so101_rynnbot_app_1234abcd"
    assert configs["master_rynnbot_app"]["app"]["app_id"] == "so101_master_rynnbot_app_1234abcd"


def test_build_snapshot_exposes_generated_app_ids(monkeypatch: Any) -> None:
    monkeypatch.setattr(web, "machine_mac_suffix", lambda: "1234abcd")
    configs = {
        "follower_server": {
            "manifest": {"robot_id": "so101_follower"},
            "components": {
                "robot": {"port": "/dev/follower"},
                "front_camera": {"device_id": 0},
                "wrist_camera": {"device_id": 1},
            },
        },
        "leader_server": {"components": {"robot": {"port": "/dev/leader"}}},
        "rynnbot_app": {"app": {"app_id": "so101_rynnbot_app"}},
        "master_rynnbot_app": {"app": {"app_id": "so101_master_rynnbot_app"}},
    }

    snapshot = web.build_snapshot(configs)

    assert snapshot["server"]["id"] == "so101_follower_1234abcd"
    assert snapshot["rynnbot"]["app_id"] == "so101_rynnbot_app_1234abcd"
    assert snapshot["master_rynnbot"]["app_id"] == "so101_master_rynnbot_app_1234abcd"


def test_dual_snapshot_writes_four_ports_three_cameras_and_unique_calibration_ids(
    monkeypatch: Any,
) -> None:
    monkeypatch.setattr(web, "machine_mac_suffix", lambda: "1234abcd")
    configs = web.load_all_configs("dual")
    snapshot = web.build_snapshot(configs, "dual")
    snapshot["server"]["id"] = "so101_bimanual_follower"
    snapshot["hardware"] = {
        "left_follower_port": "/dev/lf",
        "right_follower_port": "/dev/rf",
        "left_leader_port": "/dev/ll",
        "right_leader_port": "/dev/rl",
        "front_camera": 4,
        "left_wrist_camera": 5,
        "right_wrist_camera": 6,
    }

    web.apply_snapshot_to_configs(configs, snapshot)

    follower = configs["follower_server"]
    leader = configs["leader_server"]
    follower_robot = follower["components"]["robot"]
    leader_robot = leader["components"]["robot"]
    assert follower_robot["left_port"] == "/dev/lf"
    assert follower_robot["right_port"] == "/dev/rf"
    assert leader_robot["left_port"] == "/dev/ll"
    assert leader_robot["right_port"] == "/dev/rl"
    assert follower_robot["left_robot_id"].endswith("_left")
    assert follower_robot["right_robot_id"].endswith("_right")
    assert follower["components"]["front_camera"]["device_id"] == 4
    assert follower["components"]["left_wrist_camera"]["device_id"] == 5
    assert follower["components"]["right_wrist_camera"]["device_id"] == 6


def test_dual_calibration_args_select_each_physical_arm() -> None:
    configs = web.load_all_configs("dual")

    assert "--port=/dev/cu.usbmodem-left-follower" in web.calibration_args(
        "left_follower", configs, "dual"
    )
    assert "--port=/dev/cu.usbmodem-right-leader" in web.calibration_args(
        "right_leader", configs, "dual"
    )


def test_config_ui_contains_single_dual_selector_and_dual_task_keys() -> None:
    assert 'id="robot-profile"' in web.HTML_TEMPLATE
    assert "LeRobot SO101 Dual (12 DoF)" in web.HTML_TEMPLATE
    assert "observation.images.left_wrist" in web.HTML_TEMPLATE
    assert "observation.images.right_wrist" in web.HTML_TEMPLATE
