from __future__ import annotations

import shutil
import sys
from pathlib import Path
from typing import Any

import pytest


ROBOT_ROOT = Path(__file__).resolve().parents[2] / "robots" / "astribot_s1"
sys.path.insert(0, str(ROBOT_ROOT))

from rynnrcp_robot_astribot_s1 import configure_astribot_s1_web as web  # noqa: E402


SOURCE_CONFIG = (
    ROBOT_ROOT
    / "rynnrcp_robot_astribot_s1"
    / "config"
    / "astribot_s1_server.yaml"
)
SOURCE_RYNNBOT_CONFIG = (
    ROBOT_ROOT
    / "rynnrcp_robot_astribot_s1"
    / "config"
    / "astribot_s1_rynnbot_app.yaml"
)


class FakeController:
    instances: list["FakeController"] = []

    def __init__(self, **kwargs: Any) -> None:
        self.kwargs = kwargs
        self.positions = [index / 100 for index in range(22)]
        self.velocities = [0.0] * 22
        self.single_joint_commands: list[tuple[int, float]] = []
        self.left_gripper_commands: list[float] = []
        self.right_gripper_commands: list[float] = []
        self.started = False
        self.stopped = False
        self.stop_calls = 0
        self.clear_command_calls = 0
        self.control_rights = bool(kwargs.get("high_control_rights", False))
        self.__class__.instances.append(self)

    def start(self) -> None:
        self.started = True

    def shutdown(self) -> None:
        self.stopped = True

    def get_joint_positions(self) -> dict[str, list[float]]:
        return {
            "joint_positions": list(self.positions),
            "joint_velocities": list(self.velocities),
        }

    def get_joint_limits(self) -> dict[str, list[float]]:
        return {"lower": [-2.0] * 22, "upper": [2.0] * 22}

    def get_left_gripper_state(self) -> dict[str, float]:
        return {
            "position": 0.25,
            "velocity": 0.0,
            "sdk_joint_position": 25.0,
            "sdk_joint_velocity": 0.0,
        }

    def get_right_gripper_state(self) -> dict[str, float]:
        return {
            "position": 0.75,
            "velocity": 0.0,
            "sdk_joint_position": 75.0,
            "sdk_joint_velocity": 0.0,
        }

    def get_chassis_state(self) -> dict[str, list[float]]:
        return {
            "position_xy_yaw": [0.0, 0.0, 0.0],
            "velocity_xy_yaw": [0.0, 0.0, 0.0],
        }

    def get_health(self) -> dict[str, list[Any]]:
        return {"errors": [], "warnings": []}

    def has_control_rights(self) -> bool:
        return self.control_rights

    def set_single_joint_position(self, index: int, value: float) -> None:
        self.single_joint_commands.append((index, value))
        self.positions[index] = value

    def set_left_gripper(self, value: dict[str, float]) -> None:
        self.left_gripper_commands.append(value["position"])

    def set_right_gripper(self, value: dict[str, float]) -> None:
        self.right_gripper_commands.append(value["position"])

    def stop(self, value: dict[str, Any]) -> None:
        assert value == {}
        self.stop_calls += 1

    def clear_command(self) -> None:
        self.clear_command_calls += 1


def _session(tmp_path: Path) -> web.ConfigureSession:
    config = tmp_path / "astribot_s1_server.yaml"
    rynnbot_config = tmp_path / "astribot_s1_rynnbot_app.yaml"
    shutil.copyfile(SOURCE_CONFIG, config)
    shutil.copyfile(SOURCE_RYNNBOT_CONFIG, rynnbot_config)
    FakeController.instances.clear()
    return web.ConfigureSession(config, FakeController, rynnbot_config)


def test_connect_reads_22_dof_state_without_sending_motion(tmp_path: Path) -> None:
    session = _session(tmp_path)

    status = session.connect()
    controller = FakeController.instances[-1]

    assert status["connected"] is True
    assert status["motion_enabled"] is False
    assert status["has_control_rights"] is False
    assert len(status["positions"]) == 22
    assert status["gripper_indices"] == [11, 19]
    assert status["left_gripper"]["position"] == 0.11
    assert controller.started is True
    assert controller.single_joint_commands == []


def test_motion_requires_confirmation_and_accepts_latest_joint_target(tmp_path: Path) -> None:
    session = _session(tmp_path)
    session.connect()
    controller = FakeController.instances[-1]

    with pytest.raises(RuntimeError, match="解锁"):
        session.set_joint(4, 1.0)
    with pytest.raises(ValueError, match="现场安全"):
        session.set_motion_enabled(True, "wrong")

    session.set_motion_enabled(True, web.CONTROL_CONFIRMATION)
    controller = FakeController.instances[-1]
    session.set_joint(4, 1.0)

    assert controller.single_joint_commands == [(4, pytest.approx(1.0))]
    assert controller.kwargs["control_rights_mode"] == "force"


def test_grippers_use_normalized_dedicated_commands(tmp_path: Path) -> None:
    session = _session(tmp_path)
    session.connect()
    session.set_motion_enabled(True, web.CONTROL_CONFIRMATION)
    controller = FakeController.instances[-1]

    session.set_gripper("left", 0.25)
    session.set_gripper("right", 0.75)

    assert controller.left_gripper_commands == [0.25]
    assert controller.right_gripper_commands == [0.75]
    assert controller.single_joint_commands == []


def test_stop_is_available_and_locks_motion(tmp_path: Path) -> None:
    session = _session(tmp_path)
    session.connect()
    session.set_motion_enabled(True, web.CONTROL_CONFIRMATION)
    controller = FakeController.instances[-1]

    status = session.stop()

    assert controller.stop_calls == 1
    assert status["motion_enabled"] is False


def test_connection_config_can_only_change_while_disconnected(tmp_path: Path) -> None:
    session = _session(tmp_path)
    sdk_root = tmp_path / "sdk"

    session.save_config(str(sdk_root), True)
    assert session.config_snapshot() == {
        "sdk_root": str(sdk_root),
        "high_control_rights": True,
        "rynnbot": {
            "app_id": "astribot_s1_rynnbot_app",
            "product_key": "YOUR_PRODUCT_KEY",
            "device_name": "YOUR_DEVICE_NAME",
            "device_secret": "YOUR_DEVICE_SECRET",
            "http_url": "https://robot-access.damo-academy.com",
            "image_upload_codec": "jpeg",
        },
    }

    session.connect()
    with pytest.raises(RuntimeError, match="断开"):
        session.save_config(str(sdk_root), False)


def test_page_and_routes_expose_safe_connect_read_and_control(tmp_path: Path) -> None:
    pytest.importorskip("flask")
    session = _session(tmp_path)
    app = web.create_app(session)
    client = app.test_client()

    page = client.get("/")
    assert page.status_code == 200
    assert "Astribot S1 配置".encode() in page.data
    assert "连接并读取状态（不抢占）".encode() in page.data
    assert "接管并解锁控制".encode() in page.data
    assert "停止运动".encode() in page.data
    assert 'id="joint-slider-${index}"'.encode() in page.data
    assert "document.activeElement !== slider".encode() in page.data
    assert 'oninput="queueJoint(${index},this.value)"'.encode() in page.data
    assert 'oninput="queueGripper'.encode() in page.data
    assert "const COMMAND_INTERVAL_MS = 33".encode() in page.data
    assert "RynnBot 云端配置".encode() in page.data

    response = client.post("/api/connect", json={})
    assert response.status_code == 200
    assert response.get_json()["status"]["motion_enabled"] is False
    assert response.get_json()["status"]["has_control_rights"] is False

    denied = client.post("/api/joint", json={"index": 4, "value": 0.5})
    assert denied.status_code == 400

    unlocked = client.post(
        "/api/control",
        json={"enabled": True, "confirmation": web.CONTROL_CONFIRMATION},
    )
    assert unlocked.status_code == 200
    assert unlocked.get_json()["status"]["has_control_rights"] is True

    moved = client.post("/api/joint", json={"index": 4, "value": 0.5})
    assert moved.status_code == 200
    assert moved.get_json()["command"] == {"index": 4, "target": 0.5}
    assert "status" not in moved.get_json()
    assert FakeController.instances[-1].single_joint_commands


def test_config_page_saves_rynnbot_credentials(tmp_path: Path) -> None:
    session = _session(tmp_path)

    session.save_config(
        "/home/astribot/astribot_sdk_aarch64",
        False,
        {
            "product_key": "pk",
            "device_name": "s1",
            "device_secret": "secret",
            "http_url": "https://example.test",
        },
    )

    app = session.config_snapshot()["rynnbot"]
    assert app["app_id"] == "astribot_s1_rynnbot_app"
    assert app["product_key"] == "pk"
    assert app["device_name"] == "s1"
    assert app["device_secret"] == "secret"
    assert app["http_url"] == "https://example.test"


def test_control_request_is_rejected_after_rights_are_lost(tmp_path: Path) -> None:
    session = _session(tmp_path)
    session.connect()
    session.set_motion_enabled(True, web.CONTROL_CONFIRMATION)
    controller = FakeController.instances[-1]
    controller.control_rights = False

    with pytest.raises(RuntimeError, match="控制权已转移"):
        session.set_joint(4, 0.5)

    assert session.status()["motion_enabled"] is False
    assert controller.single_joint_commands == []
    assert controller.clear_command_calls >= 1


def test_locking_motion_clears_the_streaming_command(tmp_path: Path) -> None:
    session = _session(tmp_path)
    session.connect()
    session.set_motion_enabled(True, web.CONTROL_CONFIRMATION)
    controller = FakeController.instances[-1]

    status = session.set_motion_enabled(False)

    assert status["motion_enabled"] is False
    assert controller.clear_command_calls == 1
