from __future__ import annotations

import shutil
from pathlib import Path
from typing import Any

import pytest

from rynnrcp_robot_franka_fr3 import configure_franka_fr3_web as web


SOURCE_CONFIG = (
    Path(__file__).resolve().parents[2]
    / "robots"
    / "franka_fr3"
    / "rynnrcp_robot_franka_fr3"
    / "config"
    / "franka_fr3_server.yaml"
)


class FakeController:
    instances: list["FakeController"] = []

    def __init__(self, **kwargs: Any) -> None:
        self.kwargs = kwargs
        self.positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        self.gripper_position = 1.0
        self.home_joint_positions = list(kwargs["home_joint_positions"])
        self.joint_lower_limits = list(kwargs["joint_lower_limits"])
        self.joint_upper_limits = list(kwargs["joint_upper_limits"])
        self.commands: list[list[float]] = []
        self.gripper_commands: list[float] = []
        self.started = False
        self.stopped = False
        self.__class__.instances.append(self)

    def start(self) -> None:
        self.started = True

    def shutdown(self) -> None:
        self.stopped = True

    def get_joint_positions(self) -> dict[str, list[float]]:
        return {"joint_positions": list(self.positions)}

    def set_joint_positions(self, value: dict[str, list[float]]) -> None:
        self.commands.append(list(value["joint_positions"]))

    def get_gripper_state(self) -> dict[str, float]:
        return {"position": self.gripper_position}

    def set_gripper(self, value: dict[str, float]) -> None:
        self.gripper_commands.append(float(value["position"]))

    def home(self, value: dict[str, Any] | None = None) -> None:
        assert value == {}
        self.commands.append(list(self.home_joint_positions))

    def get_health(self) -> dict[str, list[Any]]:
        return {"errors": [], "warnings": []}


class FakeCameraPreviews:
    def refresh(self) -> list[dict[str, Any]]:
        return [
            {
                "serial": "camera-001",
                "name": "Intel RealSense D435",
                "online": True,
                "ready": True,
                "error": "",
            },
            {
                "serial": "camera-002",
                "name": "Intel RealSense D435",
                "online": True,
                "ready": False,
                "error": "",
            },
        ]

    def frame(self, serial: str) -> bytes:
        if serial != "camera-001":
            raise KeyError(serial)
        return b"\xff\xd8preview\xff\xd9"


def _session(tmp_path: Path) -> web.ConfigureSession:
    config = tmp_path / "franka_fr3_server.yaml"
    shutil.copyfile(SOURCE_CONFIG, config)
    FakeController.instances.clear()
    return web.ConfigureSession(config, FakeController)


def test_connect_reads_state_without_sending_motion(tmp_path: Path) -> None:
    session = _session(tmp_path)

    status = session.connect()
    controller = FakeController.instances[-1]

    assert status["connected"] is True
    assert status["positions"] == [*controller.positions, controller.gripper_position]
    assert controller.started is True
    assert controller.commands == []


def test_single_joint_slider_preserves_other_targets(tmp_path: Path) -> None:
    session = _session(tmp_path)
    session.connect()
    controller = FakeController.instances[-1]

    session.set_joint(2, 0.01)
    session.set_joint(2, 0.02)

    assert controller.commands == [
        [0.0, -0.785, 0.01, -2.356, 0.0, 1.571, 0.785],
        [0.0, -0.785, 0.02, -2.356, 0.0, 1.571, 0.785],
    ]


def test_gripper_slider_uses_separate_action(tmp_path: Path) -> None:
    session = _session(tmp_path)
    session.connect()
    controller = FakeController.instances[-1]

    session.set_joint(7, 0.5)

    assert controller.commands == []
    assert controller.gripper_commands == [0.5]


def test_save_current_state_as_home(tmp_path: Path) -> None:
    session = _session(tmp_path)
    session.connect()
    controller = FakeController.instances[-1]
    controller.positions = [0.1, 0.2, 0.3, -1.4, 0.5, 1.6, 0.7]

    status = session.save_current_home()
    saved = web.load_server_config(session.config_path)

    assert status["home_joint_positions"] == controller.positions
    assert saved["components"]["robot"]["home_joint_positions"] == controller.positions
    assert controller.home_joint_positions == controller.positions


def test_go_home_sends_configured_target_and_updates_slider_base(tmp_path: Path) -> None:
    session = _session(tmp_path)
    session.connect()
    controller = FakeController.instances[-1]

    status = session.go_home()
    session.set_joint(2, 0.1)

    assert status["connected"] is True
    assert controller.commands == [
        controller.home_joint_positions,
        [0.0, -0.785, 0.1, -2.356, 0.0, 1.571, 0.785],
    ]


def test_connection_config_can_only_change_while_disconnected(tmp_path: Path) -> None:
    session = _session(tmp_path)

    session.save_config("172.16.0.2", True)
    snapshot = session.config_snapshot()
    assert snapshot["robot_ip"] == "172.16.0.2"
    assert snapshot["realtime_enforce"] is True
    assert set(snapshot["cameras"]) == set(web.CAMERA_NAMES)


def test_camera_serial_mapping_can_be_saved_while_disconnected(tmp_path: Path) -> None:
    session = _session(tmp_path)
    cameras = {
        name: {"enabled": name != "cam_side", "serial": f"serial-{name}"}
        for name in web.CAMERA_NAMES
    }

    session.save_config("192.168.0.110", True, cameras)
    snapshot = session.config_snapshot()

    assert snapshot["cameras"]["cam_arm"] == {
        "enabled": True,
        "serial": "serial-cam_arm",
    }
    assert snapshot["cameras"]["cam_side"] == {
        "enabled": False,
        "serial": "serial-cam_side",
    }


def test_enabled_camera_requires_a_serial(tmp_path: Path) -> None:
    session = _session(tmp_path)

    with pytest.raises(ValueError, match="cam_arm"):
        session.save_config(
            "192.168.0.110",
            True,
            {"cam_arm": {"enabled": True, "serial": ""}},
        )


def test_enabled_cameras_require_unique_serials(tmp_path: Path) -> None:
    session = _session(tmp_path)

    with pytest.raises(ValueError, match="不同"):
        session.save_config(
            "192.168.0.110",
            True,
            {
                "cam_arm": {"enabled": True, "serial": "same"},
                "cam_main": {"enabled": True, "serial": "same"},
            },
        )


def test_page_is_limited_to_state_single_joint_steps_and_home() -> None:
    assert "通过单关节滑块进行小步运动测试" in web.HTML_TEMPLATE
    assert "状态与单关节测试" in web.HTML_TEMPLATE
    assert "末端位姿测试" not in web.HTML_TEMPLATE
    assert "/api/ee-pose" not in web.HTML_TEMPLATE
    assert 'type="range"' in web.HTML_TEMPLATE
    assert "1000 / 60" in web.HTML_TEMPLATE
    assert "activeControlCommand = pendingControlCommand" in web.HTML_TEMPLATE
    assert "stopControlLoop();" in web.HTML_TEMPLATE
    assert "回到 Home" in web.HTML_TEMPLATE
    assert 'api("/api/home/go"' in web.HTML_TEMPLATE
    assert "将当前状态保存为 Home" in web.HTML_TEMPLATE
    assert all(name in web.HTML_TEMPLATE for name in web.CAMERA_NAMES)
    assert "RealSense 对应关系" in web.HTML_TEMPLATE
    assert "自动显示所有已连接 RealSense 的实时画面" in web.HTML_TEMPLATE
    assert 'onclick="refreshCameras()"' in web.HTML_TEMPLATE
    assert 'value="">未绑定</option>' in web.HTML_TEMPLATE
    assert "/api/cameras/refresh" in web.HTML_TEMPLATE
    assert "填写 RealSense 序列号" not in web.HTML_TEMPLATE
    assert "遥操" not in web.HTML_TEMPLATE


def test_config_page_and_snapshot_routes(tmp_path: Path) -> None:
    pytest.importorskip("flask")
    app = web.create_app(_session(tmp_path))
    client = app.test_client()

    page = client.get("/")
    snapshot = client.get("/api/config")

    assert page.status_code == 200
    assert "Franka FR3 配置".encode() in page.data
    assert snapshot.status_code == 200
    assert snapshot.get_json()["status"]["connected"] is False


def test_realsense_scan_and_preview_routes(tmp_path: Path) -> None:
    pytest.importorskip("flask")
    app = web.create_app(_session(tmp_path), FakeCameraPreviews())
    client = app.test_client()

    scan = client.post("/api/cameras/refresh", json={})
    preview = client.get("/api/cameras/camera-001/frame")
    missing = client.get("/api/cameras/missing/frame")

    assert scan.status_code == 200
    assert [item["serial"] for item in scan.get_json()["cameras"]] == [
        "camera-001",
        "camera-002",
    ]
    assert preview.status_code == 200
    assert preview.mimetype == "image/jpeg"
    assert preview.data == b"\xff\xd8preview\xff\xd9"
    assert missing.status_code == 404


def test_go_home_route(tmp_path: Path) -> None:
    pytest.importorskip("flask")
    session = _session(tmp_path)
    app = web.create_app(session)
    client = app.test_client()

    assert client.post("/api/connect", json={}).status_code == 200
    response = client.post("/api/home/go", json={})

    assert response.status_code == 200
    assert response.get_json()["ok"] is True
    assert FakeController.instances[-1].commands == [
        FakeController.instances[-1].home_joint_positions
    ]
