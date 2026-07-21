"""Camera-gesture master checks for Aero Hand."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from rynnrcp_robot_aero_hand.shared_camera import AeroHandSharedCamera
from rynnrcp_robot_aero_hand.vision_master import AeroHandVisionMaster, DEFAULT_MODEL_PATH


CONFIG_DIR = (
    Path(__file__).resolve().parents[2]
    / "robots"
    / "tetheria_aerohand"
    / "rynnrcp_robot_aero_hand"
    / "config"
)


def test_single_hand_master_publishes_only_configured_side() -> None:
    source = AeroHandVisionMaster(robot_id="single-master", mode="single", side="right")
    source._update_detected(
        {
            "left": [0.1] * 7,
            "right": [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],
        },
        seen_at=1.0,
    )

    assert source.get_joint_positions() == {
        "joint_positions": [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
    }


@pytest.mark.parametrize(
    ("mode", "expected_dof"),
    [("single", 7), ("dual", 14)],
)
def test_camera_master_publishes_open_hand_until_required_hands_are_detected(
    mode: str, expected_dof: int
) -> None:
    source = AeroHandVisionMaster(robot_id=f"{mode}-master", mode=mode)

    assert source.get_joint_positions() == {"joint_positions": [0.0] * expected_dof}


@pytest.mark.parametrize("detected_side", ["left", "right"])
def test_single_hand_master_auto_accepts_either_hand(detected_side: str) -> None:
    source = AeroHandVisionMaster(robot_id="single-master", mode="single", side="auto")
    source._update_detected({detected_side: [0.4] * 7}, seen_at=1.0)

    assert source.get_joint_positions() == {"joint_positions": [0.4] * 7}


def test_dual_hand_master_requires_both_hands_and_orders_left_then_right() -> None:
    source = AeroHandVisionMaster(robot_id="dual-master", mode="dual")
    source._update_detected({"right": [0.2] * 7}, seen_at=1.0)

    assert source.get_joint_positions() == {"joint_positions": [0.0] * 14}

    source._update_detected({"left": [0.1] * 7}, seen_at=2.0)

    assert source.get_joint_positions() == {"joint_positions": [0.1] * 7 + [0.2] * 7}


def test_camera_master_rejects_invalid_profile_values() -> None:
    with pytest.raises(ValueError, match="mode"):
        AeroHandVisionMaster(robot_id="bad", mode="three-hands")
    with pytest.raises(ValueError, match="side"):
        AeroHandVisionMaster(robot_id="bad", side="center")
    with pytest.raises(ValueError, match="visual"):
        AeroHandVisionMaster(robot_id="bad", visual="browser")


def test_camera_master_model_is_packaged() -> None:
    path = Path(DEFAULT_MODEL_PATH)

    assert path.is_file()
    assert path.stat().st_size == 7_819_105


def test_camera_master_exposes_pipeline_preview() -> None:
    class Pipeline:
        @staticmethod
        def get_preview_jpeg() -> bytes:
            return b"preview"

    source = AeroHandVisionMaster(robot_id="preview-master")
    source._pipeline = Pipeline()

    assert source.get_preview_jpeg() == b"preview"


def test_camera_master_exposes_unannotated_frame_for_local_collection() -> None:
    source = AeroHandVisionMaster(robot_id="shared-frame-master")
    source._update_camera_frame(b"raw-jpeg", 640, 360)

    assert source.get_camera_frame() == (b"raw-jpeg", 640, 360)


def test_shared_camera_reads_master_frame_without_opening_a_device(monkeypatch: pytest.MonkeyPatch) -> None:
    class Response:
        status = 200
        headers = {"X-Image-Width": "1280", "X-Image-Height": "720"}

        def __enter__(self) -> "Response":
            return self

        def __exit__(self, *args: object) -> None:
            return None

        @staticmethod
        def read() -> bytes:
            return b"jpeg"

    monkeypatch.setattr(
        "rynnrcp_robot_aero_hand.shared_camera.urlopen",
        lambda *args, **kwargs: Response(),
    )
    monkeypatch.setattr(
        "rynnrcp_robot_aero_hand.shared_camera.try_start_frame_server",
        lambda *args, **kwargs: None,
    )
    camera = AeroHandSharedCamera(name="front")
    camera.start()

    assert camera.read() == (True, 1280, 720, "jpg", b"jpeg")

    camera.stop()
    assert camera.read() == (False, 0, 0, "jpg", None)


def test_shared_camera_promotes_itself_when_current_owner_disappears(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    camera = AeroHandSharedCamera(name="front")
    camera._running = True
    monkeypatch.setattr(
        "rynnrcp_robot_aero_hand.shared_camera.fetch_shared_frame",
        lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("owner offline")),
    )
    monkeypatch.setattr(camera, "_try_become_owner", lambda: True)
    monkeypatch.setattr(camera, "_read_owned_frame", lambda: (b"promoted", 640, 360))

    assert camera.read() == (True, 640, 360, "jpg", b"promoted")


@pytest.mark.parametrize("profile", ["single", "dual"])
def test_hand_master_rynnbot_profile_is_a_controller_source(profile: str) -> None:
    config = yaml.safe_load(
        (CONFIG_DIR / f"aero_hand_{profile}_hand_master_rynnbot_app.yaml").read_text(
            encoding="utf-8"
        )
    )

    assert config["app"]["role"] == "controller"
    assert config["master_source"] == {
        "enabled": True,
        "observation": "observation.robot.joint_state",
        "action_name": "action",
        "idle_hz": 5,
        "active_hz": 30,
    }
