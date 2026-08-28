"""Camera-gesture master checks for Aero Hand."""

from __future__ import annotations

import threading
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from rynnrcp_robot_aero_hand import gesture_inference, vision_master
from rynnrcp_robot_aero_hand.shared_camera import AeroHandSharedCamera
from rynnrcp_robot_aero_hand.gesture_inference import (
    MediaPipeLiteBackend,
    create_gesture_backend,
    register_gesture_backend,
)
from rynnrcp_robot_aero_hand.vision_master import (
    AeroHandVisionMaster,
    DEFAULT_MODEL_PATH,
    _VisionPipeline,
)


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
    for visual in ("browser", "window"):
        with pytest.raises(ValueError, match="web preview"):
            AeroHandVisionMaster(robot_id="bad", visual=visual)


def test_camera_master_task_model_path_is_legacy() -> None:
    # hand_landmarker.task is no longer packaged (weights moved to the gitee
    # model zoo); the pluggable gesture backends do not require it, so only
    # the conventional location is pinned here.
    path = Path(DEFAULT_MODEL_PATH)

    assert path.name == "hand_landmarker.task"
    assert path.parent.name == "model"


def test_camera_master_uses_lite_backend_without_reducing_capture_quality() -> None:
    source = AeroHandVisionMaster(robot_id="optimized-master", fps=30.0)

    assert source.fps == 30.0
    assert source.inference_fps == 15.0
    # "auto" resolves to the best platform backend at pipeline start and
    # falls back to mediapipe_lite on generic hosts.
    assert source.inference_backend == "auto"
    assert source.inference_threads == 1
    assert (source.camera_width, source.camera_height) == (640, 480)
    assert source.inference_width == 640


def test_platform_environment_selects_backend_for_configure_and_server(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("AERO_HAND_INFERENCE_BACKEND", "device_accelerator")
    monkeypatch.setenv(
        "AERO_HAND_INFERENCE_BACKEND_MODULE",
        "accelerators.device",
    )
    monkeypatch.setenv("AERO_HAND_INFERENCE_THREADS", "2")

    source = AeroHandVisionMaster(
        robot_id="platform-master",
        inference_backend="mediapipe_lite",
        inference_backend_module=None,
        inference_threads=1,
    )

    assert source.inference_backend == "device_accelerator"
    assert source.inference_backend_module == "accelerators.device"
    assert source.inference_threads == 2


def test_platform_environment_can_clear_backend_module(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("AERO_HAND_INFERENCE_BACKEND_MODULE", "")

    source = AeroHandVisionMaster(
        robot_id="portable-master",
        inference_backend_module="accelerators.device",
    )

    assert source.inference_backend_module is None


def test_camera_master_throttles_inference_to_configured_rate() -> None:
    pipeline = object.__new__(_VisionPipeline)
    pipeline.source = SimpleNamespace(inference_fps=15.0)
    pipeline._next_inference_at = 0.0

    assert pipeline._take_inference_slot(10.0) is True
    assert pipeline._take_inference_slot(10.05) is False
    assert pipeline._take_inference_slot(10.067) is True


def test_legacy_result_processing_stays_on_vision_pipeline() -> None:
    assert hasattr(_VisionPipeline, "_select_legacy_per_side")
    assert not hasattr(MediaPipeLiteBackend, "_select_legacy_per_side")


def test_gesture_backend_registry_is_an_acceleration_extension_point() -> None:
    sentinel = object()

    register_gesture_backend("test_accelerator", lambda **_kwargs: sentinel)

    assert (
        create_gesture_backend(
            "test_accelerator",
            mediapipe=object(),
            max_num_hands=1,
            num_threads=1,
        )
        is sentinel
    )


def test_unknown_gesture_backend_has_a_clear_error() -> None:
    with pytest.raises(ValueError, match="unknown gesture inference backend"):
        create_gesture_backend(
            "missing",
            mediapipe=object(),
            max_num_hands=1,
            num_threads=1,
        )


def test_external_gesture_backend_module_is_loaded_before_lookup(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    sentinel = object()

    def import_module(name: str) -> None:
        assert name == "accelerators.example"
        register_gesture_backend("loaded_accelerator", lambda **_kwargs: sentinel)

    monkeypatch.setattr(gesture_inference.importlib, "import_module", import_module)

    assert (
        create_gesture_backend(
            "loaded_accelerator",
            module_name="accelerators.example",
            mediapipe=object(),
            max_num_hands=1,
            num_threads=1,
        )
        is sentinel
    )


def test_vision_pipeline_passes_backend_module_to_shared_factory(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    pipeline = object.__new__(_VisionPipeline)
    pipeline.mp = object()
    pipeline.source = SimpleNamespace(
        mode="single",
        inference_backend="accelerated",
        inference_backend_module="accelerators.device",
        inference_threads=2,
    )
    calls: list[dict[str, object]] = []
    backend = SimpleNamespace(connections=("connection",))

    def create(name: str, **kwargs: object) -> object:
        calls.append({"name": name, **kwargs})
        return backend

    monkeypatch.setattr(vision_master, "create_gesture_backend", create)

    pipeline._create_landmarker()

    assert pipeline.backend is backend
    assert pipeline._connections == ("connection",)
    assert calls == [
        {
            "name": "accelerated",
            "module_name": "accelerators.device",
            "mediapipe": pipeline.mp,
            "max_num_hands": 1,
            "num_threads": 2,
        }
    ]


def test_camera_capture_queue_keeps_only_the_latest_frame() -> None:
    pipeline = object.__new__(_VisionPipeline)
    pipeline._frame_condition = threading.Condition()
    pipeline._latest_frame = None
    pipeline._latest_frame_sequence = 0
    pipeline._latest_frame_at = 0.0
    pipeline._last_inference_sequence = 0
    pipeline._capture_count = 0
    pipeline._capture_error = None
    pipeline._metrics_lock = threading.Lock()
    pipeline._capture_times = deque()
    pipeline._inference_samples = deque()
    pipeline.source = SimpleNamespace(fps=30.0)

    pipeline._publish_captured_frame("old", captured_at=1.0)
    pipeline._publish_captured_frame("latest", captured_at=2.0)

    assert pipeline._wait_for_latest_frame() == ("latest", 2, 2.0)


@pytest.mark.parametrize(
    "config_name",
    [
        "aero_hand_single_hand_master_server.yaml",
        "aero_hand_dual_hand_master_server.yaml",
    ],
)
def test_camera_master_config_keeps_capture_quality_while_reducing_inference(
    config_name: str,
) -> None:
    config = yaml.safe_load((CONFIG_DIR / config_name).read_text(encoding="utf-8"))
    robot = config["components"]["robot"]

    assert (robot["camera_width"], robot["camera_height"], robot["fps"]) == (640, 480, 30.0)
    assert (robot["inference_width"], robot["inference_fps"]) == (640, 15.0)
    assert robot["inference_backend"] == "auto"
    assert robot["inference_backend_module"] == ""
    # "auto" resolves to the platform accelerator (or the CPU baseline) at
    # pipeline start, so the configs keep the portable single-thread default.
    assert robot["inference_threads"] == 1
    assert "hand_landmarker_model" not in robot


def test_camera_master_exposes_pipeline_preview() -> None:
    class Pipeline:
        @staticmethod
        def get_preview_jpeg() -> bytes:
            return b"preview"

    source = AeroHandVisionMaster(robot_id="preview-master")
    source._pipeline = Pipeline()

    assert source.get_preview_jpeg() == b"preview"


def test_pipeline_preview_falls_back_to_raw_frame_when_rendering_is_disabled() -> None:
    pipeline = object.__new__(_VisionPipeline)
    pipeline.source = SimpleNamespace(render_preview=False)
    pipeline.get_camera_frame_jpeg = lambda: (b"raw-preview", 640, 480)

    assert pipeline.get_preview_jpeg() == b"raw-preview"


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
