"""Camera-gesture master source for Aero Hand teleoperation.

The MediaPipe detection and retargeting pipeline follows
AGI-Robot/aero_hand_control at commit d3da2ac36f26ad1dfb98eab8e29402cb4dfd0d76.
It exposes the camera operator as an RCP joint-state source: 7 radians for a
single hand, or left 7 + right 7 radians for a dual-hand profile.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

from rynnrcp.robot.base_controller import BaseRobotController

from .gesture_inference import (
    DEFAULT_GESTURE_BACKEND,
    create_gesture_backend,
)
from .shared_camera import fetch_shared_frame, stop_frame_server, try_start_frame_server


PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_MODEL_PATH = PACKAGE_DIR / "model" / "hand_landmarker.task"
DEFAULT_NORMALIZE_CONFIG = PACKAGE_DIR / "config" / "normalize_default_mediapipe.yaml"
HAND_ORDER = ("left", "right")
COMPACT_JOINT_INDICES = (0, 1, 2, 4, 7, 10, 13)
JOINT_UPPER_LIMITS_DEG = (100.0, 55.0) + (90.0,) * 14
_WORKER_JOIN_TIMEOUT_S = 2.0
_WORKER_ERROR_LOG_INTERVAL_S = 5.0
_BACKEND_ENV = "AERO_HAND_INFERENCE_BACKEND"
_BACKEND_MODULE_ENV = "AERO_HAND_INFERENCE_BACKEND_MODULE"
_THREADS_ENV = "AERO_HAND_INFERENCE_THREADS"


def letterbox_frame(cv2_module, np_module, image, target_width, target_height):
    """Aspect-preserving letterbox to a fixed canvas (centered, zero pad).

    Used for backends that declare ``accepts_bgr`` (hardware-preprocessing
    backends such as ``rga_rknn_zero``): the RGA path requires a CONSTANT
    frame geometry (stable stride/alignment and a stable ptr->handle import
    cache). A plain width-only resize would yield e.g. 640x360 for a 16:9
    camera; letterboxing keeps every camera at exactly the deployment preset
    (640x480) without distorting content.
    """
    height, width = image.shape[:2]
    if width == target_width and height == target_height:
        return image
    scale = min(target_width / width, target_height / height)
    new_width = max(1, round(width * scale))
    new_height = max(1, round(height * scale))
    resized = cv2_module.resize(image, (new_width, new_height))
    canvas = np_module.zeros((target_height, target_width, 3), dtype=image.dtype)
    x_offset = (target_width - new_width) // 2
    y_offset = (target_height - new_height) // 2
    canvas[y_offset:y_offset + new_height, x_offset:x_offset + new_width] = resized
    return canvas


class AeroHandVisionMaster(BaseRobotController):
    """Publish camera-recognized hand gestures as a joint-state observation."""

    def __init__(
        self,
        robot_id: str,
        mode: str = "single",
        side: str = "auto",
        camera_index: int = 0,
        fps: float = 30.0,
        inference_fps: float = 15.0,
        inference_backend: str = DEFAULT_GESTURE_BACKEND,
        inference_backend_module: str | None = None,
        inference_threads: int = 1,
        prefer_platform_acceleration: bool = False,
        swap_handedness: bool = True,
        camera_width: int | None = 640,
        camera_height: int | None = 480,
        inference_width: int = 640,
        frame_server_port: int = 28412,
        min_hand_area: float = 0.005,
        ema_alpha: float = 0.7,
        min_cutoff: float = 5.0,
        beta: float = 1.3,
        d_cutoff: float = 1.4,
        visual: str = "none",
        render_preview: bool = False,
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__(logger=logger)
        self.robot_id = str(robot_id)
        self.mode = _validate_mode(mode)
        self.side = _validate_side(side)
        self.camera_index = int(camera_index)
        self.fps = max(1.0, float(fps))
        self.prefer_platform_acceleration = bool(prefer_platform_acceleration) and not (
            _BACKEND_ENV in os.environ or _BACKEND_MODULE_ENV in os.environ
        )
        requested_inference_fps = max(1.0, float(inference_fps))
        self.inference_fps = min(self.fps, requested_inference_fps)
        selected_backend = os.environ.get(_BACKEND_ENV, inference_backend)
        selected_module = (
            os.environ[_BACKEND_MODULE_ENV]
            if _BACKEND_MODULE_ENV in os.environ
            else inference_backend_module
        )
        selected_threads = os.environ.get(_THREADS_ENV, inference_threads)
        self.inference_backend = str(selected_backend).strip().lower()
        if not self.inference_backend:
            raise ValueError("inference_backend must not be empty")
        self.inference_backend_module = str(selected_module or "").strip() or None
        self.inference_threads = (
            _optional_positive_int(selected_threads, "inference_threads") or 1
        )
        self.swap_handedness = bool(swap_handedness)
        self.camera_width = _optional_positive_int(camera_width, "camera_width")
        self.camera_height = _optional_positive_int(camera_height, "camera_height")
        self.inference_width = (
            _optional_positive_int(inference_width, "inference_width") or 640
        )
        self.frame_server_port = int(frame_server_port)
        if not 0 <= self.frame_server_port <= 65535:
            raise ValueError("frame_server_port must be between 0 and 65535")
        self.min_hand_area = max(0.0, float(min_hand_area))
        self.ema_alpha = min(1.0, max(0.0, float(ema_alpha)))
        self.min_cutoff = max(1e-6, float(min_cutoff))
        self.beta = max(0.0, float(beta))
        self.d_cutoff = max(1e-6, float(d_cutoff))
        self.visual = _validate_visual(visual)
        self.render_preview = bool(render_preview)
        self._pipeline: _VisionPipeline | Any | None = None
        self._positions_by_side: dict[str, list[float]] = {}
        self._last_seen_by_side: dict[str, float] = {}
        self._selected_side: str | None = None
        self._last_error: dict[str, Any] | None = None
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._frame_server: Any | None = None
        self._frame_server_thread: threading.Thread | None = None
        self._owns_camera = False
        self._next_owner_attempt_at = 0.0
        self._camera_frame: tuple[bytes, int, int] | None = None

    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._start_frame_server()
            try:
                pipeline = _create_pipeline(self)
            except Exception:
                self._stop_frame_server()
                raise
            self._pipeline = pipeline
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._worker_loop,
                name=f"aero-hand-vision-master-{self.robot_id}",
                daemon=True,
            )
            self._thread.start()

    def shutdown(self) -> None:
        self._stop.set()
        self._stop_frame_server()
        with self._lock:
            thread = self._thread
            pipeline = self._pipeline
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=_WORKER_JOIN_TIMEOUT_S)
        if pipeline is not None:
            try:
                pipeline.close()
            except Exception as exc:
                self.logger.warning("Aero Hand vision pipeline close failed: %s", exc)
        with self._lock:
            self._thread = None
            self._pipeline = None

    def get_joint_positions(self) -> dict[str, list[float]]:
        with self._lock:
            if self.mode == "single":
                if self._selected_side is None:
                    positions = [0.0] * len(COMPACT_JOINT_INDICES)
                else:
                    positions = list(self._positions_by_side[self._selected_side])
            else:
                missing = [side for side in HAND_ORDER if side not in self._positions_by_side]
                if missing:
                    positions = [0.0] * (len(HAND_ORDER) * len(COMPACT_JOINT_INDICES))
                else:
                    positions = [value for side in HAND_ORDER for value in self._positions_by_side[side]]
        return {"joint_positions": positions}

    def get_health(self) -> dict[str, Any]:
        warnings: list[dict[str, Any]] = []
        with self._lock:
            thread_alive = self._thread is not None and self._thread.is_alive()
            missing = (
                [side for side in HAND_ORDER if side not in self._positions_by_side]
                if self.mode == "dual"
                else (["hand"] if self._selected_side is None else [])
            )
            health_sides = HAND_ORDER if self.mode == "dual" else ((self._selected_side,) if self._selected_side else ())
            stale = [
                side
                for side in health_sides
                if (seen_at := self._last_seen_by_side.get(side)) is not None
                if time.monotonic() - seen_at > 1.0
            ]
            last_error = dict(self._last_error) if self._last_error else None
        if not thread_alive:
            warnings.append(_warning("aero_hand.vision_not_running", "Camera gesture source is not running"))
        if missing:
            warnings.append(
                _warning(
                    "aero_hand.hands_not_detected",
                    "Required operator hand gestures have not been detected",
                    {"missing_sides": missing},
                )
            )
        if stale:
            warnings.append(
                _warning(
                    "aero_hand.hand_gesture_stale",
                    "Operator hand gesture is stale; the last valid pose is held",
                    {"stale_sides": stale},
                )
            )
        if last_error:
            warnings.append(_warning("aero_hand.vision_error", "Camera gesture pipeline failed", last_error))
        return {"errors": [], "warnings": warnings}

    def get_preview_jpeg(self) -> bytes:
        with self._lock:
            pipeline = self._pipeline
        if pipeline is None:
            raise RuntimeError("camera gesture source is not running")
        return pipeline.get_preview_jpeg()

    def get_camera_frame(self) -> tuple[bytes, int, int]:
        """Return the latest unannotated camera frame for local data collection."""
        with self._lock:
            pipeline = self._pipeline
            cached = self._camera_frame
        if pipeline is not None and hasattr(pipeline, "get_camera_frame_jpeg"):
            return pipeline.get_camera_frame_jpeg()
        if cached is None:
            raise RuntimeError("camera has not produced a frame")
        data, width, height = cached
        return data, width, height

    def get_performance(self) -> dict[str, Any]:
        """Return live capture and inference timing without touching the camera."""
        with self._lock:
            pipeline = self._pipeline
        if pipeline is None or not hasattr(pipeline, "get_performance"):
            return {
                "capture_target_fps": self.fps,
                "inference_target_fps": self.inference_fps,
                "capture_fps": 0.0,
                "inference_fps": 0.0,
                "latest_frame_age_ms": None,
                "latest_result_age_ms": None,
            }
        return pipeline.get_performance()

    def _update_camera_frame(self, data: bytes, width: int, height: int) -> None:
        with self._lock:
            self._camera_frame = (bytes(data), int(width), int(height))

    def _start_frame_server(self) -> None:
        if self._frame_server is not None or self._owns_camera:
            return
        owned = try_start_frame_server(self.frame_server_port, self.get_camera_frame)
        self._owns_camera = owned is not None
        if owned is not None:
            self._frame_server, self._frame_server_thread = owned

    def _stop_frame_server(self) -> None:
        server = self._frame_server
        thread = self._frame_server_thread
        self._frame_server = None
        self._frame_server_thread = None
        self._owns_camera = False
        stop_frame_server(server, thread)

    def _try_become_camera_owner(self) -> bool:
        if self._owns_camera:
            return True
        now = time.monotonic()
        if now < self._next_owner_attempt_at:
            return False
        self._next_owner_attempt_at = now + 1.0
        self._start_frame_server()
        return self._owns_camera

    def _detectable_sides(self) -> tuple[str, ...]:
        return HAND_ORDER if self.mode == "dual" or self.side == "auto" else (self.side,)

    def _worker_loop(self) -> None:
        period_s = 1.0 / self.inference_fps
        next_tick = time.monotonic()
        last_error_log = 0.0
        while not self._stop.is_set():
            try:
                pipeline = self._pipeline
                if pipeline is None:
                    return
                detected = pipeline.read()
                self._update_detected(detected)
                next_tick += period_s
                delay_s = max(0.0, next_tick - time.monotonic())
                if self._stop.wait(delay_s):
                    return
                if delay_s <= 0.0:
                    next_tick = time.monotonic()
            except Exception as exc:
                with self._lock:
                    self._last_error = {
                        "error": str(exc),
                        "error_type": type(exc).__name__,
                        "timestamp": time.time(),
                    }
                now = time.monotonic()
                if now - last_error_log >= _WORKER_ERROR_LOG_INTERVAL_S:
                    self.logger.warning("Aero Hand camera gesture read failed: %s", exc)
                    last_error_log = now
                self._stop.wait(0.1)
                next_tick = time.monotonic()

    def _update_detected(self, detected: dict[str, Any], *, seen_at: float | None = None) -> None:
        now = time.monotonic() if seen_at is None else float(seen_at)
        with self._lock:
            valid = [
                (side, _validate_compact_positions(positions))
                for side, positions in detected.items()
                if side in self._detectable_sides()
            ]
            if self.mode == "single" and valid:
                side, positions = valid[0]
                self._positions_by_side = {side: positions}
                self._last_seen_by_side = {side: now}
                self._selected_side = side
            elif self.mode == "dual":
                for side, positions in valid:
                    self._positions_by_side[side] = positions
                    self._last_seen_by_side[side] = now
            self._last_error = None


class _VisionPipeline:
    """Gesture camera reader and Aero Hand compact-joint retargeter."""

    def __init__(self, source: AeroHandVisionMaster) -> None:
        try:
            import cv2
            import mediapipe as mp
            import numpy as np
            import yaml
        except ImportError as exc:
            raise RuntimeError(
                "Camera gesture control dependencies are missing; install Aero Hand with "
                "setup_aero_hand.sh"
            ) from exc

        normalize_config = yaml.safe_load(DEFAULT_NORMALIZE_CONFIG.read_text(encoding="utf-8"))
        if not isinstance(normalize_config, dict):
            raise ValueError(f"Invalid normalization config: {DEFAULT_NORMALIZE_CONFIG}")

        self.cv2 = cv2
        self.mp = mp
        self.np = np
        self.source = source
        self.normalize_config = normalize_config
        self._connections: Any = ()
        self._filters = {
            side: _OneEuroFilterND(
                np,
                source.inference_fps,
                source.min_cutoff,
                source.beta,
                source.d_cutoff,
            )
            for side in HAND_ORDER
        }
        self._ema_cache = {side: np.zeros((25, 3), dtype=np.float32) for side in HAND_ORDER}
        self._preview_lock = threading.Lock()
        self._preview_frame: Any | None = None
        self._create_landmarker()
        self.capture = None
        if source._owns_camera:
            self._open_capture()
        self._next_inference_at = 0.0
        self._frame_condition = threading.Condition()
        self._latest_frame: Any | None = None
        self._latest_frame_sequence = 0
        self._latest_frame_at = 0.0
        self._last_inference_sequence = 0
        self._capture_error: Exception | None = None
        self._capture_stop = threading.Event()
        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            name=f"aero-hand-camera-capture-{source.robot_id}",
            daemon=True,
        )
        self._started_at = time.monotonic()
        self._capture_count = 0
        self._inference_count = 0
        self._inference_time_s = 0.0
        self._latest_result_at = 0.0
        self._metrics_lock = threading.Lock()
        self._capture_times: deque[float] = deque()
        self._inference_samples: deque[tuple[float, float]] = deque()
        self._capture_thread.start()

    def _create_landmarker(self) -> None:
        num_hands = 1 if self.source.mode == "single" else 2
        use_platform_backend = bool(
            getattr(self.source, "prefer_platform_acceleration", False)
        )
        self.backend = create_gesture_backend(
            DEFAULT_GESTURE_BACKEND
            if use_platform_backend
            else self.source.inference_backend,
            module_name=None
            if use_platform_backend
            else self.source.inference_backend_module,
            mediapipe=self.mp,
            max_num_hands=num_hands,
            num_threads=self.source.inference_threads,
        )
        self._connections = self.backend.connections
        # Hardware-preprocessing backends take raw BGR (color conversion is
        # done on the RGA during letterbox) via process_bgr().
        self._backend_takes_bgr = bool(
            getattr(self.backend, "accepts_bgr", False)
        ) and hasattr(self.backend, "process_bgr")

    def _prepare_inference_image(self, frame_bgr: Any) -> tuple[Any, Any]:
        """Mirror and convert one frame; override here for backend preprocessing.

        Two paths:
        - default backends: aspect resize to ``inference_width`` + BGR->RGB
          (color conversion on the CPU, MediaPipe letterboxes internally);
        - ``accepts_bgr`` backends (e.g. rga_rknn_zero): NO cvtColor here --
          the RGA hardware does BGR->RGB during its letterbox pass -- and the
          frame is letterboxed to the fixed 640x480 deployment geometry so
          the RGA stride/alignment and ptr cache stay constant.
        """

        height, width = frame_bgr.shape[:2]
        preview_bgr = self.cv2.flip(frame_bgr, 1)
        if self._backend_takes_bgr:
            target_width = int(self.source.inference_width)
            target_height = int(round(target_width * 3 / 4))  # 640x480 preset
            inference_bgr = letterbox_frame(
                self.cv2, self.np, preview_bgr, target_width, target_height
            )
            return preview_bgr, inference_bgr
        inference_bgr = preview_bgr
        if (
            not getattr(self.backend, "requires_full_resolution", False)
            and width > self.source.inference_width
        ):
            inference_height = max(1, round(height * self.source.inference_width / width))
            inference_bgr = self.cv2.resize(
                preview_bgr,
                (self.source.inference_width, inference_height),
            )
        return preview_bgr, self.cv2.cvtColor(inference_bgr, self.cv2.COLOR_BGR2RGB)

    def read(self) -> dict[str, list[float]]:
        frame_bgr, sequence, _ = self._wait_for_latest_frame()
        self._last_inference_sequence = sequence
        now = time.monotonic()
        if not self._take_inference_slot(now):
            return {}
        frame_bgr, frame_image = self._prepare_inference_image(frame_bgr)
        inference_started = time.monotonic()
        if self._backend_takes_bgr:
            results = self.backend.process_bgr(frame_image)
        else:
            results = self.backend.process(frame_image)
        selected = self._select_legacy_per_side(results)
        inference_finished = time.monotonic()
        inference_duration = inference_finished - inference_started
        with self._metrics_lock:
            self._inference_count += 1
            self._inference_time_s += inference_duration
            self._latest_result_at = inference_finished
            self._inference_samples.append((inference_finished, inference_duration))
            self._prune_metrics(inference_finished)
        commands: dict[str, list[float]] = {}
        for side, (landmarks_2d, world_landmarks) in selected.items():
            processed = self._process_world_landmarks(world_landmarks, side)
            landmarks = self._mediapipe_to_mocap(processed)
            landmarks = (
                self.source.ema_alpha * landmarks
                + (1.0 - self.source.ema_alpha) * self._ema_cache[side]
            ).astype(self.np.float32)
            self._ema_cache[side] = landmarks
            landmarks = self._filters[side](landmarks, timestamp=inference_finished)
            commands[side] = self._retarget_compact(landmarks)
            if self.source.render_preview:
                self._draw_landmarks(frame_bgr, landmarks_2d, side)
        if self.source.render_preview:
            with self._preview_lock:
                self._preview_frame = frame_bgr.copy()
        return commands

    def _capture_loop(self) -> None:
        period_s = 1.0 / self.source.fps
        next_tick = time.monotonic()
        while not self._capture_stop.is_set():
            try:
                frame_bgr = self._read_camera_frame()
                self._publish_captured_frame(frame_bgr, captured_at=time.monotonic())
                next_tick += period_s
                delay_s = max(0.0, next_tick - time.monotonic())
                if self._capture_stop.wait(delay_s):
                    return
                if delay_s <= 0.0:
                    next_tick = time.monotonic()
            except Exception as exc:
                with self._frame_condition:
                    self._capture_error = exc
                    self._frame_condition.notify_all()
                self._capture_stop.wait(0.1)
                next_tick = time.monotonic()

    def _read_camera_frame(self) -> Any:
        if self.capture is not None:
            ok, frame_bgr = self.capture.read()
            if not ok or frame_bgr is None:
                raise RuntimeError("Could not read frame from camera")
            return frame_bgr
        try:
            jpeg, _, _ = fetch_shared_frame(
                f"http://127.0.0.1:{self.source.frame_server_port}/frame",
                timeout_s=max(0.1, 3.0 / self.source.fps),
            )
            frame_bgr = self.cv2.imdecode(
                self.np.frombuffer(jpeg, dtype=self.np.uint8),
                self.cv2.IMREAD_COLOR,
            )
            if frame_bgr is None:
                raise RuntimeError("Could not decode shared camera frame")
            return frame_bgr
        except RuntimeError:
            if not self.source._try_become_camera_owner():
                raise
            try:
                self._open_capture()
            except Exception:
                self.source._stop_frame_server()
                raise
            ok, frame_bgr = self.capture.read()
            if not ok or frame_bgr is None:
                raise RuntimeError("Could not read frame from camera")
            return frame_bgr

    def _publish_captured_frame(self, frame_bgr: Any, *, captured_at: float) -> None:
        with self._frame_condition:
            self._latest_frame = frame_bgr
            self._latest_frame_sequence += 1
            self._latest_frame_at = float(captured_at)
            self._capture_count += 1
            self._capture_error = None
            self._frame_condition.notify_all()
        with self._metrics_lock:
            self._capture_times.append(float(captured_at))
            self._prune_metrics(float(captured_at))

    def _wait_for_latest_frame(self) -> tuple[Any, int, float]:
        timeout_s = max(0.5, 3.0 / self.source.fps)
        deadline = time.monotonic() + timeout_s
        with self._frame_condition:
            while (
                self._latest_frame is None
                or self._latest_frame_sequence == self._last_inference_sequence
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    if self._capture_error is not None:
                        raise RuntimeError("Camera capture failed") from self._capture_error
                    raise RuntimeError("Timed out waiting for a fresh camera frame")
                self._frame_condition.wait(remaining)
            return (
                self._latest_frame,
                self._latest_frame_sequence,
                self._latest_frame_at,
            )

    def _take_inference_slot(self, now: float) -> bool:
        if now < self._next_inference_at:
            return False
        self._next_inference_at = now + 1.0 / self.source.inference_fps
        return True

    def _open_capture(self) -> None:
        capture = self.cv2.VideoCapture(self.source.camera_index)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"Could not open camera index {self.source.camera_index}")
        if self.source.camera_width:
            capture.set(self.cv2.CAP_PROP_FRAME_WIDTH, self.source.camera_width)
        if self.source.camera_height:
            capture.set(self.cv2.CAP_PROP_FRAME_HEIGHT, self.source.camera_height)
        capture.set(self.cv2.CAP_PROP_FPS, self.source.fps)
        capture.set(self.cv2.CAP_PROP_BUFFERSIZE, 1)
        self.capture = capture

    def get_camera_frame_jpeg(self) -> tuple[bytes, int, int]:
        with self._frame_condition:
            if self._latest_frame is None:
                raise RuntimeError("camera has not produced a frame")
            frame = self._latest_frame
        height, width = frame.shape[:2]
        encoded, jpeg = self.cv2.imencode(
            ".jpg",
            frame,
            [int(self.cv2.IMWRITE_JPEG_QUALITY), 90],
        )
        if not encoded:
            raise RuntimeError("failed to encode camera frame")
        return jpeg.tobytes(), width, height

    def get_performance(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._frame_condition:
            latest_frame_at = self._latest_frame_at
        with self._metrics_lock:
            self._prune_metrics(now)
            capture_times = tuple(self._capture_times)
            inference_samples = tuple(self._inference_samples)
            latest_result_at = self._latest_result_at
        capture_window_s = (
            max(capture_times[-1] - capture_times[0], 1e-6)
            if len(capture_times) >= 2
            else 0.0
        )
        inference_window_s = (
            max(inference_samples[-1][0] - inference_samples[0][0], 1e-6)
            if len(inference_samples) >= 2
            else 0.0
        )
        return {
            "capture_target_fps": self.source.fps,
            "inference_target_fps": self.source.inference_fps,
            "capture_fps": (
                (len(capture_times) - 1) / capture_window_s
                if capture_window_s
                else 0.0
            ),
            "inference_fps": (
                (len(inference_samples) - 1) / inference_window_s
                if inference_window_s
                else 0.0
            ),
            "inference_mean_ms": (
                sum(duration for _, duration in inference_samples)
                / len(inference_samples)
                * 1000.0
                if inference_samples
                else 0.0
            ),
            "latest_frame_age_ms": (
                max(0.0, now - latest_frame_at) * 1000.0
                if latest_frame_at
                else None
            ),
            "latest_result_age_ms": (
                max(0.0, now - latest_result_at) * 1000.0
                if latest_result_at
                else None
            ),
            "inference_backend": getattr(
                self.backend, "name", self.source.inference_backend
            ),
            "inference_threads": self.source.inference_threads,
        }

    def _prune_metrics(self, now: float) -> None:
        cutoff = now - 5.0
        while self._capture_times and self._capture_times[0] < cutoff:
            self._capture_times.popleft()
        while self._inference_samples and self._inference_samples[0][0] < cutoff:
            self._inference_samples.popleft()

    def get_preview_jpeg(self) -> bytes:
        if not self.source.render_preview:
            jpeg, _, _ = self.get_camera_frame_jpeg()
            return jpeg
        with self._preview_lock:
            if self._preview_frame is None:
                raise RuntimeError("camera has not produced a preview frame")
            frame = self._preview_frame.copy()
        height, width = frame.shape[:2]
        scale = min(1.0, 960.0 / float(max(width, height)))
        if scale < 1.0:
            frame = self.cv2.resize(frame, (int(width * scale), int(height * scale)))
        ok, data = self.cv2.imencode(
            ".jpg",
            frame,
            [int(self.cv2.IMWRITE_JPEG_QUALITY), 80],
        )
        if not ok:
            raise RuntimeError("failed to encode camera gesture preview")
        return data.tobytes()

    def close(self) -> None:
        self._capture_stop.set()
        with self._frame_condition:
            self._frame_condition.notify_all()
        if self._capture_thread is not threading.current_thread():
            self._capture_thread.join(timeout=_WORKER_JOIN_TIMEOUT_S)
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        self.backend.close()

    def _select_legacy_per_side(self, results: Any) -> dict[str, tuple[Any, Any]]:
        selected: dict[str, tuple[Any, Any]] = {}
        areas: dict[str, float] = {}
        landmarks = results.multi_hand_landmarks or []
        world_landmarks = results.multi_hand_world_landmarks or []
        handedness = results.multi_handedness or []
        for landmarks_2d, world, handed in zip(landmarks, world_landmarks, handedness):
            if not handed.classification:
                continue
            side = str(handed.classification[0].label).lower()
            if self.source.swap_handedness:
                side = {"left": "right", "right": "left"}.get(side, side)
            if side not in HAND_ORDER or side not in self.source._detectable_sides():
                continue
            points_2d = landmarks_2d.landmark
            area = _bbox_area(points_2d)
            if area < self.source.min_hand_area:
                continue
            if side not in areas or area > areas[side]:
                selected[side] = (points_2d, world.landmark)
                areas[side] = area
        if self.source.mode == "single" and self.source.side == "auto" and selected:
            selected_side = (
                self.source._selected_side
                if self.source._selected_side in selected
                else max(selected, key=areas.__getitem__)
            )
            return {selected_side: selected[selected_side]}
        return selected

    def _process_world_landmarks(self, landmarks: Any, side: str) -> Any:
        points = self.np.asarray([[-p.x, p.y, p.z] for p in landmarks], dtype=self.np.float32)
        x_axis = _unit(self.np, points[5] - points[13])
        if side == "left":
            x_axis = -x_axis
        z_axis = _unit(self.np, points[9] - points[0])
        y_axis = _unit(self.np, self.np.cross(z_axis, x_axis))
        x_axis = _unit(self.np, self.np.cross(y_axis, z_axis))
        rotation = self.np.array([x_axis, y_axis, z_axis], dtype=self.np.float32).T
        return (points - points[0]) @ rotation

    def _mediapipe_to_mocap(self, landmarks: Any) -> Any:
        indices = (0, 1, 2, 3, 4, 0, 5, 6, 7, 8, 0, 9, 10, 11, 12, 0, 13, 14, 15, 16, 0, 17, 18, 19, 20)
        return self.np.asarray(landmarks, dtype=self.np.float32)[list(indices)]

    def _retarget_compact(self, landmarks: Any) -> list[float]:
        thumb = landmarks[0:5]
        joints = [
            _angle(self.np, self.np.array([thumb[2][0], thumb[1][1], 0.0]), self.np.array([thumb[1][0], thumb[1][1], 0.0]), self.np.array([thumb[2][0], thumb[2][1], 0.0])),
            self.np.pi - _angle(self.np, thumb[0], thumb[1], thumb[2]),
            self.np.pi - _angle(self.np, thumb[1], thumb[2], thumb[3]),
            self.np.pi - _angle(self.np, thumb[2], thumb[3], thumb[4]),
        ]
        for start in (5, 10, 15, 20):
            finger = landmarks[start : start + 5]
            joints.extend(
                [
                    self.np.pi - _angle(self.np, finger[0], finger[1], finger[2]),
                    self.np.pi - _angle(self.np, finger[1], finger[2], finger[3]),
                    self.np.pi - _angle(self.np, finger[2], finger[3], finger[4]),
                ]
            )
        normalized = [
            _normalize_joint(self.np, float(value), index, self.normalize_config)
            for index, value in enumerate(joints)
        ]
        return [float(normalized[index]) for index in COMPACT_JOINT_INDICES]

    def _draw_landmarks(self, frame: Any, landmarks: Any, side: str) -> None:
        height, width = frame.shape[:2]
        points = [(int(p.x * width), int(p.y * height)) for p in landmarks]
        for connection in self._connections:
            start = connection[0] if isinstance(connection, tuple) else connection.start
            end = connection[1] if isinstance(connection, tuple) else connection.end
            self.cv2.line(frame, points[start], points[end], (0, 255, 0), 1)
        for point in points:
            self.cv2.circle(frame, point, 3, (0, 0, 255), -1)
        self.cv2.putText(frame, side, points[0], self.cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)


class _OneEuroFilterND:
    def __init__(self, np_module: Any, freq: float, min_cutoff: float, beta: float, d_cutoff: float) -> None:
        self.np = np_module
        self.freq = freq
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.initialized = False
        self.x_prev: Any = None
        self.x_hat: Any = None
        self.dx_hat: Any = None
        self._last_timestamp: float | None = None

    def __call__(self, value: Any, *, timestamp: float | None = None) -> Any:
        value = self.np.asarray(value, dtype=self.np.float32)
        if timestamp is None or self._last_timestamp is None:
            dt = 1.0 / self.freq
        else:
            dt = max(1e-6, float(timestamp) - self._last_timestamp)
        if timestamp is not None:
            self._last_timestamp = float(timestamp)
        if not self.initialized:
            self.initialized = True
            self.x_prev = value.copy()
            self.x_hat = value.copy()
            self.dx_hat = self.np.zeros_like(value)
            return value.copy()
        dx = (value - self.x_prev) / dt
        alpha_d = _filter_alpha(self.np, self.d_cutoff, dt)
        self.dx_hat = alpha_d * dx + (1.0 - alpha_d) * self.dx_hat
        cutoff = self.min_cutoff + self.beta * self.np.abs(self.dx_hat)
        alpha = _filter_alpha(self.np, cutoff, dt)
        self.x_hat = alpha * value + (1.0 - alpha) * self.x_hat
        self.x_prev = value.copy()
        return self.x_hat.copy()


def _create_pipeline(source: AeroHandVisionMaster) -> _VisionPipeline:
    return _VisionPipeline(source)


def _validate_mode(value: Any) -> str:
    mode = str(value or "single").strip().lower()
    if mode not in {"single", "dual"}:
        raise ValueError("mode must be 'single' or 'dual'")
    return mode


def _validate_side(value: Any) -> str:
    side = str(value or "auto").strip().lower()
    if side not in {*HAND_ORDER, "auto"}:
        raise ValueError("side must be 'auto', 'left', or 'right'")
    return side


def _validate_visual(value: Any) -> str:
    visual = str(value or "none").strip().lower()
    if visual != "none":
        raise ValueError("visual must be 'none'; use the web preview to view camera output")
    return visual


def _optional_positive_int(value: Any, field_name: str) -> int | None:
    if value in (None, ""):
        return None
    number = int(value)
    if number <= 0:
        raise ValueError(f"{field_name} must be positive")
    return number


def _validate_compact_positions(values: Any) -> list[float]:
    positions = [float(value) for value in values]
    if len(positions) != 7:
        raise ValueError(f"camera gesture retargeter must return 7 joints per hand, got {len(positions)}")
    return positions


def _warning(code: str, message: str, details: dict[str, Any] | None = None) -> dict[str, Any]:
    return {
        "code": code,
        "message": message,
        "source": "robot",
        "timestamp": time.time(),
        "details": details or {},
    }


def _bbox_area(landmarks: Any) -> float:
    xs = [float(point.x) for point in landmarks]
    ys = [float(point.y) for point in landmarks]
    return max(max(xs) - min(xs), 0.0) * max(max(ys) - min(ys), 0.0)


def _unit(np_module: Any, vector: Any) -> Any:
    norm = float(np_module.linalg.norm(vector))
    return np_module.zeros_like(vector) if norm < 1e-9 else vector / norm


def _angle(np_module: Any, a: Any, b: Any, c: Any) -> float:
    ba = a - b
    bc = c - b
    denom = float(np_module.linalg.norm(ba) * np_module.linalg.norm(bc))
    if denom < 1e-9:
        return 0.0
    cosine = float(np_module.dot(ba, bc) / denom)
    return float(np_module.arccos(np_module.clip(cosine, -1.0, 1.0)))


def _filter_alpha(np_module: Any, cutoff: Any, dt: float) -> Any:
    cutoff = np_module.maximum(cutoff, 1e-6)
    tau = 1.0 / (2.0 * np_module.pi * cutoff)
    return dt / (dt + tau)


def _normalize_joint(np_module: Any, raw: float, index: int, config: dict[str, Any]) -> float:
    names = (
        "thumb_cmc_abd", "thumb_cmc_flex", "thumb_mcp", "thumb_ip",
        "index_mcp_flex", "index_pip", "index_dip",
        "middle_mcp_flex", "middle_pip", "middle_dip",
        "ring_mcp_flex", "ring_pip", "ring_dip",
        "pinky_mcp_flex", "pinky_pip", "pinky_dip",
    )
    item = config[names[index]]
    valley = float(item["valley"])
    peak = float(item["peak"])
    ratio = (raw - valley) / max(abs(peak - valley), 1e-9)
    if index > 0:
        open_threshold = float(config.get("grip_open_threshold", 0.0))
        close_threshold = float(config.get("grip_close_threshold", 1.0))
        ratio = (ratio - open_threshold) / max(
            close_threshold - open_threshold,
            1e-9,
        )
    upper = float(np_module.deg2rad(JOINT_UPPER_LIMITS_DEG[index]))
    return float(np_module.clip(ratio * upper, 0.0, upper))
