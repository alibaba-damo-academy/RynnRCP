"""Camera-gesture master source for Aero Hand teleoperation.

The MediaPipe detection and retargeting pipeline follows
AGI-Robot/aero_hand_control at commit d3da2ac36f26ad1dfb98eab8e29402cb4dfd0d76.
It exposes the camera operator as an RCP joint-state source: 7 radians for a
single hand, or left 7 + right 7 radians for a dual-hand profile.
"""

from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Any

from rynnrcp.robot.base_controller import BaseRobotController

from .shared_camera import fetch_shared_frame, stop_frame_server, try_start_frame_server


PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_MODEL_PATH = PACKAGE_DIR / "model" / "hand_landmarker.task"
DEFAULT_NORMALIZE_CONFIG = PACKAGE_DIR / "config" / "normalize_default_mediapipe.yaml"
HAND_ORDER = ("left", "right")
COMPACT_JOINT_INDICES = (0, 1, 2, 4, 7, 10, 13)
JOINT_UPPER_LIMITS_DEG = (100.0, 55.0) + (90.0,) * 14
_WORKER_JOIN_TIMEOUT_S = 2.0
_WORKER_ERROR_LOG_INTERVAL_S = 5.0


class AeroHandVisionMaster(BaseRobotController):
    """Publish camera-recognized hand gestures as a joint-state observation."""

    def __init__(
        self,
        robot_id: str,
        mode: str = "single",
        side: str = "auto",
        camera_index: int = 0,
        fps: float = 30.0,
        hand_landmarker_model: str | None = None,
        swap_handedness: bool = True,
        camera_width: int | None = None,
        camera_height: int | None = None,
        frame_server_port: int = 28412,
        min_hand_area: float = 0.005,
        ema_alpha: float = 0.7,
        min_cutoff: float = 5.0,
        beta: float = 1.3,
        d_cutoff: float = 1.4,
        visual: str = "none",
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__(logger=logger)
        self.robot_id = str(robot_id)
        self.mode = _validate_mode(mode)
        self.side = _validate_side(side)
        self.camera_index = int(camera_index)
        self.fps = max(1.0, float(fps))
        self.hand_landmarker_model = str(hand_landmarker_model or DEFAULT_MODEL_PATH)
        self.swap_handedness = bool(swap_handedness)
        self.camera_width = _optional_positive_int(camera_width)
        self.camera_height = _optional_positive_int(camera_height)
        self.frame_server_port = int(frame_server_port)
        if not 0 <= self.frame_server_port <= 65535:
            raise ValueError("frame_server_port must be between 0 and 65535")
        self.min_hand_area = max(0.0, float(min_hand_area))
        self.ema_alpha = min(1.0, max(0.0, float(ema_alpha)))
        self.min_cutoff = max(1e-6, float(min_cutoff))
        self.beta = max(0.0, float(beta))
        self.d_cutoff = max(1e-6, float(d_cutoff))
        self.visual = _validate_visual(visual)
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
            if self._camera_frame is None:
                raise RuntimeError("camera has not produced a frame")
            data, width, height = self._camera_frame
        return data, width, height

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
        period_s = 1.0 / self.fps
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
    """MediaPipe Tasks camera reader and Aero Hand compact-joint retargeter."""

    def __init__(self, source: AeroHandVisionMaster) -> None:
        try:
            import cv2
            import mediapipe as mp
            import numpy as np
            import yaml
            from mediapipe.tasks import python
            from mediapipe.tasks.python import vision
        except ImportError as exc:
            raise RuntimeError(
                "Camera gesture control requires opencv-python, mediapipe, numpy, and pyyaml"
            ) from exc

        model_path = Path(source.hand_landmarker_model).expanduser().resolve()
        if not model_path.is_file():
            raise FileNotFoundError(f"MediaPipe hand landmarker model not found: {model_path}")
        normalize_config = yaml.safe_load(DEFAULT_NORMALIZE_CONFIG.read_text(encoding="utf-8"))
        if not isinstance(normalize_config, dict):
            raise ValueError(f"Invalid normalization config: {DEFAULT_NORMALIZE_CONFIG}")

        self.cv2 = cv2
        self.mp = mp
        self.np = np
        self.vision = vision
        self.source = source
        self.normalize_config = normalize_config
        self._filters = {
            side: _OneEuroFilterND(np, source.fps, source.min_cutoff, source.beta, source.d_cutoff)
            for side in HAND_ORDER
        }
        self._ema_cache = {side: np.zeros((25, 3), dtype=np.float32) for side in HAND_ORDER}
        self._preview_lock = threading.Lock()
        self._preview_frame: Any | None = None
        options = vision.HandLandmarkerOptions(
            base_options=python.BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=4,
        )
        self.landmarker = vision.HandLandmarker.create_from_options(options)
        self.capture = None
        if source._owns_camera:
            self._open_capture()
        self._last_timestamp_ms = 0

    def read(self) -> dict[str, list[float]]:
        if self.capture is None:
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
        else:
            ok, frame_bgr = self.capture.read()
            if not ok or frame_bgr is None:
                raise RuntimeError("Could not read frame from camera")
        height, width = frame_bgr.shape[:2]
        encoded, jpeg = self.cv2.imencode(
            ".jpg",
            frame_bgr,
            [int(self.cv2.IMWRITE_JPEG_QUALITY), 90],
        )
        if encoded:
            self.source._update_camera_frame(jpeg.tobytes(), width, height)
        frame_bgr = self.cv2.flip(frame_bgr, 1)
        frame_rgb = self.cv2.cvtColor(frame_bgr, self.cv2.COLOR_BGR2RGB)
        mp_image = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=frame_rgb)
        timestamp_ms = max(self._last_timestamp_ms + 1, int(time.monotonic() * 1000))
        self._last_timestamp_ms = timestamp_ms
        results = self.landmarker.detect_for_video(mp_image, timestamp_ms)
        selected = self._select_closest_per_side(results)
        commands: dict[str, list[float]] = {}
        for side, (landmarks_2d, world_landmarks) in selected.items():
            processed = self._process_world_landmarks(world_landmarks, side)
            landmarks = self._mediapipe_to_mocap(processed)
            landmarks = (
                self.source.ema_alpha * landmarks
                + (1.0 - self.source.ema_alpha) * self._ema_cache[side]
            ).astype(self.np.float32)
            self._ema_cache[side] = landmarks
            landmarks = self._filters[side](landmarks)
            commands[side] = self._retarget_compact(landmarks)
            self._draw_landmarks(frame_bgr, landmarks_2d, side)
        with self._preview_lock:
            self._preview_frame = frame_bgr.copy()
        if self.source.visual == "window":
            self.cv2.imshow("Aero Hand Camera Master", frame_bgr)
            self.cv2.waitKey(1)
        return commands

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
        self.capture = capture

    def get_preview_jpeg(self) -> bytes:
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
        self.landmarker.close()
        if self.capture is not None:
            self.capture.release()
        if self.source.visual == "window":
            self.cv2.destroyAllWindows()

    def _select_closest_per_side(self, results: Any) -> dict[str, tuple[Any, Any]]:
        selected: dict[str, tuple[Any, Any]] = {}
        areas: dict[str, float] = {}
        if not (results.hand_landmarks and results.hand_world_landmarks and results.handedness):
            return selected
        for landmarks_2d, world_landmarks, handedness in zip(
            results.hand_landmarks,
            results.hand_world_landmarks,
            results.handedness,
        ):
            side = str(handedness[0].category_name).lower()
            if self.source.swap_handedness:
                side = {"left": "right", "right": "left"}.get(side, side)
            if side not in HAND_ORDER or side not in self.source._detectable_sides():
                continue
            area = _bbox_area(landmarks_2d)
            if area < self.source.min_hand_area:
                continue
            if side not in areas or area > areas[side]:
                selected[side] = (landmarks_2d, world_landmarks)
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
        for connection in self.vision.HandLandmarksConnections.HAND_CONNECTIONS:
            self.cv2.line(frame, points[connection.start], points[connection.end], (0, 255, 0), 1)
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

    def __call__(self, value: Any) -> Any:
        value = self.np.asarray(value, dtype=self.np.float32)
        dt = 1.0 / self.freq
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
    if visual not in {"none", "window"}:
        raise ValueError("visual must be 'none' or 'window'")
    return visual


def _optional_positive_int(value: Any) -> int | None:
    if value in (None, ""):
        return None
    number = int(value)
    if number <= 0:
        raise ValueError("camera dimensions must be positive")
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
    upper = float(np_module.deg2rad(JOINT_UPPER_LIMITS_DEG[index]))
    return float(np_module.clip(ratio * upper, 0.0, upper))
