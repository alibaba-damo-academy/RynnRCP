"""MediaPipe Lite cascade for Jetson (ORIN NX): backend ``mediapipe_lite_jetson``.

Portable-across-devices design: the ONNX exports in the repo-root
``models/onnx/`` tree (distributed by the gitee model zoo, fetched on first
use) are the only shared assets; TensorRT engines are device-local (they
depend on the TRT version and GPU SM) and are built on the target device,
then cached under the repo-root ``models/jetson_orin_nx/`` with a versioned
file name.

Executor fallback chain (each level falls through automatically):

  1. cached TensorRT engine   (models/jetson_orin_nx/*_trt<ver>_sm<cc>.engine)
  2. TensorRT runtime build   (builds + caches the engine on first use;
                               disable with AERO_HAND_TRT_BUILD=0)
  3. onnxruntime CUDA EP      (requires the Jetson onnxruntime-gpu wheel)
  4. onnxruntime CPU EP       (always available, slowest)

Env vars:

  AERO_HAND_JETSON_DIR          model dir (default: package models/)
  AERO_HAND_JETSON_PALM/HAND    ONNX overrides
  AERO_HAND_JETSON_ENGINE_DIR   engine cache dir (default: models/engine/)
  AERO_HAND_JETSON_EXECUTOR     auto|trt|onnx|cpu   (default auto)
  AERO_HAND_TRT_BUILD           auto|1|0            (default auto: build only
                                when the cached engine is missing/incompatible)
  AERO_HAND_TRT_FP16            1|0                 (default 1)

Prebuild engines on a fresh device with ``scripts/build_jetson_engines.py``.

Cascade (identical to mediapipe_lite_rknn / mediapipe_lite_mtnn):

  frame RGB -> letterbox 192 -> palm -> SSD 2016-anchor decode + weighted NMS
            -> rotated square ROI (wrist->middle, x2.6, shift -0.5)
            -> affine crop 224 -> hand -> landmark projection back
  tracking  -> next-frame ROI from previous landmarks (x2.0, shift -0.1);
               palm reruns only when tracked hands < num_hands.

Preprocessing: float32 NCHW, /255 -> [0,1] (matches models/README.md).
"""
from __future__ import annotations

import logging
import math
import os
import re
from types import SimpleNamespace
from typing import Any

import cv2
import numpy as np

from rynnrcp_robot_aero_hand import model_store
from rynnrcp_robot_aero_hand.gesture_inference import register_gesture_backend

logger = logging.getLogger(__name__)

# ONNX source models live in the repo-root models/ tree (not git-tracked;
# fetched from the gitee model zoo by model_store on first use).
_MODEL_DIR = os.environ.get(
    "AERO_HAND_JETSON_DIR", model_store.models_root(),
)
PALM_ONNX = os.environ.get(
    "AERO_HAND_JETSON_PALM",
    os.path.join(_MODEL_DIR, "onnx", "palm_detection_192_nchw.onnx"),
)
HAND_ONNX = os.environ.get(
    "AERO_HAND_JETSON_HAND",
    os.path.join(_MODEL_DIR, "onnx", "hand_landmark_224.onnx"),
)
ENGINE_DIR = os.environ.get(
    "AERO_HAND_JETSON_ENGINE_DIR",
    os.path.join(model_store.models_root(), "jetson_orin_nx"),
)

PALM_SIZE = 192
HAND_SIZE = 224
PALM_SCORE_THR = 0.5
PRESENCE_THR = 0.5
NMS_IOU = 0.3
# Official HandLandmarksToRectCalculator: the tracking ROI is built from the
# 12 stable palm/knuckle landmarks only (fingertips excluded).
ROI_LANDMARKS = (0, 1, 2, 3, 5, 6, 9, 10, 13, 14, 17, 18)


# ----------------------------------------------------------------- executors
def _natural_key(name: str):
    return [int(c) if c.isdigit() else c for c in re.split(r"(\d+)", name)]


def engine_tag() -> str:
    """Version tag binding an engine to this device: TRT version + GPU SM."""
    import tensorrt as trt

    sm = "smXX"
    try:
        import pycuda.driver as cuda

        cuda.init()
        major, minor = cuda.Device(0).compute_capability()
        sm = f"sm{major}{minor}"
    except Exception:
        pass
    return f"trt{trt.__version__}_{sm}"


def engine_path_for(onnx_path: str) -> str:
    stem = os.path.splitext(os.path.basename(onnx_path))[0]
    suffix = "" if _env_flag("AERO_HAND_TRT_FP16", "1") in ("1", "true", "yes") else "_fp32"
    return os.path.join(ENGINE_DIR, f"{stem}_{engine_tag()}{suffix}.engine")


def build_engine(onnx_path: str, engine_path: str, fp16: bool = True,
                 workspace_bytes: int = 1 << 30) -> None:
    """Build a fixed-shape FP16/FP32 engine from ONNX on this device."""
    import tensorrt as trt

    trt_logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(trt_logger)
    network = builder.create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    )
    parser = trt.OnnxParser(network, trt_logger)
    with open(onnx_path, "rb") as fh:
        if not parser.parse(fh.read()):
            errors = "; ".join(
                str(parser.get_error(i)) for i in range(parser.num_errors)
            )
            raise RuntimeError(f"ONNX parse failed for {onnx_path}: {errors}")
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, workspace_bytes)
    if fp16 and builder.platform_has_fast_fp16:
        config.set_flag(trt.BuilderFlag.FP16)
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError(f"TensorRT build failed for {onnx_path}")
    os.makedirs(os.path.dirname(engine_path), exist_ok=True)
    tmp_path = engine_path + ".tmp"
    with open(tmp_path, "wb") as fh:
        fh.write(bytes(serialized))
    os.replace(tmp_path, engine_path)


class _TrtExecutor:
    """Deserialized-engine executor with pinned host buffers (pycuda)."""

    def __init__(self, engine_path: str) -> None:
        import tensorrt as trt
        import pycuda.driver as cuda

        if not os.path.isfile(engine_path):
            raise FileNotFoundError(f"engine not found: {engine_path}")
        self._cuda = cuda
        cuda.init()
        # Primary context (shared with other runtimes, e.g. ORT CUDA EP).
        # Palm/hand executors retain the same primary context, so the
        # push here is balanced by pop() in close().
        self._ctx = cuda.Device(0).retain_primary_context()
        self._ctx.push()
        try:
            runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
            with open(engine_path, "rb") as fh:
                self._engine = runtime.deserialize_cuda_engine(fh.read())
            if self._engine is None:
                raise RuntimeError(f"failed to deserialize engine: {engine_path}")
            self._context = self._engine.create_execution_context()
            names = [
                self._engine.get_tensor_name(i)
                for i in range(self._engine.num_io_tensors)
            ]
            self._in_name = next(
                n for n in names
                if self._engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT
            )
            self._out_names = sorted(
                (n for n in names
                 if self._engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT),
                key=_natural_key,
            )
            self._dev = {}
            self._shapes = {}
            for name in names:
                shape = tuple(self._engine.get_tensor_shape(name))
                self._shapes[name] = shape
                self._dev[name] = cuda.mem_alloc(int(np.prod(shape)) * 4)
            for name, dev in self._dev.items():
                self._context.set_tensor_address(name, int(dev))
            self._in_host = cuda.pagelocked_empty(self._shapes[self._in_name], np.float32)
            self._out_hosts = {
                n: cuda.pagelocked_empty(self._shapes[n], np.float32)
                for n in self._out_names
            }
            self._stream = cuda.Stream()
        except Exception:
            # Balance the push above so a failed init (e.g. corrupt engine
            # on the runtime-rebuild path) does not leak a context, which
            # makes PyCUDA abort at interpreter exit.
            try:
                self._ctx.pop()
            except Exception:
                pass
            raise

    @property
    def input_shape(self) -> tuple[int, ...]:
        return self._shapes[self._in_name]

    def run(self, nchw_f32: np.ndarray) -> list[np.ndarray]:
        cuda = self._cuda
        np.copyto(self._in_host, nchw_f32)
        cuda.memcpy_htod_async(self._dev[self._in_name], self._in_host, self._stream)
        self._context.execute_async_v3(self._stream.handle)
        for name in self._out_names:
            cuda.memcpy_dtoh_async(self._out_hosts[name], self._dev[name], self._stream)
        self._stream.synchronize()
        return [np.array(self._out_hosts[n]) for n in self._out_names]

    def close(self) -> None:
        try:
            self._ctx.pop()
        except Exception:
            pass
        del self._context
        del self._engine


class _OrtExecutor:
    """onnxruntime executor; outputs returned in natural ONNX output order."""

    def __init__(self, onnx_path: str, providers: list[str]) -> None:
        import onnxruntime as ort

        if not os.path.isfile(onnx_path):
            raise FileNotFoundError(f"onnx model not found: {onnx_path}")
        self.session = ort.InferenceSession(str(onnx_path), providers=providers)
        self.providers = self.session.get_providers()
        self._in_name = self.session.get_inputs()[0].name
        self._out_names = sorted((o.name for o in self.session.get_outputs()),
                                 key=_natural_key)

    def run(self, nchw_f32: np.ndarray) -> list[np.ndarray]:
        return self.session.run(
            self._out_names,
            {self._in_name: np.ascontiguousarray(nchw_f32, np.float32)},
        )

    def close(self) -> None:
        pass


def _env_flag(name: str, default: str) -> str:
    return os.environ.get(name, default).strip().lower()


def _create_executors(palm_onnx: str, hand_onnx: str) -> tuple[Any, Any, str]:
    """Pick executors for BOTH models together so they share one path."""
    mode = _env_flag("AERO_HAND_JETSON_EXECUTOR", "auto")
    allow_build = _env_flag("AERO_HAND_TRT_BUILD", "auto") in ("auto", "1", "true", "yes")
    fp16 = _env_flag("AERO_HAND_TRT_FP16", "1") in ("1", "true", "yes")

    if mode in ("auto", "trt"):
        executors: list[Any] = []
        try:
            for path in (palm_onnx, hand_onnx):
                executors.append(_trt_executor_for(path, allow_build, fp16))
            logger.info("mediapipe_lite_jetson: TensorRT engines (%s)", engine_tag())
            return executors[0], executors[1], "tensorrt"
        except Exception as exc:
            for executor in executors:
                try:
                    executor.close()
                except Exception:
                    pass
            logger.warning("mediapipe_lite_jetson: TensorRT path unavailable: %s", exc)
            if mode == "trt":
                raise

    providers = (
        ["CPUExecutionProvider"]
        if mode == "cpu"
        else ["CUDAExecutionProvider", "CPUExecutionProvider"]
    )
    executors = [_OrtExecutor(p, providers) for p in (palm_onnx, hand_onnx)]
    active = executors[0].providers
    if "CUDAExecutionProvider" in active:
        logger.info("mediapipe_lite_jetson: onnxruntime CUDA EP")
        return executors[0], executors[1], "onnx_cuda"
    logger.warning(
        "mediapipe_lite_jetson: onnxruntime fell back to CPU EP (%s); "
        "install the Jetson onnxruntime-gpu wheel for GPU acceleration", active
    )
    return executors[0], executors[1], "onnx_cpu"


def _try_fetch_prebuilt_engine(engine_path: str) -> None:
    """Pull a matching prebuilt engine from the model zoo, if published."""

    name = os.path.basename(engine_path)
    for zoo_name, sha256, size in model_store.MANIFEST.get("jetson_orin_nx", ()):
        if zoo_name == name:
            try:
                model_store.fetch_file(
                    f"models/jetson_orin_nx/{zoo_name}", engine_path,
                    sha256, size,
                )
            except Exception as exc:  # noqa: BLE001 - local build path remains
                logger.info(
                    "mediapipe_lite_jetson: prebuilt engine download "
                    "skipped (%s); will build locally if needed", exc,
                )
            return


def _trt_executor_for(onnx_path: str, allow_build: bool, fp16: bool) -> _TrtExecutor:
    engine_path = engine_path_for(onnx_path)
    if not os.path.isfile(engine_path):
        _try_fetch_prebuilt_engine(engine_path)
    try:
        executor = _TrtExecutor(engine_path)
        executor.run(np.zeros(executor.input_shape, np.float32))
        return executor
    except Exception as exc:
        if not allow_build:
            raise RuntimeError(
                f"cached engine unusable ({exc}) and AERO_HAND_TRT_BUILD "
                "disabled; run scripts/build_jetson_engines.py"
            ) from exc
        logger.info("mediapipe_lite_jetson: building engine %s (first use)", engine_path)
        build_engine(onnx_path, engine_path, fp16=fp16)
        executor = _TrtExecutor(engine_path)
        executor.run(np.zeros(executor.input_shape, np.float32))
        return executor


def build_default_engines(force: bool = False, fp16: bool = True) -> list[str]:
    """Build (and validate) engines for the default models. Used by
    scripts/build_jetson_engines.py and setup on fresh devices."""
    built: list[str] = []
    model_store.ensure_backend_files((PALM_ONNX, HAND_ONNX))
    for onnx_path in (PALM_ONNX, HAND_ONNX):
        if not os.path.isfile(onnx_path):
            raise FileNotFoundError(f"onnx model not found: {onnx_path}")
        engine_path = engine_path_for(onnx_path)
        if not force:
            try:
                executor = _TrtExecutor(engine_path)
                executor.run(np.zeros(executor.input_shape, np.float32))
                executor.close()
                logger.info("engine already valid: %s", engine_path)
                built.append(engine_path)
                continue
            except Exception:
                pass
        build_engine(onnx_path, engine_path, fp16=fp16)
        executor = _TrtExecutor(engine_path)
        executor.run(np.zeros(executor.input_shape, np.float32))
        executor.close()
        logger.info("engine built and verified: %s", engine_path)
        built.append(engine_path)
    return built


# ------------------------------------------------------------------ cascade
def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -50.0, 50.0)))


def _palm_anchors() -> np.ndarray:
    """Canonical MediaPipe SSD anchors for palm_detection 192 (2016 x 2)."""
    strides = [8, 16, 16, 16]
    anchors: list[tuple[float, float]] = []
    layer = 0
    while layer < 4:
        count = 0
        last = layer
        while last < 4 and strides[last] == strides[layer]:
            count += 2  # aspect 1.0 + interpolated scale
            last += 1
        fm = int(math.ceil(PALM_SIZE / strides[layer]))
        for y in range(fm):
            for x in range(fm):
                for _ in range(count):
                    anchors.append(((x + 0.5) / fm, (y + 0.5) / fm))
        layer = last
    return np.asarray(anchors, np.float32)


def _weighted_nms(dets: list[dict], iou_thr: float) -> list[dict]:
    """MediaPipe-style weighted non-max suppression on normalized boxes."""

    def iou(a, b):
        ax0, ay0, ax1, ay1 = a["box"]
        bx0, by0, bx1, by1 = b["box"]
        iw = max(0.0, min(ax1, bx1) - max(ax0, bx0))
        ih = max(0.0, min(ay1, by1) - max(ay0, by0))
        inter = iw * ih
        union = (ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - inter
        return inter / union if union > 0 else 0.0

    remaining = sorted(dets, key=lambda d: -d["score"])
    out: list[dict] = []
    while remaining:
        top = remaining[0]
        cluster = [d for d in remaining if iou(top, d) > iou_thr]
        remaining = [d for d in remaining if iou(top, d) <= iou_thr]
        weight = np.asarray([d["score"] for d in cluster], np.float32)
        weight /= weight.sum()
        merged = {
            "score": top["score"],
            "box": np.tensordot(weight, np.asarray([d["box"] for d in cluster]), 1),
            "kps": np.tensordot(weight, np.asarray([d["kps"] for d in cluster]), 1),
        }
        out.append(merged)
    return out


def _as_prob(raw: float) -> float:
    """Model emits probability or logit depending on export; normalize."""
    return raw if 0.0 <= raw <= 1.0 else float(_sigmoid(raw))


class _Roi:
    __slots__ = ("cx", "cy", "size", "rot")

    def __init__(self, cx: float, cy: float, size: float, rot: float) -> None:
        self.cx, self.cy, self.size, self.rot = cx, cy, size, rot


def _make_roi(cx, cy, w, h, rot, scale, shift_y) -> _Roi:
    """RectTransformation: shift (orig size, rotated frame) -> scale -> square_long."""
    vx, vy = -math.sin(rot), math.cos(rot)  # rotated y axis (down)
    cx += shift_y * h * vx
    cy += shift_y * h * vy
    size = max(w, h) * scale
    return _Roi(cx, cy, size, rot)


def _rotation(x0, y0, x1, y1) -> float:
    """MediaPipe: target 90 deg - atan2(-(dy), dx), normalized."""
    rot = 0.5 * math.pi - math.atan2(-(y1 - y0), x1 - x0)
    return rot - 2.0 * math.pi * math.floor((rot + math.pi) / (2.0 * math.pi))


class MediaPipeLiteJetsonBackend:
    """MediaPipe Lite pipeline on Jetson GPU with device-portable fallbacks."""

    name = "mediapipe_lite_jetson"

    def __init__(self, *, mediapipe: Any = None, max_num_hands: int = 1,
                 num_threads: int = 1, palm_model: str | None = None,
                 hand_model: str | None = None,
                 name: str | None = None, **_ignored: Any) -> None:
        if name is not None:
            self.name = name
        palm_path = palm_model or PALM_ONNX
        hand_path = hand_model or HAND_ONNX
        model_store.ensure_backend_files((palm_path, hand_path))
        for path in (palm_path, hand_path):
            if not os.path.isfile(path):
                raise FileNotFoundError(f"onnx model not found: {path}")
        self.num_hands = max(1, int(max_num_hands))
        self.connections = (
            mediapipe.solutions.hands.HAND_CONNECTIONS
            if mediapipe is not None and hasattr(mediapipe, "solutions")
            else ()
        )
        self.model_bytes = sum(
            os.path.getsize(p) for p in (palm_path, hand_path) if os.path.isfile(p)
        )
        self._anchors = _palm_anchors()
        self._palm, self._hand, self.executor_kind = _create_executors(
            palm_path, hand_path
        )
        self._rois: list[_Roi] = []
        self._palm_buf = np.zeros((1, 3, PALM_SIZE, PALM_SIZE), np.float32)
        self._hand_buf = np.zeros((1, 3, HAND_SIZE, HAND_SIZE), np.float32)
        self._crop_buf = np.zeros((HAND_SIZE, HAND_SIZE, 3), np.uint8)

    # ------------------------------------------------------------------ palm
    def _detect_palms(self, image: np.ndarray) -> list[_Roi]:
        h, w = image.shape[:2]
        scale = PALM_SIZE / max(w, h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        pad_x, pad_y = (PALM_SIZE - nw) // 2, (PALM_SIZE - nh) // 2
        canvas = np.zeros((PALM_SIZE, PALM_SIZE, 3), np.uint8)
        canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = cv2.resize(
            image, (nw, nh), interpolation=cv2.INTER_LINEAR
        )
        self._palm_buf[0] = canvas.transpose(2, 0, 1) / 255.0
        outs = self._palm.run(self._palm_buf)
        outs = sorted(outs, key=lambda a: a.size, reverse=True)
        reg = outs[0].reshape(-1, 18)
        scores = _sigmoid(outs[1].reshape(-1))
        keep = np.where(scores > PALM_SCORE_THR)[0]
        if keep.size == 0:
            return []
        anchors = self._anchors[keep]
        xc = reg[keep, 0] / PALM_SIZE + anchors[:, 0]
        yc = reg[keep, 1] / PALM_SIZE + anchors[:, 1]
        bw = reg[keep, 2] / PALM_SIZE
        bh = reg[keep, 3] / PALM_SIZE
        kps = reg[keep, 4:18].reshape(-1, 7, 2) / PALM_SIZE + anchors[:, None, :]
        dets = [
            {
                "score": float(scores[i]),
                "box": np.array([xc[j] - bw[j] / 2, yc[j] - bh[j] / 2,
                                 xc[j] + bw[j] / 2, yc[j] + bh[j] / 2], np.float32),
                "kps": kps[j],
            }
            for j, i in enumerate(keep)
        ]
        rois = []
        for det in _weighted_nms(dets, NMS_IOU):
            box = det["box"] * PALM_SIZE
            kp = det["kps"] * PALM_SIZE
            x0, y0 = (box[0] - pad_x) / scale, (box[1] - pad_y) / scale
            x1, y1 = (box[2] - pad_x) / scale, (box[3] - pad_y) / scale
            wrist = ((kp[0, 0] - pad_x) / scale, (kp[0, 1] - pad_y) / scale)
            middle = ((kp[2, 0] - pad_x) / scale, (kp[2, 1] - pad_y) / scale)
            rot = _rotation(wrist[0], wrist[1], middle[0], middle[1])
            rois.append(_make_roi((x0 + x1) / 2, (y0 + y1) / 2,
                                  x1 - x0, y1 - y0, rot, 2.6, -0.5))
        return rois

    # ------------------------------------------------------------------ hand
    def _crop(self, image: np.ndarray, roi: _Roi) -> tuple[np.ndarray, np.ndarray]:
        cos, sin = math.cos(roi.rot), math.sin(roi.rot)
        half = roi.size / 2.0
        ux, uy = cos * half, sin * half        # rotated x axis
        vx, vy = -sin * half, cos * half       # rotated y axis
        src = np.float32([
            [roi.cx - ux - vx, roi.cy - uy - vy],   # top-left
            [roi.cx + ux - vx, roi.cy + uy - vy],   # top-right
            [roi.cx - ux + vx, roi.cy - uy + vy],   # bottom-left
        ])
        dst = np.float32([[0, 0], [HAND_SIZE, 0], [0, HAND_SIZE]])
        matrix = cv2.getAffineTransform(src, dst)
        cv2.warpAffine(image, matrix, (HAND_SIZE, HAND_SIZE),
                       dst=self._crop_buf, flags=cv2.INTER_LINEAR, borderValue=0)
        return self._crop_buf, matrix

    def _roi_from_landmarks(self, points: np.ndarray) -> _Roi:
        tip_x = (points[5, 0] + points[13, 0]) / 2.0
        tip_y = (points[5, 1] + points[13, 1]) / 2.0
        tip_x = (tip_x + points[9, 0]) / 2.0
        tip_y = (tip_y + points[9, 1]) / 2.0
        rot = _rotation(points[0, 0], points[0, 1], tip_x, tip_y)
        stable = points[list(ROI_LANDMARKS)]
        cos, sin = math.cos(rot), math.sin(rot)
        a = stable[:, 0] * cos + stable[:, 1] * sin
        b = -stable[:, 0] * sin + stable[:, 1] * cos
        ca, cb = (a.min() + a.max()) / 2.0, (b.min() + b.max()) / 2.0
        cx = ca * cos - cb * sin
        cy = ca * sin + cb * cos
        return _make_roi(cx, cy, a.max() - a.min(), b.max() - b.min(), rot, 2.0, -0.1)

    # --------------------------------------------------------------- process
    def process(self, image_rgb: np.ndarray) -> Any:
        image = np.ascontiguousarray(image_rgb)
        h, w = image.shape[:2]
        rois = list(self._rois)
        if len(rois) < self.num_hands:
            for roi in self._detect_palms(image):
                if any(
                    math.hypot(roi.cx - t.cx, roi.cy - t.cy)
                    < 0.5 * min(roi.size, t.size)
                    for t in rois
                ):
                    continue
                rois.append(roi)
                if len(rois) >= self.num_hands:
                    break

        landmarks_out, world_out, handed_out = [], [], []
        next_rois: list[_Roi] = []
        for roi in rois[: self.num_hands]:
            crop, matrix = self._crop(image, roi)
            self._hand_buf[0] = crop.transpose(2, 0, 1) / 255.0
            outs = self._hand.run(self._hand_buf)
            # hand outputs in ONNX order: [lm(63), score(1), handed(1), world(63)]
            lm, score, handed, world = outs[0], outs[1], outs[2], outs[3]
            if _as_prob(float(score.reshape(-1)[0])) < PRESENCE_THR:
                continue
            lm = lm.reshape(21, 3).astype(np.float32)
            inverse = cv2.invertAffineTransform(matrix)
            pts = lm[:, :2].copy()
            pts_frame = pts @ inverse[:, :2].T + inverse[:, 2]
            depth = lm[:, 2] / HAND_SIZE * (roi.size / w)
            landmarks_out.append(SimpleNamespace(landmark=[
                SimpleNamespace(x=float(px / w), y=float(py / h), z=float(pz))
                for (px, py), pz in zip(pts_frame, depth)
            ]))
            world = world.reshape(21, 3).astype(np.float32)
            cos, sin = math.cos(roi.rot), math.sin(roi.rot)
            wx = world[:, 0] * cos - world[:, 1] * sin
            wy = world[:, 0] * sin + world[:, 1] * cos
            world_out.append(SimpleNamespace(landmark=[
                SimpleNamespace(x=float(a), y=float(b), z=float(c))
                for a, b, c in zip(wx, wy, world[:, 2])
            ]))
            # MediaPipe convention (handedness.txt): raw score = P("Left")
            left_score = _as_prob(float(handed.reshape(-1)[0]))
            label = "Left" if left_score > 0.5 else "Right"
            handed_out.append(SimpleNamespace(classification=[SimpleNamespace(
                label=label, score=max(left_score, 1.0 - left_score)
            )]))
            next_rois.append(self._roi_from_landmarks(pts_frame))
        self._rois = next_rois
        if not landmarks_out:
            return SimpleNamespace(multi_hand_landmarks=None,
                                   multi_hand_world_landmarks=None,
                                   multi_handedness=None)
        return SimpleNamespace(multi_hand_landmarks=landmarks_out,
                               multi_hand_world_landmarks=world_out,
                               multi_handedness=handed_out)

    def close(self) -> None:
        for executor in (self._palm, self._hand):
            try:
                executor.close()
            except Exception:
                pass


register_gesture_backend(
    "mediapipe_lite_jetson",
    lambda **kwargs: MediaPipeLiteJetsonBackend(**kwargs),
    replace=True,
)
