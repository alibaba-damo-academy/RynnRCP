"""MediaPipe Lite cascade with MTNN (Moore Threads NPU) inference.

Backend ``mediapipe_lite_mtnn`` targets the E300 / M1000 NPU via the MTNN
toolkit (MTNNRT runtime).  The cascade mirrors ``mediapipe_lite_rknn`` and
uses the official MediaPipe models exported from ``*_full.tflite`` to ONNX
then converted to .mtnn via MTC (int16 quantization for accuracy):

  frame RGB -> letterbox 192 -> palm int16 (NPU) -> SSD 2016-anchor decode
            + weighted-NMS -> rotated square ROI (wrist->middle, x2.6, shift -0.5)
            -> affine crop 224 -> hand int16 (NPU) -> landmark projection back
  tracking  -> next-frame ROI from previous landmarks (x2.0, shift -0.1);
               palm reruns only when tracked hands < num_hands.

MTNN models take float32 NCHW input normalized to [0,1] (div 255):
palm [1,3,192,192], hand [1,3,224,224].

Model preparation: export MediaPipe ``palm_detection_full.tflite`` /
``hand_landmark_full.tflite`` to ONNX (tf2onnx / tflite2onnx), then convert
with ``mtc`` using ``quant_type: int16`` + ``quantizer: dynamic_fixed_point``
and real-image calibration (div255).
"""
from __future__ import annotations

import math
import os
from types import SimpleNamespace
from typing import Any

import cv2
import numpy as np

from rynnrcp_robot_aero_hand import model_store
from rynnrcp_robot_aero_hand.gesture_inference import register_gesture_backend

_MODEL_DIR = os.environ.get(
    "AERO_HAND_MTNN_DIR",
    os.path.join(model_store.models_root(), "mtnn"),
)
PALM_MTNN = os.environ.get(
    "AERO_HAND_MTNN_PALM", os.path.join(_MODEL_DIR, "palm_detection.mtnn"),
)
HAND_MTNN = os.environ.get(
    "AERO_HAND_MTNN_HAND", os.path.join(_MODEL_DIR, "hand_landmark.mtnn"),
)

PALM_SIZE = 192
HAND_SIZE = 224
PALM_SCORE_THR = 0.5
PRESENCE_THR = 0.5
NMS_IOU = 0.3
# Official HandLandmarksToRectCalculator: the tracking ROI is built from the
# 12 stable palm/knuckle landmarks only (fingertips excluded).
ROI_LANDMARKS = (0, 1, 2, 3, 5, 6, 9, 10, 13, 14, 17, 18)


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


class MediaPipeLiteMtnnBackend:
    """MediaPipe Lite pipeline with palm/hand inference on the Moore Threads NPU."""

    name = "mediapipe_lite_mtnn"

    def __init__(self, *, mediapipe: Any = None, max_num_hands: int = 1,
                 num_threads: int = 1, palm_model: str | None = None,
                 hand_model: str | None = None,
                 name: str | None = None, **_ignored: Any) -> None:
        from mtnn_api import MTNNSession

        if name is not None:
            self.name = name
        palm_path = palm_model or PALM_MTNN
        hand_path = hand_model or HAND_MTNN
        model_store.ensure_backend_files((palm_path, hand_path))
        for path in (palm_path, hand_path):
            if not os.path.isfile(path):
                raise FileNotFoundError(f"mtnn model not found: {path}")

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
        self._palm = MTNNSession(palm_path)
        self._hand = MTNNSession(hand_path)
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
        # normalize to [0,1], NCHW
        self._palm_buf[0] = canvas.transpose(2, 0, 1) / 255.0
        outs = self._palm.run({0: self._palm_buf})
        # MTNN may emit packed layouts; element order matches the flat
        # (2016,18)/(2016,) tensors (largest tensor = regressor).
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
            # letterbox(192) -> frame pixels
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
            outs = self._hand.run({0: self._hand_buf})
            # hand outputs: [lm(63), score(1), handed(1), world(63)]
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
        for session in (self._palm, self._hand):
            try:
                del session
            except Exception:
                pass


register_gesture_backend(
    "mediapipe_lite_mtnn",
    lambda **kwargs: MediaPipeLiteMtnnBackend(**kwargs),
    replace=True,
)
