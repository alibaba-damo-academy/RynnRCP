"""C++ MTNN zero-copy backend: ``mediapipe_lite_mtnn_zero``.

The entire cascade (preprocessing, NPU inference, SSD decode, NMS, hand
landmark, tracking) runs in C++ with zero-copy I/O via the MTNN C API.
Exposed via pybind11 as ``aero_hand_mtnn.AeroHandPipeline``.
"""
from __future__ import annotations

import os
from types import SimpleNamespace
from typing import Any

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


class MtnnZeroBackend:
    """C++ MTNN zero-copy pipeline exposed via pybind11."""

    name = "mediapipe_lite_mtnn_zero"
    accepts_bgr = True

    def __init__(self, *, mediapipe: Any = None, max_num_hands: int = 1,
                 num_threads: int = 1, palm_model: str | None = None,
                 hand_model: str | None = None,
                 name: str | None = None, **_ignored: Any) -> None:
        from rynnrcp_robot_aero_hand.accelerators import aero_hand_mtnn

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

        self._pipeline = aero_hand_mtnn.AeroHandPipeline(
            palm_path, hand_path, self.num_hands,
        )
        self._clones: list[Any] = []
        self._rr_idx = 0

    def process(self, image_rgb: np.ndarray) -> Any:
        frame = np.ascontiguousarray(image_rgb)
        hands = self._pipeline.process(frame, 0)
        return self._to_mediapipe(hands)

    def process_bgr(self, image_bgr: np.ndarray) -> Any:
        frame = np.ascontiguousarray(image_bgr)
        hands = self._pipeline.process(frame, 1)
        return self._to_mediapipe(hands)

    @staticmethod
    def _to_mediapipe(hands: list) -> Any:
        if not hands:
            return SimpleNamespace(multi_hand_landmarks=None,
                                   multi_hand_world_landmarks=None,
                                   multi_handedness=None)
        landmarks_out, world_out, handed_out = [], [], []
        for h in hands:
            lm = h["image"]
            wd = h["world"]
            score = float(h["score"])
            label = h["label"]
            landmarks_out.append(SimpleNamespace(landmark=[
                SimpleNamespace(x=float(lm[i, 0]), y=float(lm[i, 1]), z=float(lm[i, 2]))
                for i in range(21)
            ]))
            world_out.append(SimpleNamespace(landmark=[
                SimpleNamespace(x=float(wd[i, 0]), y=float(wd[i, 1]), z=float(wd[i, 2]))
                for i in range(21)
            ]))
            handed_out.append(SimpleNamespace(classification=[
                SimpleNamespace(label=label, score=score)
            ]))
        return SimpleNamespace(multi_hand_landmarks=landmarks_out,
                               multi_hand_world_landmarks=world_out,
                               multi_handedness=handed_out)

    def close(self) -> None:
        try:
            self._pipeline.release()
        except Exception:
            pass


register_gesture_backend(
    "mediapipe_lite_mtnn_zero",
    lambda **kwargs: MtnnZeroBackend(**kwargs),
    replace=True,
)
