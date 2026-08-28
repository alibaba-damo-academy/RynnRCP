"""Gesture backend ``rga_rknn_zero``: RGA preprocessing + RKNN zero-copy NPU.

The whole cascade (letterbox -> palm -> ROI -> hand -> tracking) runs inside
the C++ pybind module :mod:`aero_hand_rga` built from
``robots/tetheria_aerohand/src`` (see ``src/build.sh``):

- palm letterbox (+ optional BGR->RGB) on the RGA 2D hardware, written
  directly into the palm RKNN zero-copy input buffer;
- frame pointers imported into RGA once and cached (ptr -> handle);
- bundled librknnrt 2.3.2 resolved via $ORIGIN -- no system lib replacement
  or user-namespace bind mount required.
"""
from __future__ import annotations

import os
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
from typing import Any

import numpy as np

from rynnrcp_robot_aero_hand import model_store
from rynnrcp_robot_aero_hand.gesture_inference import register_gesture_backend
from rynnrcp_robot_aero_hand.platform_detect import detect_platform

_MODEL_DIR = os.environ.get(
    "AERO_HAND_RKNN_DIR",
    model_store.resolve_model_dir(detect_platform()),
)
# Default full fp16 exports: jitter/accuracy on par with mediapipe legacy
# full (matrix-tested on RK3566), 86fps with 2 pipelined workers.
PALM_RKNN = os.environ.get(
    "AERO_HAND_RKNN_PALM",
    os.path.join(_MODEL_DIR, "palm_detection_full_fp16_rknn2.3.0.rknn"),
)
HAND_RKNN = os.environ.get(
    "AERO_HAND_RKNN_HAND",
    os.path.join(_MODEL_DIR, "hand_landmark_full_fp16_rknn2.3.0.rknn"),
)

_FORMAT_RGB = 0
_FORMAT_BGR = 1


def _landmarks(points: np.ndarray) -> SimpleNamespace:
    return SimpleNamespace(landmark=[
        SimpleNamespace(x=float(x), y=float(y), z=float(z)) for x, y, z in points
    ])


class RgaRknnZeroBackend:
    """MediaPipe-compatible backend backed by the C++ RGA/RKNN pipeline."""

    name = "rga_rknn_zero"
    accepts_bgr = True  # C++ side can color-convert on the RGA if fed BGR

    def __init__(self, *, mediapipe: Any = None, max_num_hands: int = 1,
                 num_threads: int = 1, **_ignored: Any) -> None:
        from rynnrcp_robot_aero_hand.accelerators import aero_hand_rga

        self.connections = (
            mediapipe.solutions.hands.HAND_CONNECTIONS
            if mediapipe is not None and hasattr(mediapipe, "solutions")
            else ()
        )
        self.model_bytes = sum(
            os.path.getsize(p) for p in (PALM_RKNN, HAND_RKNN) if os.path.isfile(p)
        )
        # Worker pool for NPU throughput (pipeline parallelism). num_threads=1
        # (production default) is a plain synchronous passthrough. For N>1 we
        # keep N pipeline instances -- clones share the model WEIGHTS via
        # rknn_dup_context -- and round-robin frames across an executor: while
        # one worker waits on the NPU, another runs its CPU stages. Results
        # then lag the input by N-1 frames (pipeline latency).
        self._depth = max(1, int(num_threads))
        model_store.ensure_backend_files((PALM_RKNN, HAND_RKNN))
        primary = aero_hand_rga.AeroHandPipeline(
            PALM_RKNN, HAND_RKNN, max(1, int(max_num_hands))
        )
        self._pipes = [primary] + [primary.clone() for _ in range(self._depth - 1)]
        self._pipe = primary  # kept for stats()/tests
        self._executor = (
            ThreadPoolExecutor(max_workers=self._depth) if self._depth > 1 else None
        )
        self._pending: deque = deque()
        self._round_robin = 0

    def _run(self, frame: np.ndarray, fmt: int) -> Any:
        if self._depth == 1:
            return self._to_result(self._pipes[0].process(frame, fmt))
        pipe = self._pipes[self._round_robin]
        self._round_robin = (self._round_robin + 1) % self._depth
        self._pending.append(self._executor.submit(pipe.process, frame, fmt))
        if len(self._pending) < self._depth:  # pipeline warm-up
            return self._to_result([])
        return self._to_result(self._pending.popleft().result())

    def process(self, image_rgb: np.ndarray) -> Any:
        return self._run(np.ascontiguousarray(image_rgb), _FORMAT_RGB)

    def process_bgr(self, image_bgr: np.ndarray) -> Any:
        """Deployment path: raw BGR in, BGR->RGB done by the RGA hardware."""
        return self._run(np.ascontiguousarray(image_bgr), _FORMAT_BGR)

    @staticmethod
    def _to_result(hands) -> Any:
        if not hands:
            return SimpleNamespace(multi_hand_landmarks=None,
                                   multi_hand_world_landmarks=None,
                                   multi_handedness=None)
        return SimpleNamespace(
            multi_hand_landmarks=[_landmarks(h["image"]) for h in hands],
            multi_hand_world_landmarks=[_landmarks(h["world"]) for h in hands],
            multi_handedness=[
                SimpleNamespace(classification=[
                    SimpleNamespace(label=h["label"], score=float(h["score"]))
                ])
                for h in hands
            ],
        )

    def stats(self):
        return self._pipe.stats()

    def close(self) -> None:
        while self._pending:
            try:
                self._pending.popleft().result()
            except Exception:
                pass
        if self._executor is not None:
            self._executor.shutdown(wait=True)
        for pipe in self._pipes:
            pipe.release()


register_gesture_backend(
    "rga_rknn_zero",
    lambda **kwargs: RgaRknnZeroBackend(**kwargs),
    replace=True,
)
