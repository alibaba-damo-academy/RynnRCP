"""Pluggable gesture-inference backends for Aero Hand.

The built-in baseline deliberately stays portable: MediaPipe Legacy Lite,
640x480 input and one CPU inference thread. Hardware-specific acceleration
can be added by registering another backend without changing the camera,
retargeting or controller code.
"""

from __future__ import annotations

import importlib
import logging
from pathlib import Path
from typing import Any, Callable, Protocol


logger = logging.getLogger(__name__)

# "auto" resolves to the best accelerator for the current device via
# platform_detect (RKNN/MTNN/TensorRT NPU/GPU backends when present,
# otherwise the portable mediapipe_lite CPU baseline).
DEFAULT_GESTURE_BACKEND = "auto"


class GestureInferenceBackend(Protocol):
    """Minimal contract consumed by the shared Aero Hand vision pipeline."""

    name: str
    connections: Any
    model_bytes: int

    def process(self, image_rgb: Any) -> Any:
        """Return MediaPipe-compatible hand landmarks for one RGB frame."""

    def close(self) -> None:
        """Release inference resources."""


BackendFactory = Callable[..., GestureInferenceBackend]
_BACKEND_FACTORIES: dict[str, BackendFactory] = {}


def load_gesture_backend_module(module_name: str | None) -> None:
    """Import an optional module whose top level registers a backend."""

    if module_name and str(module_name).strip():
        importlib.import_module(str(module_name).strip())


def available_gesture_backends() -> tuple[str, ...]:
    """Return registered backend names for validation and diagnostics."""

    return tuple(sorted(_BACKEND_FACTORIES))


def register_gesture_backend(
    name: str,
    factory: BackendFactory,
    *,
    replace: bool = False,
) -> None:
    """Register a backend factory for hardware-specific acceleration tests."""

    normalized = str(name).strip().lower()
    if not normalized:
        raise ValueError("gesture backend name must not be empty")
    if normalized in _BACKEND_FACTORIES and not replace:
        raise ValueError(f"gesture backend is already registered: {normalized}")
    _BACKEND_FACTORIES[normalized] = factory


def create_gesture_backend(
    name: str = DEFAULT_GESTURE_BACKEND,
    *,
    module_name: str | None = None,
    mediapipe: Any,
    max_num_hands: int,
    num_threads: int = 1,
) -> GestureInferenceBackend:
    """Create a configured backend using the common pipeline contract."""

    load_gesture_backend_module(module_name)
    normalized = str(name).strip().lower()
    if normalized == "auto":
        try:
            from .platform_detect import detect_platform, recommended_backend

            platform = detect_platform()
            normalized, auto_module = recommended_backend(platform)
            load_gesture_backend_module(auto_module)
            logger.info(
                "gesture backend auto-selected for platform %s: %s",
                platform, normalized,
            )
        except Exception as exc:  # noqa: BLE001 - keep the CPU baseline usable
            normalized = "mediapipe_lite"
            logger.warning(
                "gesture backend auto-selection failed (%s); "
                "falling back to mediapipe_lite", exc,
            )
    try:
        factory = _BACKEND_FACTORIES[normalized]
    except KeyError as exc:
        available = ", ".join(sorted(_BACKEND_FACTORIES))
        raise ValueError(
            f"unknown gesture inference backend {name!r}; available: {available}"
        ) from exc
    return factory(
        mediapipe=mediapipe,
        max_num_hands=int(max_num_hands),
        num_threads=max(1, int(num_threads)),
    )


class MediaPipeLiteBackend:
    """MediaPipe Legacy Lite graph with an explicit CPU thread count."""

    name = "mediapipe_lite"

    def __init__(
        self,
        *,
        mediapipe: Any,
        max_num_hands: int,
        num_threads: int = 1,
    ) -> None:
        from mediapipe.calculators.tensor import inference_calculator_pb2
        from mediapipe.framework import calculator_pb2
        from mediapipe.python import solution_base
        from mediapipe.python._framework_bindings import validated_graph_config

        if not hasattr(mediapipe, "solutions"):
            raise RuntimeError(
                "Aero Hand Lite requires mediapipe==0.10.18; run setup_aero_hand.sh"
            )
        hands = mediapipe.solutions.hands
        self.connections = hands.HAND_CONNECTIONS
        package_dir = Path(mediapipe.__file__).resolve().parent
        self.model_bytes = sum(
            path.stat().st_size
            for path in (
                package_dir
                / "modules"
                / "palm_detection"
                / "palm_detection_lite.tflite",
                package_dir
                / "modules"
                / "hand_landmark"
                / "hand_landmark_lite.tflite",
            )
            if path.is_file()
        )
        graph_path = (
            package_dir
            / "modules"
            / "hand_landmark"
            / "hand_landmark_tracking_cpu.binarypb"
        )
        validated = validated_graph_config.ValidatedGraphConfig()
        validated.initialize(binary_graph_path=str(graph_path))
        config = calculator_pb2.CalculatorGraphConfig()
        config.ParseFromString(validated.binary_config)
        configured_nodes = 0
        for node in config.node:
            if node.calculator != "InferenceCalculatorCpu":
                continue
            options = node.options.Extensions[
                inference_calculator_pb2.InferenceCalculatorOptions.ext
            ]
            options.delegate.xnnpack.num_threads = max(1, int(num_threads))
            configured_nodes += 1
        if configured_nodes != 2:
            raise RuntimeError(
                f"expected two MediaPipe Lite inference nodes, found {configured_nodes}"
            )
        self._solution = solution_base.SolutionBase(
            graph_config=config,
            side_inputs={
                "model_complexity": 0,
                "num_hands": int(max_num_hands),
                "use_prev_landmarks": True,
            },
            calculator_params={
                "palmdetectioncpu__TensorsToDetectionsCalculator.min_score_thresh": 0.5,
                "handlandmarkcpu__ThresholdingCalculator.threshold": 0.5,
            },
            outputs=[
                "multi_hand_landmarks",
                "multi_hand_world_landmarks",
                "multi_handedness",
            ],
        )

    def process(self, image_rgb: Any) -> Any:
        return self._solution.process({"image": image_rgb})

    def close(self) -> None:
        self._solution.close()


register_gesture_backend(
    "mediapipe_lite",
    lambda **kwargs: MediaPipeLiteBackend(**kwargs),
)
