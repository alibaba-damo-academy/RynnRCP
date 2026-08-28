"""Checks for the reusable Aero Hand gesture benchmark dataset."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest


BENCHMARK_PATH = (
    Path(__file__).resolve().parents[2]
    / "robots"
    / "tetheria_aerohand"
    / "benchmark_gesture_inference.py"
)
SPEC = importlib.util.spec_from_file_location("aero_hand_gesture_benchmark", BENCHMARK_PATH)
assert SPEC is not None and SPEC.loader is not None
benchmark = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = benchmark
SPEC.loader.exec_module(benchmark)


def test_candidate_spec_supports_multiple_named_backends() -> None:
    candidate = benchmark._parse_candidate_spec(
        "rknn_int8=rynnrcp_robot_aero_hand.accelerators.rknn_int8@2"
    )

    assert candidate.backend == "rknn_int8"
    assert candidate.backend_module == (
        "rynnrcp_robot_aero_hand.accelerators.rknn_int8"
    )
    assert candidate.threads == 2
    assert candidate.name == "rknn_int8-640-t2"


@pytest.mark.parametrize("value", ["rknn", "rknn=module", "rknn=module@0"])
def test_candidate_spec_rejects_incomplete_values(value: str) -> None:
    with pytest.raises(ValueError):
        benchmark._parse_candidate_spec(value)


def test_recorded_dataset_round_trips_exact_frames(tmp_path: Path) -> None:
    clips = {
        "single": [
            np.full((4, 6, 3), 10, dtype=np.uint8),
            np.full((4, 6, 3), 20, dtype=np.uint8),
        ],
        "dual": [np.full((4, 6, 3), 30, dtype=np.uint8)],
    }
    camera = {
        "width": 640,
        "height": 480,
        "sample_fps": 15.0,
    }

    directory = benchmark._save_dataset(
        np=np,
        directory=tmp_path / "dataset",
        clips=clips,
        camera=camera,
    )
    loaded, manifest = benchmark._load_dataset(
        np=np,
        directory=directory,
        profiles=("single", "dual"),
    )

    assert manifest["format"] == "aero_hand_gesture_frames_v1"
    assert manifest["camera"] == camera
    assert np.array_equal(loaded["single"], np.stack(clips["single"]))
    assert np.array_equal(loaded["dual"], np.stack(clips["dual"]))


def test_recorded_dataset_detects_modified_frame_file(tmp_path: Path) -> None:
    directory = benchmark._save_dataset(
        np=np,
        directory=tmp_path / "dataset",
        clips={"single": [np.zeros((2, 2, 3), dtype=np.uint8)]},
        camera={},
    )
    with (directory / "single.npz").open("ab") as stream:
        stream.write(b"changed")

    with pytest.raises(ValueError, match="checksum mismatch"):
        benchmark._load_dataset(
            np=np,
            directory=directory,
            profiles=("single",),
        )
