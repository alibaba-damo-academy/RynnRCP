"""SO101 preset motion trajectories used for calibration validation."""

from __future__ import annotations

import math
from collections.abc import Sequence


PRESET_MOTION_FPS = 30
PRESET_MOTION_APPROACH_S = 2.0
PRESET_MOTION_DEFAULT_DURATION_S = 6.0
PRESET_MOTION_AMPLITUDE = 0.5
PRESET_MOTION_FREQUENCY = 0.2
PRESET_MOTION_IDS = {1, 2, 3, 4, 5}


def preset_motion_frame(motion: int, elapsed_s: float, initial: Sequence[float]) -> list[float]:
    duration = PRESET_MOTION_APPROACH_S
    amplitude = PRESET_MOTION_AMPLITUDE
    frequency = PRESET_MOTION_FREQUENCY
    initial_raw = list(initial[:5]) + [_clamp(float(initial[-1]), 0.0, 1.0) * 100.0]

    if motion == 1:
        target = [0.0, -math.pi / 4.0, math.pi / 10.0, 0.0, 0.0, 0.0]
        if elapsed_s > duration:
            t = elapsed_s - duration
            target[0] += amplitude * math.sin(2 * math.pi * frequency * t)
            target[1] += 0.8 * amplitude * math.sin(3 * math.pi * frequency * t)
            target[2] += 1.2 * amplitude * math.sin(4 * math.pi * frequency * t)
            target[5] += 30 - 30 * math.cos(2 * math.pi * frequency * t)
    elif motion == 2:
        target = [amplitude, 0.0, 0.0, 0.0, 0.0, 0.0]
        if elapsed_s > duration:
            t = elapsed_s - duration
            target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            target[0] += amplitude * math.cos(2 * math.pi * frequency * t)
            target[2] += amplitude * math.sin(2 * math.pi * frequency * t)
    elif motion == 3:
        target = [
            1.5 * amplitude,
            -1.5 * amplitude + math.pi / 2.0,
            amplitude - math.pi / 2.0,
            amplitude,
            0.0,
            0.0,
        ]
        if elapsed_s > duration:
            t = elapsed_s - duration
            target = [0.0, -1.5 * amplitude + math.pi / 2.0, -math.pi / 2.0, 0.0, 0.0, 0.0]
            target[0] += 1.5 * amplitude * math.cos(2 * math.pi * frequency * t)
            target[2] += amplitude * math.cos(4 * math.pi * frequency * t)
            target[3] += amplitude * math.cos(4 * math.pi * frequency * t)
    elif motion == 4:
        target = [0.5 * math.pi, 0.0, -math.pi / 2.0, 0.0, 0.0, 30.0]
        if elapsed_s > duration:
            t = elapsed_s - duration
            target[1] += amplitude * math.sin(2 * math.pi * frequency * t)
            target[3] += amplitude * math.sin(2 * math.pi * frequency * t)
            target[4] += 2 * amplitude * math.sin(2 * math.pi * frequency * t)
    elif motion == 5:
        target = [0.25 * math.pi, -0.2 * math.pi, 0.2 * math.pi, 0.25 * math.pi, -0.5 * math.pi, 30.0]
    else:
        raise ValueError(f"preset_motion.motion must be one of {sorted(PRESET_MOTION_IDS)}")

    if elapsed_s <= duration:
        ratio = max(0.0, min(1.0, elapsed_s / duration))
        target = [start + (end - start) * ratio for start, end in zip(initial_raw, target)]
    return [float(v) for v in target[:5]] + [_clamp(float(target[-1]) / 100.0, 0.0, 1.0)]


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
