"""Three-omniwheel kinematics used by LeKiwi."""

from __future__ import annotations

import math
from collections.abc import Mapping


WHEEL_NAMES = ("base_left_wheel", "base_back_wheel", "base_right_wheel")
WHEEL_ANGLES_RAD = tuple(math.radians(angle - 90.0) for angle in (240.0, 0.0, 120.0))
ENCODER_STEPS_PER_TURN = 4096.0


def body_to_wheel_raw(
    x_m_s: float,
    y_m_s: float,
    yaw_rad_s: float,
    *,
    wheel_radius_m: float,
    base_radius_m: float,
    max_raw: int,
) -> dict[str, int]:
    """Convert body velocity to signed STS3215 wheel velocity commands."""
    _validate_geometry(wheel_radius_m, base_radius_m, max_raw)
    wheel_linear = [
        math.cos(angle) * float(x_m_s)
        + math.sin(angle) * float(y_m_s)
        + float(base_radius_m) * float(yaw_rad_s)
        for angle in WHEEL_ANGLES_RAD
    ]
    raw = [speed / float(wheel_radius_m) * ENCODER_STEPS_PER_TURN / math.tau for speed in wheel_linear]
    peak = max(abs(value) for value in raw)
    if peak > max_raw:
        scale = float(max_raw) / peak
        raw = [value * scale for value in raw]
    return {name: int(round(value)) for name, value in zip(WHEEL_NAMES, raw)}


def wheel_raw_to_body(
    wheel_raw: Mapping[str, int | float],
    *,
    wheel_radius_m: float,
    base_radius_m: float,
) -> dict[str, list[float]]:
    """Convert signed STS3215 wheel feedback to standard RCP base velocity."""
    _validate_geometry(wheel_radius_m, base_radius_m, 1)
    wheel_linear = [
        float(wheel_raw[name]) * math.tau / ENCODER_STEPS_PER_TURN * float(wheel_radius_m)
        for name in WHEEL_NAMES
    ]
    x_m_s = (2.0 / 3.0) * sum(
        math.cos(angle) * speed for angle, speed in zip(WHEEL_ANGLES_RAD, wheel_linear)
    )
    y_m_s = (2.0 / 3.0) * sum(
        math.sin(angle) * speed for angle, speed in zip(WHEEL_ANGLES_RAD, wheel_linear)
    )
    yaw_rad_s = sum(wheel_linear) / (3.0 * float(base_radius_m))
    return {
        "linear_vel": [x_m_s, y_m_s, 0.0],
        "angular_vel": [0.0, 0.0, yaw_rad_s],
    }


def _validate_geometry(wheel_radius_m: float, base_radius_m: float, max_raw: int) -> None:
    if not math.isfinite(float(wheel_radius_m)) or float(wheel_radius_m) <= 0.0:
        raise ValueError("wheel_radius_m must be positive")
    if not math.isfinite(float(base_radius_m)) or float(base_radius_m) <= 0.0:
        raise ValueError("base_radius_m must be positive")
    if int(max_raw) <= 0 or int(max_raw) > 0x7FFF:
        raise ValueError("max_raw must be in [1, 32767]")
