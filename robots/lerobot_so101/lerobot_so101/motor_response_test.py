#!/usr/bin/env python3
"""Direct SO101 follower motor response test."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import time
from pathlib import Path
from typing import Sequence

from .so101 import SO101Follower, SO101FollowerConfig


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
LEROBOT_KEYS = [f"{name}.pos" for name in JOINT_NAMES]
ARM_KEYS = LEROBOT_KEYS[:5]
GRIPPER_KEY = LEROBOT_KEYS[-1]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Measure direct SO101 motor response without Teleop/RCP.")
    parser.add_argument("--port", required=True, help="Follower serial port, e.g. /dev/cu.usbmodem... or COM5")
    parser.add_argument("--joint", choices=JOINT_NAMES, default="shoulder_lift")
    parser.add_argument("--amplitude", type=float, default=0.20, help="Sine amplitude in rad, gripper uses ratio")
    parser.add_argument("--frequency", type=float, default=0.5, help="Sine frequency in Hz")
    parser.add_argument("--duration", type=float, default=8.0, help="Test duration in seconds")
    parser.add_argument("--control-hz", type=float, default=60.0)
    parser.add_argument("--settle-s", type=float, default=1.0)
    parser.add_argument("--out-dir", default="/tmp/so101_motor_response")
    parser.add_argument("--robot-id", default="so101_follower")
    parser.add_argument("--no-return", action="store_true", help="Do not command the initial pose at the end")
    args = parser.parse_args(argv)

    joint_index = JOINT_NAMES.index(args.joint)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = int(time.time())
    csv_path = out_dir / f"so101_motor_response_{args.joint}_{stamp}.csv"
    svg_path = out_dir / f"so101_motor_response_{args.joint}_{stamp}.svg"

    robot = SO101Follower(
        SO101FollowerConfig(
            port=args.port,
            id=args.robot_id,
            use_degrees=True,
            disable_torque_on_disconnect=True,
        )
    )
    rows: list[dict[str, float]] = []
    center: list[float] | None = None
    try:
        robot.connect(calibrate=False)
        center = _obs_to_positions(robot.get_observation())
        print(f"center: {[round(v, 6) for v in center]}", flush=True)
        period = 1.0 / max(1.0, args.control_hz)
        start = time.monotonic()
        next_tick = start

        while True:
            now = time.monotonic()
            elapsed = now - start
            if elapsed >= args.duration:
                break
            if next_tick > now:
                time.sleep(next_tick - now)
            target = list(center)
            target[joint_index] = _target_value(center[joint_index], args.amplitude, args.frequency, elapsed, args.joint)

            send_started = time.perf_counter()
            robot.send_action(_positions_to_action(target))
            send_ms = (time.perf_counter() - send_started) * 1000.0
            read_started = time.perf_counter()
            state = _obs_to_positions(robot.get_observation())
            read_ms = (time.perf_counter() - read_started) * 1000.0
            rows.append(
                {
                    "elapsed_s": elapsed,
                    "target": target[joint_index],
                    "state": state[joint_index],
                    "send_ms": send_ms,
                    "read_ms": read_ms,
                }
            )
            next_tick += period

        if center is not None and not args.no_return:
            robot.send_action(_positions_to_action(center))
            time.sleep(max(0.0, args.settle_s))
    finally:
        if robot.is_connected:
            robot.disconnect()

    _write_csv(csv_path, rows)
    _write_svg(svg_path, args.joint, rows)
    summary = _summarize(rows)
    print(f"csv: {csv_path}", flush=True)
    print(f"svg: {svg_path}", flush=True)
    print(f"summary: {summary}", flush=True)
    return 0


def _target_value(center: float, amplitude: float, frequency: float, elapsed: float, joint: str) -> float:
    value = float(center) + float(amplitude) * math.sin(2.0 * math.pi * float(frequency) * float(elapsed))
    if joint == "gripper":
        return max(0.0, min(1.0, value))
    return value


def _obs_to_positions(obs: dict[str, float]) -> list[float]:
    arm_positions = [math.radians(float(obs[key])) for key in ARM_KEYS]
    gripper_ratio = max(0.0, min(1.0, float(obs[GRIPPER_KEY]) / 100.0))
    return arm_positions + [gripper_ratio]


def _positions_to_action(positions: Sequence[float]) -> dict[str, float]:
    action = {key: math.degrees(float(value)) for key, value in zip(ARM_KEYS, positions[:5])}
    action[GRIPPER_KEY] = max(0.0, min(1.0, float(positions[-1]))) * 100.0
    return action


def _write_csv(path: Path, rows: list[dict[str, float]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=["elapsed_s", "target", "state", "send_ms", "read_ms"])
        writer.writeheader()
        for row in rows:
            writer.writerow({key: f"{value:.9g}" for key, value in row.items()})


def _summarize(rows: list[dict[str, float]]) -> dict[str, float]:
    if len(rows) < 3:
        return {}
    times = [row["elapsed_s"] for row in rows]
    gaps = [b - a for a, b in zip(times, times[1:]) if b > a]
    lag_ms, rmse = _best_lag(rows)
    target_span = _span(row["target"] for row in rows)
    state_span = _span(row["state"] for row in rows)
    return {
        "samples": float(len(rows)),
        "actual_hz": round(1.0 / statistics.fmean(gaps), 2) if gaps else 0.0,
        "lag_ms": float(lag_ms),
        "rmse": round(rmse, 5),
        "amplitude_ratio": round(state_span / target_span, 3) if target_span > 0 else 0.0,
        "send_p95_ms": round(_percentile([row["send_ms"] for row in rows], 0.95), 3),
        "read_p95_ms": round(_percentile([row["read_ms"] for row in rows], 0.95), 3),
    }


def _best_lag(rows: list[dict[str, float]]) -> tuple[int, float]:
    best_lag = 0
    best_error = float("inf")
    for lag_ms in range(-500, 501, 5):
        error = _lag_rmse(rows, lag_ms / 1000.0)
        if error < best_error:
            best_lag = lag_ms
            best_error = error
    return best_lag, best_error


def _lag_rmse(rows: list[dict[str, float]], lag_s: float) -> float:
    start = max(rows[0]["elapsed_s"], rows[0]["elapsed_s"] - lag_s)
    end = min(rows[-1]["elapsed_s"], rows[-1]["elapsed_s"] - lag_s)
    if end <= start:
        return float("inf")
    step = 0.01
    count = int((end - start) / step)
    if count < 10:
        return float("inf")
    total = 0.0
    for index in range(count):
        t = start + index * step
        diff = _interp(rows, t, "target") - _interp(rows, t + lag_s, "state")
        total += diff * diff
    return math.sqrt(total / count)


def _interp(rows: list[dict[str, float]], t: float, key: str) -> float:
    if t <= rows[0]["elapsed_s"]:
        return rows[0][key]
    if t >= rows[-1]["elapsed_s"]:
        return rows[-1][key]
    lo, hi = 0, len(rows) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if rows[mid]["elapsed_s"] <= t:
            lo = mid
        else:
            hi = mid
    t0 = rows[lo]["elapsed_s"]
    t1 = rows[hi]["elapsed_s"]
    alpha = (t - t0) / (t1 - t0) if t1 != t0 else 0.0
    return rows[lo][key] * (1.0 - alpha) + rows[hi][key] * alpha


def _span(values: Sequence[float]) -> float:
    data = list(values)
    return max(data) - min(data) if data else 0.0


def _percentile(values: list[float], percentile: float) -> float:
    data = sorted(values)
    if not data:
        return 0.0
    return data[int(float(percentile) * (len(data) - 1))]


def _write_svg(path: Path, joint: str, rows: list[dict[str, float]]) -> None:
    width, height = 1000, 360
    left, right, top, bottom = 70, 20, 30, 40
    plot_w = width - left - right
    plot_h = height - top - bottom
    max_t = rows[-1]["elapsed_s"] if rows else 1.0
    values = [row["target"] for row in rows] + [row["state"] for row in rows]
    low, high = min(values), max(values)
    if math.isclose(low, high):
        low -= 1.0
        high += 1.0
    pad = (high - low) * 0.08
    low -= pad
    high += pad

    def points(key: str) -> str:
        return " ".join(
            f"{left + row['elapsed_s'] / max_t * plot_w:.1f},{top + (high - row[key]) / (high - low) * plot_h:.1f}"
            for row in rows
        )

    svg = "\n".join(
        [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            '<rect width="100%" height="100%" fill="#fff"/>',
            f'<text x="16" y="22" font-family="monospace" font-size="14">SO101 motor response: {joint}</text>',
            '<text x="780" y="22" font-family="monospace" font-size="12" fill="#d62728">target</text>',
            '<text x="850" y="22" font-family="monospace" font-size="12" fill="#1f77b4">state</text>',
            f'<line x1="{left}" y1="{height-bottom}" x2="{width-right}" y2="{height-bottom}" stroke="#ddd"/>',
            f'<line x1="{left}" y1="{top}" x2="{left}" y2="{height-bottom}" stroke="#ddd"/>',
            f'<text x="14" y="{top + 20}" font-family="monospace" font-size="11">{low:.3g}..{high:.3g}</text>',
            f'<polyline points="{points("target")}" fill="none" stroke="#d62728" stroke-width="1.5"/>',
            f'<polyline points="{points("state")}" fill="none" stroke="#1f77b4" stroke-width="1.5"/>',
            f'<text x="{left}" y="{height-12}" font-family="monospace" font-size="11">0s</text>',
            f'<text x="{width-80}" y="{height-12}" font-family="monospace" font-size="11">{max_t:.2f}s</text>',
            "</svg>",
        ]
    )
    path.write_text(svg, encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
