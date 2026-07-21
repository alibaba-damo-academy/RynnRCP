"""Optional SO101 position/timing trace writer."""

from __future__ import annotations

import atexit
import csv
import math
import time
from pathlib import Path
from threading import RLock
from typing import Sequence


class PositionTrace:
    def __init__(
        self,
        *,
        enabled: bool,
        trace_dir: str,
        robot_id: str,
        role: str,
        joint_names: Sequence[str],
    ) -> None:
        self.enabled = bool(enabled)
        self._dir = Path(trace_dir or ".").resolve()
        self._robot_id = str(robot_id)
        self._role = str(role)
        self._joint_names = list(joint_names)
        self._started_mono = time.monotonic()
        self._positions: list[tuple[float, str, list[float]]] = []
        self._timings: list[tuple[float, str, float]] = []
        self._lock = RLock()
        self._flushed = False
        if self.enabled:
            atexit.register(self.flush)

    def record_position(self, kind: str, positions: Sequence[float]) -> None:
        if not self.enabled:
            return
        values = [float(v) for v in positions]
        if len(values) != len(self._joint_names):
            return
        with self._lock:
            self._positions.append((time.monotonic() - self._started_mono, str(kind), values))

    def record_timing(self, kind: str, duration_ms: float) -> None:
        if not self.enabled:
            return
        with self._lock:
            self._timings.append((time.monotonic() - self._started_mono, str(kind), float(duration_ms)))

    def flush(self) -> None:
        if not self.enabled:
            return
        with self._lock:
            if self._flushed:
                return
            self._flushed = True
            positions = list(self._positions)
            timings = list(self._timings)
        if not positions:
            print(f"[SO101 Trace] no samples: robot_id={self._robot_id} role={self._role}", flush=True)
            return

        self._dir.mkdir(parents=True, exist_ok=True)
        base = self._dir / f"so101_trace_{_slug(self._robot_id)}_{self._role}_{int(time.time())}"
        csv_path = base.with_suffix(".csv")
        timing_path = base.with_suffix(".timing.csv")
        svg_path = base.with_suffix(".svg")
        _write_position_csv(csv_path, self._joint_names, positions)
        _write_timing_csv(timing_path, timings)
        _write_position_svg(svg_path, self._joint_names, positions)
        print(f"[SO101 Trace] saved: {csv_path}", flush=True)
        print(f"[SO101 Trace] saved: {timing_path}", flush=True)
        print(f"[SO101 Trace] saved: {svg_path}", flush=True)
        print(f"[SO101 Trace] last action: {_last_values(positions, 'action')}", flush=True)
        print(f"[SO101 Trace] last state:  {_last_values(positions, 'state')}", flush=True)
        print(f"[SO101 Trace] timing: {_timing_summary(timings)}", flush=True)


def _write_position_csv(path: Path, joint_names: list[str], positions: list[tuple[float, str, list[float]]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["elapsed_s", "kind", *joint_names])
        for elapsed_s, kind, values in positions:
            writer.writerow([f"{elapsed_s:.6f}", kind, *[f"{value:.9g}" for value in values]])


def _write_timing_csv(path: Path, timings: list[tuple[float, str, float]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["elapsed_s", "kind", "duration_ms"])
        for elapsed_s, kind, duration_ms in timings:
            writer.writerow([f"{elapsed_s:.6f}", kind, f"{duration_ms:.6f}"])


def _write_position_svg(path: Path, joint_names: list[str], positions: list[tuple[float, str, list[float]]]) -> None:
    width = 1100
    row_h = 132
    left = 78
    right = 24
    top = 36
    plot_h = 86
    height = top + row_h * len(joint_names) + 34
    max_t = max(elapsed_s for elapsed_s, _, _ in positions) or 1.0

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<text x="16" y="22" font-family="monospace" font-size="14" fill="#111">SO101 action/state trace</text>',
        '<text x="850" y="22" font-family="monospace" font-size="12" fill="#d62728">target</text>',
        '<text x="930" y="22" font-family="monospace" font-size="12" fill="#2ca02c">sent</text>',
        '<text x="990" y="22" font-family="monospace" font-size="12" fill="#1f77b4">state</text>',
    ]
    for index, joint in enumerate(joint_names):
        y0 = top + index * row_h
        values = [row[2][index] for row in positions]
        low, high = min(values), max(values)
        if math.isclose(low, high):
            low -= 1.0
            high += 1.0
        pad = (high - low) * 0.08
        low -= pad
        high += pad
        lines.extend(
            [
                f'<line x1="{left}" y1="{y0 + plot_h}" x2="{width - right}" y2="{y0 + plot_h}" stroke="#ddd"/>',
                f'<line x1="{left}" y1="{y0}" x2="{left}" y2="{y0 + plot_h}" stroke="#ddd"/>',
                f'<text x="14" y="{y0 + 18}" font-family="monospace" font-size="12" fill="#111">{_svg_escape(joint)}</text>',
                f'<text x="14" y="{y0 + 38}" font-family="monospace" font-size="10" fill="#666">{low:.3g}..{high:.3g}</text>',
            ]
        )
        for kind, color in (("action", "#d62728"), ("sent", "#2ca02c"), ("state", "#1f77b4")):
            points = [
                (
                    left + (elapsed_s / max_t) * (width - left - right),
                    y0 + ((high - row[index]) / (high - low)) * plot_h,
                )
                for elapsed_s, sample_kind, row in positions
                if sample_kind == kind
            ]
            points = _downsample(points, 2000)
            if points:
                lines.append(
                    '<polyline points="{}" fill="none" stroke="{}" stroke-width="1.4"/>'.format(
                        " ".join(f"{x:.1f},{y:.1f}" for x, y in points),
                        color,
                    )
                )

    lines.append(f'<text x="{left}" y="{height - 12}" font-family="monospace" font-size="11" fill="#666">0s</text>')
    lines.append(
        f'<text x="{width - 92}" y="{height - 12}" font-family="monospace" font-size="11" fill="#666">{max_t:.2f}s</text>'
    )
    lines.append("</svg>")
    path.write_text("\n".join(lines), encoding="utf-8")


def _timing_summary(timings: list[tuple[float, str, float]]) -> dict[str, dict[str, float]]:
    result: dict[str, dict[str, float]] = {}
    for kind in sorted({kind for _, kind, _ in timings}):
        values = sorted(duration_ms for _, sample_kind, duration_ms in timings if sample_kind == kind)
        if values:
            result[kind] = {
                "count": float(len(values)),
                "avg_ms": round(sum(values) / len(values), 3),
                "p50_ms": round(values[len(values) // 2], 3),
                "p95_ms": round(values[int(0.95 * (len(values) - 1))], 3),
                "max_ms": round(values[-1], 3),
            }
    return result


def _downsample(points: list[tuple[float, float]], limit: int) -> list[tuple[float, float]]:
    if len(points) <= limit:
        return points
    return points[:: math.ceil(len(points) / float(limit))]


def _last_values(positions: list[tuple[float, str, list[float]]], kind: str) -> list[float] | None:
    for _, sample_kind, values in reversed(positions):
        if sample_kind == kind:
            return [round(value, 6) for value in values]
    return None


def _slug(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(value)) or "robot"


def _svg_escape(value: str) -> str:
    return str(value).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
