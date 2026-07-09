"""Raw capture stream alignment helpers."""

from __future__ import annotations

from bisect import bisect_right
from typing import Any, Dict, Iterable, List, Optional

import msgpack

from rynnrcp_app_common.collection_reader import RawCaptureReader, RawCaptureSample, RawCaptureSampleRef


def _align_raw_capture_refs(
    reader: RawCaptureReader,
    keys: Optional[Iterable[str]] = None,
    fps: Optional[float] = None,
    stats: Optional[Dict[str, Any]] = None,
    policy: str = "nearest",
) -> List[Dict[str, Any]]:
    streams = {key: samples for key, samples in reader.read_all_indexes(keys).items() if samples}
    if not streams:
        return []

    effective_fps = float(fps or reader.capture_meta().get("fps") or 30.0)
    interval = 1.0 / effective_fps
    start = max(samples[0].timestamp for samples in streams.values())
    end = min(samples[-1].timestamp for samples in streams.values())
    if end < start:
        return []

    selector = _latest_index_at_or_before if policy == "hold_last" else _nearest_index
    timelines = {key: [sample.timestamp for sample in samples] for key, samples in streams.items()}
    last_idx: Dict[str, Optional[int]] = {key: None for key in streams}
    reused_count: Dict[str, int] = {key: 0 for key in streams}
    frames: List[Dict[str, Any]] = []

    t = start
    while t <= end + 1e-9:
        samples_by_key: Dict[str, RawCaptureSampleRef] = {}
        reused_by_key: Dict[str, bool] = {}
        for key, samples in streams.items():
            idx = selector(timelines[key], t)
            if idx is None:
                continue
            reused = idx == last_idx[key]
            reused_by_key[key] = reused
            if reused:
                reused_count[key] += 1
            last_idx[key] = idx
            samples_by_key[key] = samples[idx]
        if samples_by_key:
            frames.append({
                "frame_index": len(frames),
                "timestamp": t,
                "samples": samples_by_key,
                "reused_sample_by_key": reused_by_key,
            })
        t += interval

    if stats is not None:
        total = len(frames)
        stats.update({
            "policy": policy,
            "fps": effective_fps,
            "interval_s": interval,
            "total_frames": total,
            "reused_sample_count_by_key": reused_count,
            "reused_sample_rate_by_key": {
                key: (reused_count[key] / total) if total else 0.0
                for key in streams
            },
            "start_timestamp": start,
            "end_timestamp": end,
            "recording_duration_s": float(end - start),
            "frames_recorded_by_key": {key: len(samples) for key, samples in streams.items()},
            "recording_duration_by_key": {
                key: float(samples[-1].timestamp - samples[0].timestamp)
                for key, samples in streams.items()
            },
        })
    return frames


def _latest_index_at_or_before(values: List[float], target: float) -> Optional[int]:
    pos = bisect_right(values, target)
    return None if pos <= 0 else pos - 1


def _nearest_index(values: List[float], target: float) -> Optional[int]:
    if not values:
        return None
    pos = bisect_right(values, target)
    if pos == 0:
        return 0
    if pos >= len(values):
        return len(values) - 1
    before = values[pos - 1]
    after = values[pos]
    return pos - 1 if (target - before) <= (after - target) else pos


def _decode_non_image(sample: RawCaptureSample) -> Any:
    if sample.meta.get("encoding") == "msgpack":
        return msgpack.unpackb(sample.data, raw=False)
    return sample.data.decode("utf-8", errors="replace")
