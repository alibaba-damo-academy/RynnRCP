"""Application-level raw capture encoding facade."""

from __future__ import annotations

from bisect import bisect_right
import json
import os
from typing import Any, Callable, Dict, Iterable, Optional

from rynnrcp.process import run_python_function_task
from rynnrcp_app_common.collection_reader import RawCaptureReader
from rynnrcp_app_common.recording_align import (
    _align_raw_capture_refs,
    _decode_non_image,
)
from rynnrcp_app_common.video_writers import (
    _ProgressPulse,
    _emit_progress,
    _encode_image_videos_parallel,
    _infer_fps,
)


def encode_raw_capture(
    capture_dir: str,
    output_dir: Optional[str] = None,
    keys: Optional[Iterable[str]] = None,
    fps: Optional[float] = None,
    include_original_videos: bool = False,
    key_mapping: Optional[Dict[str, str]] = None,
    video_backend: str = "auto",
    video_encoder: Optional[str] = None,
    _progress_callback: Optional[Callable[[float, float, str], None]] = None,
) -> Dict[str, Any]:
    """Encode a raw capture into aligned timeseries, videos, and metadata."""

    capture_dir = os.path.abspath(os.path.expanduser(capture_dir))
    output_dir = os.path.abspath(os.path.expanduser(output_dir or os.path.join(capture_dir, "encoded")))
    os.makedirs(output_dir, exist_ok=True)

    reader = RawCaptureReader(capture_dir)
    selected_keys = list(keys) if keys is not None else reader.stream_keys()
    key_mapping = dict(key_mapping or {})
    effective_fps = float(fps or reader.capture_meta().get("fps") or 30.0)

    _emit_progress(_progress_callback, 0, 1, f"aligning {os.path.basename(capture_dir)}")
    alignment_stats: Dict[str, Any] = {}
    frames = _align_raw_capture_refs(reader, selected_keys, effective_fps, stats=alignment_stats)
    if not frames:
        raise RuntimeError("no aligned frames available")

    image_keys = [
        key for key in selected_keys
        if key in frames[0]["samples"] and frames[0]["samples"][key].meta.get("type") == "image"
    ]
    scalar_keys = [key for key in selected_keys if key not in image_keys]
    aligned_video_streams = {key: [frame["samples"][key] for frame in frames] for key in image_keys}
    raw_streams = (
        {key: samples for key, samples in reader.read_all_indexes(image_keys).items() if samples}
        if include_original_videos
        else {}
    )

    timeseries_units = len(frames)
    aligned_video_units = sum(len(refs) for refs in aligned_video_streams.values())
    original_video_units = sum(len(refs) for refs in raw_streams.values())
    total_units = max(1, timeseries_units + aligned_video_units + original_video_units)

    timeseries_path = os.path.join(output_dir, "timeseries.parquet")
    _write_timeseries_parquet(
        reader,
        frames,
        scalar_keys,
        timeseries_path,
        effective_fps,
        key_mapping,
        total_units,
        _progress_callback,
    )

    mapped_aligned_video_streams = {
        key_mapping.get(key, key): refs for key, refs in aligned_video_streams.items()
    }
    videos, video_codecs = _encode_image_videos_parallel(
        reader,
        mapped_aligned_video_streams,
        output_dir,
        effective_fps,
        video_backend=video_backend,
        video_encoder=video_encoder,
        filename_prefix="",
        rel_dir="",
        progress_callback=lambda current, _total, message: _emit_progress(
            _progress_callback,
            timeseries_units + current,
            total_units,
            message,
        ),
    )

    original_videos: Dict[str, str] = {}
    original_video_codecs: Dict[str, Dict[str, Any]] = {}
    if include_original_videos:
        original_dir = os.path.join(output_dir, "original_videos")
        os.makedirs(original_dir, exist_ok=True)
        mapped_raw_streams = {key_mapping.get(key, key): samples for key, samples in raw_streams.items()}
        mapped_raw_fps = {
            key_mapping.get(key, key): _infer_fps(samples, effective_fps)
            for key, samples in raw_streams.items()
        }
        original_videos, original_video_codecs = _encode_image_videos_parallel(
            reader,
            mapped_raw_streams,
            original_dir,
            effective_fps,
            video_backend=video_backend,
            video_encoder=video_encoder,
            filename_prefix="original_",
            rel_dir="original_videos",
            fps_by_key=mapped_raw_fps,
            progress_callback=lambda current, _total, message: _emit_progress(
                _progress_callback,
                timeseries_units + aligned_video_units + current,
                total_units,
                message,
            ),
        )

    metadata_path = os.path.join(output_dir, "metadata.json")
    metadata = {
        "source_format": "rynnrcp_raw_capture",
        "capture_dir": capture_dir,
        "fps": effective_fps,
        "timestamp_policy": "relative_frame_index",
        "source_start_timestamp": frames[0]["timestamp"],
        "alignment_policy": alignment_stats.get("policy", "nearest"),
        "timeseries_value_policy": "linear_numeric_nearest_non_numeric",
        "alignment": alignment_stats,
        "total_frames": len(frames),
        "keys": selected_keys,
        "key_mappings": {key: value for key, value in key_mapping.items() if key in selected_keys and value != key},
        "image_keys": image_keys,
        "timeseries": os.path.basename(timeseries_path),
        "videos": videos,
        "video_codecs": video_codecs,
    }
    if original_videos:
        metadata["original_videos"] = original_videos
        metadata["original_video_codecs"] = original_video_codecs
    with open(metadata_path, "w", encoding="utf-8") as f:
        json.dump(metadata, f, ensure_ascii=False, indent=2)

    _emit_progress(_progress_callback, total_units, total_units, f"encoding finished {os.path.basename(capture_dir)}")
    return {
        "output_dir": output_dir,
        "metadata_path": metadata_path,
        "timeseries_path": timeseries_path,
        "videos": videos,
        "original_videos": original_videos,
        "video_codecs": video_codecs,
        "original_video_codecs": original_video_codecs,
        "capture_dir": capture_dir,
        "total_frames": len(frames),
    }


def encode_raw_capture_subprocess(
    capture_dir: str,
    output_dir: Optional[str] = None,
    keys: Optional[Iterable[str]] = None,
    fps: Optional[float] = None,
    include_original_videos: bool = False,
    key_mapping: Optional[Dict[str, str]] = None,
    video_backend: str = "auto",
    video_encoder: Optional[str] = None,
    timeout_s: Optional[float] = None,
) -> Dict[str, Any]:
    """Run ``encode_raw_capture`` in a child Python process."""

    return run_python_function_task(
        "rynnrcp_app_common.recording_worker:encode_from_config",
        kwargs={
            "config": {
                "capture_dir": os.path.abspath(os.path.expanduser(capture_dir)),
                "output_dir": os.path.abspath(os.path.expanduser(output_dir)) if output_dir is not None else None,
                "keys": list(keys) if keys is not None else None,
                "fps": fps,
                "include_original_videos": include_original_videos,
                "key_mapping": dict(key_mapping or {}),
                "video_backend": video_backend,
                "video_encoder": video_encoder,
            }
        },
        timeout_s=timeout_s,
    )


def _write_timeseries_parquet(
    reader: RawCaptureReader,
    frames: list[Dict[str, Any]],
    scalar_keys: list[str],
    timeseries_path: str,
    effective_fps: float,
    key_mapping: Dict[str, str],
    total_units: int,
    progress_callback: Optional[Callable[[float, float, str], None]],
) -> None:
    try:
        import pandas as pd
        import pyarrow as pa
        import pyarrow.parquet as pq
    except ImportError as exc:
        raise RuntimeError("pandas and pyarrow are required to export timeseries.parquet") from exc

    rows: list[Dict[str, Any]] = []
    data_files: Dict[str, Any] = {}
    scalar_indexes = reader.read_all_indexes(scalar_keys)
    scalar_timelines = {key: [float(ref.timestamp) for ref in refs] for key, refs in scalar_indexes.items()}
    try:
        progress_pulse = _ProgressPulse(len(frames))
        for index, frame in enumerate(frames, start=1):
            row = _timeseries_row(
                reader,
                frame,
                scalar_keys,
                data_files,
                effective_fps,
                key_mapping,
                scalar_indexes,
                scalar_timelines,
            )
            row.pop("source_timestamp", None)
            rows.append(row)
            if progress_pulse.should_emit(index):
                _emit_progress(
                    progress_callback,
                    index,
                    total_units,
                    f"writing timeseries {index}/{len(frames)}",
                )
    finally:
        for data_file in data_files.values():
            close = getattr(data_file, "close", None)
            if callable(close):
                close()

    seen_keys = {key for row in rows for key in row}
    ordered_keys = sorted(key for key in seen_keys if key != "reused_sample_by_key")
    rows = [{key: row.get(key) for key in ordered_keys if key in row} for row in rows]

    table = pa.Table.from_pandas(pd.DataFrame(rows, columns=ordered_keys), preserve_index=False)
    pq.write_table(table, timeseries_path)


def _timeseries_row(
    reader: RawCaptureReader,
    frame: Dict[str, Any],
    scalar_keys: list[str],
    data_files: Dict[str, Any],
    effective_fps: float,
    key_mapping: Dict[str, str],
    scalar_indexes: Dict[str, list[Any]],
    scalar_timelines: Dict[str, list[float]],
) -> Dict[str, Any]:
    row = {
        "index": frame["frame_index"],
        "frame_index": frame["frame_index"],
        "episode_index": 0,
        "task_index": 0,
        "timestamp": frame["frame_index"] / effective_fps,
        "source_timestamp": frame["timestamp"],
        "reused_sample_by_key": frame.get("reused_sample_by_key", {}),
    }
    for key in scalar_keys:
        ref = frame["samples"].get(key)
        if ref is None:
            continue
        data_file = data_files.get(key)
        if data_file is None:
            data_file = reader.open_stream_data(key)
            data_files[key] = data_file
        value = _interpolated_timeseries_value(
            reader,
            key,
            frame["timestamp"],
            scalar_indexes.get(key) or [],
            scalar_timelines.get(key) or [],
            ref,
            data_file,
        )
        row[key] = value
        mapped_key = key_mapping.get(key)
        if mapped_key:
            row[mapped_key] = value
    return row


def _interpolated_timeseries_value(
    reader: RawCaptureReader,
    key: str,
    timestamp: float,
    refs: list[Any],
    times: list[float],
    nearest_ref: Any,
    data_file: Any,
) -> Any:
    nearest_value = _export_timeseries_value(key, _decode_non_image(reader.read_ref(nearest_ref, data_file=data_file)))
    if len(refs) < 2 or len(times) != len(refs):
        return nearest_value
    pos = bisect_right(times, timestamp)
    if pos <= 0 or pos >= len(refs):
        return nearest_value
    before_t = times[pos - 1]
    after_t = times[pos]
    if after_t <= before_t:
        return nearest_value
    before_value = _export_timeseries_value(key, _decode_non_image(reader.read_ref(refs[pos - 1], data_file=data_file)))
    after_value = _export_timeseries_value(key, _decode_non_image(reader.read_ref(refs[pos], data_file=data_file)))
    ratio = (float(timestamp) - before_t) / (after_t - before_t)
    return _lerp_value(before_value, after_value, ratio, fallback=nearest_value)


def _export_timeseries_value(key: str, value: Any) -> Any:
    if isinstance(value, dict) and "joint_positions" in value and (
        key.startswith("action.") or key.startswith("observation.")
    ):
        return [float(item) for item in value["joint_positions"]]
    return value


def _lerp_value(before: Any, after: Any, ratio: float, *, fallback: Any) -> Any:
    if _is_number(before) and _is_number(after):
        return float(before) + (float(after) - float(before)) * float(ratio)
    if (
        isinstance(before, list)
        and isinstance(after, list)
        and len(before) == len(after)
        and all(_is_number(x) for x in before)
        and all(_is_number(x) for x in after)
    ):
        r = float(ratio)
        return [float(a) + (float(b) - float(a)) * r for a, b in zip(before, after)]
    return fallback


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)
