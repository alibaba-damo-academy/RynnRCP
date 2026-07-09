"""Protocol collection scanning and deletion for Teleop."""

from __future__ import annotations

import json
import os
import shutil
import time
from bisect import bisect_right
from typing import Any, Dict, List, Optional

import msgpack
import numpy as np

from rynnrcp.utils import safe_name
from rynnrcp_app_common.recording_package import package_encoded_captures
from rynnrcp_app_common.video_writers import encode_bgr_frames_to_h264

from .utils import directory_size, format_bytes


class TeleopRecordingManager:
    def __init__(self, record_dir: str, config: Dict[str, Any]) -> None:
        self.record_dir = os.path.abspath(record_dir)
        self._config = config
        self._configured_record_roots = {
            os.path.abspath(os.path.expanduser(str(path)))
            for path in config.get("record_roots", [])
            if path
        }
        self._extra_record_roots: set[str] = set()

    def add_collection_dir(self, collection_dir: str) -> None:
        path = os.path.abspath(os.path.expanduser(str(collection_dir)))
        if os.path.isfile(os.path.join(path, "collection_meta.json")):
            self.add_record_root(path)

    def add_record_root(self, root_dir: str) -> None:
        self._extra_record_roots.add(os.path.abspath(os.path.expanduser(str(root_dir))))

    def scan_local_records(self) -> Dict[str, Any]:
        collections: Dict[str, Dict[str, Any]] = {}
        exported_zips = []
        total_size = 0

        for root in self._record_roots():
            if not os.path.isdir(root):
                continue
            for dirpath, dirnames, filenames in os.walk(root):
                if "collection_meta.json" not in filenames:
                    continue
                try:
                    episode = self.episode_info_from_collection(dirpath)
                except ValueError:
                    dirnames[:] = []
                    continue
                total_size += episode["size"]
                collection = collections.setdefault(
                    episode["collection_id"],
                    {"collection_id": episode["collection_id"], "task_prompts": {}, "size": 0},
                )
                collection["size"] += episode["size"]
                prompt = collection["task_prompts"].setdefault(
                    episode["task_prompt"],
                    {"task_prompt": episode["task_prompt"], "episodes": []},
                )
                prompt["episodes"].append(episode)
                dirnames[:] = []

        if os.path.isdir(self.export_dir):
            for dirpath, _dirnames, filenames in os.walk(self.export_dir):
                for name in filenames:
                    if not name.endswith(".zip"):
                        continue
                    path = os.path.join(dirpath, name)
                    size = os.path.getsize(path)
                    exported_at_unix = os.path.getmtime(path)
                    total_size += size
                    exported_zips.append({
                        "name": name,
                        "path": path,
                        "size": size,
                        "size_formatted": format_bytes(size),
                        "exported_at_unix": exported_at_unix,
                        "exported_at": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(exported_at_unix)),
                    })

        data_collections = []
        for collection in collections.values():
            prompts = list(collection["task_prompts"].values())
            for prompt in prompts:
                prompt["episodes"].sort(key=lambda ep: ep.get("created_at_unix", 0.0), reverse=True)
                prompt["latest_created_at_unix"] = max(
                    (float(ep.get("created_at_unix") or 0.0) for ep in prompt["episodes"]),
                    default=0.0,
                )
            prompts.sort(key=lambda item: item.get("latest_created_at_unix", 0.0), reverse=True)
            data_collections.append({
                "collection_id": collection["collection_id"],
                "task_prompts": prompts,
                "size": collection["size"],
                "size_formatted": format_bytes(collection["size"]),
                "latest_created_at_unix": max(
                    (float(item.get("latest_created_at_unix") or 0.0) for item in prompts),
                    default=0.0,
                ),
            })

        data_collections.sort(key=lambda item: item.get("latest_created_at_unix", 0.0), reverse=True)
        exported_zips.sort(key=lambda item: item.get("exported_at_unix", 0), reverse=True)
        return {
            "data_collections": data_collections,
            "exported_zips": exported_zips,
            "export_dir": self.export_dir,
            "total_size": total_size,
            "total_size_formatted": format_bytes(total_size),
        }

    def delete_episode(self, path: str) -> Dict[str, Any]:
        try:
            episode_dir = self.safe_record_path(path)
            if not os.path.isdir(episode_dir):
                return {"success": False, "message": "episode path not found", "path": path}
            shutil.rmtree(episode_dir)
            return {"success": True, "message": "episode deleted", "path": episode_dir}
        except Exception as exc:
            return {"success": False, "message": str(exc), "path": path}

    def delete_exported_zip(self, path: str) -> Dict[str, Any]:
        try:
            zip_path = self.safe_export_path(path)
            if not zip_path.endswith(".zip"):
                return {"success": False, "message": "export path must be a zip file", "path": path}
            if not os.path.isfile(zip_path):
                return {"success": False, "message": "export zip not found", "path": path}
            os.remove(zip_path)
            return {"success": True, "message": "export zip deleted", "path": zip_path}
        except Exception as exc:
            return {"success": False, "message": str(exc), "path": path}

    def delete_data_collection(self, collection_id: str) -> Dict[str, Any]:
        records = self.scan_local_records()
        deleted = []
        for collection in records["data_collections"]:
            if collection["collection_id"] != collection_id:
                continue
            for prompt in collection["task_prompts"]:
                for episode in prompt["episodes"]:
                    result = self.delete_episode(episode["path"])
                    if result.get("success"):
                        deleted.append(result["path"])
        if not deleted:
            return {"success": False, "message": "data collection not found", "collection_id": collection_id}
        return {"success": True, "message": "data collection deleted", "collection_id": collection_id, "deleted": deleted}

    def encode_episodes(self, episode_paths: List[str], **kwargs: Any) -> Dict[str, Any]:
        progress_callback = kwargs.get("_progress_callback")
        keys = kwargs.get("keys")
        fps = kwargs.get("fps")
        video_encoder = kwargs.get("video_encoder") or self._config.get("video_encoder")
        mapping_enabled = bool(kwargs.get("export_rynnbot_mapping", self._config.get("export_rynnbot_mapping", True)))
        key_mapping = kwargs.get("key_mapping")
        if key_mapping is None:
            key_mapping = self._config.get("export_key_mapping") or {}
        encoded = []
        failed = []
        total = len(episode_paths)
        for index, path in enumerate(episode_paths, start=1):
            try:
                if progress_callback:
                    progress_callback(index - 1, total, f"encoding {os.path.basename(path)}")
                episode_dir = self.safe_record_path(path)
                encoded.append(_encode_collection(
                    episode_dir,
                    keys=keys,
                    fps=fps,
                    video_encoder=video_encoder,
                    key_mapping=key_mapping,
                    export_rynnbot_mapping=mapping_enabled,
                ))
            except Exception as exc:
                failed.append({"episode_path": path, "error": str(exc)})
        if progress_callback:
            progress_callback(total, total, "encoding finished")
        return {
            "success": bool(encoded),
            "message": f"encoded={len(encoded)} failed={len(failed)}",
            "result": {"encoded": encoded, "failed": failed},
            "encoded": encoded,
            "failed": failed,
        }

    def export_episodes(self, episode_paths: List[str], zip_name: Optional[str] = None, **kwargs: Any) -> Dict[str, Any]:
        progress_callback = kwargs.get("_progress_callback")
        try:
            episode_dirs = [self.safe_record_path(path) for path in episode_paths]
            if not episode_dirs:
                return {"success": False, "message": "episode_paths required"}
            encode_result = self.encode_episodes(episode_dirs, **kwargs)
            encoded = list(encode_result.get("encoded") or [])
            failed = list(encode_result.get("failed") or [])
            if failed:
                return {
                    "success": False,
                    "message": _format_failed_export_message(failed),
                    "result": {"encoded": encoded, "failed": failed},
                    "encoded": encoded,
                    "failed": failed,
                }
            if not encoded:
                return {"success": False, "message": "no episodes encoded", "result": {"encoded": [], "failed": []}}
            if progress_callback:
                progress_callback(len(episode_dirs), len(episode_dirs), "packaging zip")
            package = package_encoded_captures(
                encoded,
                package_dir=self.export_dir,
                zip_name=zip_name or self._default_zip_name_for_episode_paths(episode_dirs),
                include_original_videos=False,
            )
            return {
                "success": True,
                "message": f"exported={len(encoded)} failed=0",
                "result": {"encoded": encoded, "failed": [], "exported": package["episodes"], "package": package, "zip_path": package["zip_path"], "zip_name": package["zip_name"]},
                "encoded": encoded,
                "failed": [],
                "package": package,
                "zip_path": package["zip_path"],
                "zip_name": package["zip_name"],
                "episode_paths": episode_dirs,
            }
        except Exception as exc:
            return {"success": False, "message": str(exc), "episode_paths": episode_paths}

    def export_data_collection(self, collection_id: str, **kwargs: Any) -> Dict[str, Any]:
        records = self.scan_local_records()
        episode_paths: List[str] = []
        for collection in records["data_collections"]:
            if collection["collection_id"] != collection_id:
                continue
            for prompt in collection["task_prompts"]:
                for episode in prompt["episodes"]:
                    episode_paths.append(episode["path"])
            break
        if not episode_paths:
            return {"success": False, "message": "data collection not found", "collection_id": collection_id}
        result = self.export_episodes(episode_paths, zip_name=kwargs.pop("zip_name", None), **kwargs)
        result["collection_id"] = collection_id
        result["episode_paths"] = episode_paths
        return result

    def safe_record_path(self, path: str) -> str:
        candidate = os.path.abspath(os.path.expanduser(path))
        roots = self._record_roots()
        if not any(os.path.commonpath([root, candidate]) == root for root in roots):
            raise ValueError(f"path is outside record_dir: {path}")
        return candidate

    @property
    def export_dir(self) -> str:
        return os.path.abspath(str(self._config.get("export_dir") or os.path.join(self.record_dir, "exports")))

    def safe_export_path(self, path: str) -> str:
        root = self.export_dir
        candidate = os.path.abspath(os.path.expanduser(path))
        if os.path.commonpath([root, candidate]) != root:
            raise ValueError(f"path is outside export_dir: {path}")
        return candidate

    def _record_roots(self) -> List[str]:
        roots = {self.record_dir, *self._configured_record_roots, *self._extra_record_roots}
        normalized = sorted(os.path.abspath(os.path.expanduser(root)) for root in roots)
        collapsed: List[str] = []
        for root in normalized:
            if any(_is_relative_to(root, parent) for parent in collapsed):
                continue
            collapsed = [parent for parent in collapsed if not _is_relative_to(parent, root)]
            collapsed.append(root)
        return collapsed

    def episode_info_from_collection(self, collection_dir: str) -> Dict[str, Any]:
        collection_dir = os.path.abspath(collection_dir)
        meta_path = os.path.join(collection_dir, "collection_meta.json")
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = json.load(f)

        counts = _stream_counts(os.path.join(collection_dir, "streams"))
        metadata = meta.get("metadata") if isinstance(meta.get("metadata"), dict) else {}
        collection_id = _required_metadata_value(meta, metadata, "collection_id", meta_path)
        episode_id = _required_metadata_value(meta, metadata, "episode_id", meta_path)
        created_at = float(meta.get("started_at_unix") or os.path.getmtime(meta_path) or time.time())
        size = directory_size(collection_dir)
        return {
            "episode_name": episode_id,
            "folder_name": os.path.basename(collection_dir),
            "path": collection_dir,
            "status": "complete" if counts else "invalid",
            "frames": max(counts.values(), default=0),
            "size": size,
            "size_formatted": format_bytes(size),
            "collection_id": collection_id,
            "episode_id": episode_id,
            "task_prompt": str(metadata.get("task_prompt") or "default"),
            "created_at_unix": created_at,
            "streams": sorted(counts),
            "counts": counts,
        }

    def _default_zip_name_for_episode_paths(self, episode_paths: List[str]) -> str:
        paths = {os.path.abspath(os.path.expanduser(path)) for path in episode_paths}
        matches: Dict[str, List[Dict[str, Any]]] = {}
        for collection in self.scan_local_records().get("data_collections", []):
            for prompt in collection.get("task_prompts", []):
                for episode in prompt.get("episodes", []):
                    if os.path.abspath(os.path.expanduser(str(episode.get("path")))) in paths:
                        matches.setdefault(str(collection.get("collection_id") or "teleop_export"), []).append(episode)
        if len(matches) == 1:
            collection_id, episodes = next(iter(matches.items()))
            episode_ids = sorted(str(ep.get("episode_id") or ep.get("episode_name") or "") for ep in episodes)
            suffix = safe_name(episode_ids[0]) if len(episode_ids) == 1 and episode_ids[0] else f"episodes_{len(episodes)}"
            return f"{safe_name(collection_id)}_{suffix}.zip"
        return f"teleop_export_selected_{int(time.time())}.zip"


def _stream_counts(streams_dir: str) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    if not os.path.isdir(streams_dir):
        return counts
    for name in sorted(os.listdir(streams_dir)):
        path = os.path.join(streams_dir, name, "samples.msgpack")
        if not os.path.isfile(path):
            continue
        count = 0
        with open(path, "rb") as f:
            for _sample in msgpack.Unpacker(f, raw=False):
                count += 1
        counts[name] = count
    return counts


def _encode_collection(
    collection_dir: str,
    *,
    keys: Any = None,
    fps: Any = None,
    video_encoder: Any = None,
    key_mapping: Any = None,
    export_rynnbot_mapping: bool = True,
) -> Dict[str, Any]:
    collection_dir = os.path.abspath(collection_dir)
    meta = _read_json(os.path.join(collection_dir, "collection_meta.json"))
    streams = _read_collection_streams(collection_dir)
    selected = [str(key) for key in (keys or meta.get("names") or streams.keys()) if str(key) in streams]
    if not selected:
        raise RuntimeError("no collection streams selected")
    image_keys = [key for key in selected if _is_image_samples(streams[key])]
    scalar_keys = [key for key in selected if key not in image_keys]

    output_dir = os.path.join(collection_dir, "encoded")
    if os.path.isdir(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir, exist_ok=True)
    effective_fps = float(fps or meta.get("fps") or 30.0)
    timeline = _alignment_timeline(streams, selected, effective_fps)
    if not timeline:
        raise RuntimeError("no aligned frames available")
    timelines = {key: [float(sample["timestamp"]) for sample in streams[key]] for key in selected}
    last_index_by_key: Dict[str, Optional[int]] = {key: None for key in selected}
    reused_count_by_key: Dict[str, int] = {key: 0 for key in selected}
    mappings = _key_mappings(selected, key_mapping, enabled=export_rynnbot_mapping)
    frames = []
    for index, source_timestamp in enumerate(timeline):
        row = {
            "index": index,
            "frame_index": index,
            "episode_index": 0,
            "task_index": 0,
            "timestamp": index / effective_fps,
            "source_timestamp": source_timestamp,
        }
        for key in selected:
            nearest_index = _nearest_sample_index(timelines[key], source_timestamp)
            if nearest_index is None:
                continue
            if last_index_by_key[key] == nearest_index:
                reused_count_by_key[key] += 1
            last_index_by_key[key] = nearest_index
            if key not in scalar_keys:
                continue
            value = _interpolated_export_value(key, streams[key], timelines[key], source_timestamp)
            row[key] = value
            mapped_key = mappings.get(key)
            if mapped_key:
                row[mapped_key] = value
        frames.append(row)

    timeseries_path = os.path.join(output_dir, "timeseries.parquet")
    _write_timeseries_parquet(frames, timeseries_path)

    videos = {}
    video_codecs = {}
    for key in image_keys:
        video_key = mappings.get(key) or key
        video_path = os.path.join(output_dir, f"{safe_name(video_key)}.mp4")
        codec = _write_image_video(
            [_nearest_sample(streams[key], row["source_timestamp"]) for row in frames],
            video_path,
            effective_fps,
            video_encoder=video_encoder,
        )
        videos[video_key] = os.path.basename(video_path)
        video_codecs[video_key] = {**codec, "source": key}

    duration_s = timeline[-1] - timeline[0] if len(timeline) > 1 else 0.0
    alignment = {
        "policy": "nearest",
        "timeseries_value_policy": "linear_numeric_nearest_non_numeric",
        "fps": effective_fps,
        "interval_s": 1.0 / effective_fps,
        "total_frames": len(frames),
        "reused_sample_count_by_key": reused_count_by_key,
        "reused_sample_rate_by_key": {
            key: (reused_count_by_key[key] / len(frames)) if frames else 0.0
            for key in selected
        },
        "start_timestamp": timeline[0],
        "end_timestamp": timeline[-1],
        "recording_duration_s": duration_s,
        "frames_recorded_by_key": {key: len(streams[key]) for key in selected},
        "recording_duration_by_key": {
            key: float(streams[key][-1]["timestamp"] - streams[key][0]["timestamp"])
            for key in selected
        },
    }
    metadata = {
        "source_format": "rynnrcp_collection",
        "capture_dir": collection_dir,
        "fps": effective_fps,
        "timestamp_policy": "relative_frame_index",
        "source_start_timestamp": timeline[0],
        "alignment_policy": alignment["policy"],
        "timeseries_value_policy": alignment["timeseries_value_policy"],
        "alignment": alignment,
        "total_frames": len(frames),
        "keys": selected,
        "image_keys": image_keys,
        "key_mappings": mappings,
        "timeseries": os.path.basename(timeseries_path),
        "videos": videos,
        "video_codecs": video_codecs,
    }
    metadata_path = os.path.join(output_dir, "metadata.json")
    with open(metadata_path, "w", encoding="utf-8") as f:
        json.dump(metadata, f, ensure_ascii=False, indent=2)
    return {
        "output_dir": output_dir,
        "metadata_path": metadata_path,
        "timeseries_path": timeseries_path,
        "videos": videos,
        "video_codecs": video_codecs,
        "capture_dir": collection_dir,
        "total_frames": len(frames),
    }


def _read_collection_streams(collection_dir: str) -> Dict[str, List[Dict[str, Any]]]:
    streams_dir = os.path.join(collection_dir, "streams")
    streams: Dict[str, List[Dict[str, Any]]] = {}
    if not os.path.isdir(streams_dir):
        return streams
    for name in sorted(os.listdir(streams_dir)):
        path = os.path.join(streams_dir, name, "samples.msgpack")
        if not os.path.isfile(path):
            continue
        samples = []
        with open(path, "rb") as f:
            for sample in msgpack.Unpacker(f, raw=False):
                if isinstance(sample, dict) and "timestamp" in sample and "value" in sample:
                    samples.append({"timestamp": float(sample["timestamp"]), "value": sample["value"]})
        streams[name] = samples
    return streams


def _is_image_samples(samples: List[Dict[str, Any]]) -> bool:
    return bool(samples and isinstance(samples[0].get("value"), dict) and "image" in samples[0]["value"])


def _alignment_timeline(streams: Dict[str, List[Dict[str, Any]]], selected: List[str], fps: float) -> List[float]:
    start = max(float(streams[key][0]["timestamp"]) for key in selected)
    end = min(float(streams[key][-1]["timestamp"]) for key in selected)
    if end < start:
        return []
    interval = 1.0 / float(fps)
    timeline: List[float] = []
    t = start
    while t <= end + 1e-9:
        timeline.append(t)
        t += interval
    return timeline


def _key_mappings(names: List[str], mapping: Any, *, enabled: bool = True) -> Dict[str, str]:
    if not enabled:
        return {}
    result = {
        name: mapped
        for name in names
        if (mapped := _default_rynnbot_key(name))
    }
    if not isinstance(mapping, dict):
        return result
    selected = set(names)
    result.update({
        str(source): str(target)
        for source, target in mapping.items()
        if str(source) in selected and str(target).strip()
    })
    return result


def _default_rynnbot_key(name: str) -> Optional[str]:
    if name == "action.robot.joint_position":
        return "action"
    if name == "observation.robot.joint_state":
        return "observation.state"
    parts = name.split(".")
    if len(parts) == 3 and parts[0] == "observation" and parts[2] == "image":
        return f"observation.images.{parts[1]}"
    return None


def _export_value(name: str, value: Any) -> Any:
    if isinstance(value, dict):
        if "joint_positions" in value and (name.startswith("action.") or name.startswith("observation.")):
            return [float(item) for item in value["joint_positions"]]
    return _json_safe(value)


def _write_timeseries_parquet(rows: List[Dict[str, Any]], path: str) -> None:
    try:
        import pandas as pd
        import pyarrow as pa
        import pyarrow.parquet as pq
    except ImportError as exc:
        raise RuntimeError("pandas and pyarrow are required to export timeseries.parquet") from exc

    output_rows = []
    for row in rows:
        item = dict(row)
        item.pop("source_timestamp", None)
        output_rows.append(item)

    seen_keys = {key for row in output_rows for key in row}
    ordered_keys = sorted(seen_keys)
    output_rows = [{key: row.get(key) for key in ordered_keys if key in row} for row in output_rows]

    table = pa.Table.from_pandas(pd.DataFrame(output_rows, columns=ordered_keys), preserve_index=False)
    pq.write_table(table, path)


def _nearest_sample(samples: List[Dict[str, Any]], timestamp: float) -> Optional[Dict[str, Any]]:
    if not samples:
        return None
    index = _nearest_sample_index([float(item["timestamp"]) for item in samples], timestamp)
    return None if index is None else samples[index]


def _interpolated_export_value(
    key: str,
    samples: List[Dict[str, Any]],
    timestamps: List[float],
    timestamp: float,
) -> Any:
    nearest_index = _nearest_sample_index(timestamps, timestamp)
    if nearest_index is None:
        return None
    nearest_value = _export_value(key, samples[nearest_index]["value"])
    pos = bisect_right(timestamps, timestamp)
    if pos <= 0 or pos >= len(samples):
        return nearest_value
    before_t = timestamps[pos - 1]
    after_t = timestamps[pos]
    if after_t <= before_t:
        return nearest_value
    before_value = _export_value(key, samples[pos - 1]["value"])
    after_value = _export_value(key, samples[pos]["value"])
    ratio = (timestamp - before_t) / (after_t - before_t)
    return _lerp_value(before_value, after_value, ratio, fallback=nearest_value)


def _nearest_sample_index(values: List[float], timestamp: float) -> Optional[int]:
    if not values:
        return None
    pos = bisect_right(values, timestamp)
    if pos == 0:
        return 0
    if pos >= len(values):
        return len(values) - 1
    before = values[pos - 1]
    after = values[pos]
    return pos - 1 if (timestamp - before) <= (after - timestamp) else pos


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


def _write_image_video(
    samples: List[Optional[Dict[str, Any]]],
    output_path: str,
    fps: float,
    *,
    video_encoder: Any = None,
) -> Dict[str, Any]:
    frames = [_image_to_bgr(sample["value"]) for sample in samples if sample is not None]
    return encode_bgr_frames_to_h264(
        frames,
        output_path,
        fps,
        video_encoder=str(video_encoder) if video_encoder else None,
    )


def _image_to_bgr(value: Dict[str, Any]) -> np.ndarray:
    try:
        import cv2
    except Exception as exc:
        raise RuntimeError("opencv-python is required for image decoding") from exc
    encoding = str(value.get("encoding") or "").lower()
    image = value.get("image")
    if encoding in ("jpg", "jpeg", "png"):
        decoded = cv2.imdecode(np.frombuffer(image, dtype=np.uint8), cv2.IMREAD_COLOR)
        if decoded is None:
            raise ValueError("failed to decode compressed image")
        return decoded
    width = int(value["width"])
    height = int(value["height"])
    channels = 1 if encoding in ("mono8", "gray8") else 3
    raw = np.frombuffer(image, dtype=np.uint8).reshape((height, width, channels))
    if channels == 1:
        return cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
    if encoding == "rgb8":
        return cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
    if encoding == "bgr8":
        return raw
    raise ValueError(f"unsupported image encoding: {encoding}")


def _json_safe(value: Any) -> Any:
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, bytes):
        return {"bytes": len(value)}
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return str(value)


def _read_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return data if isinstance(data, dict) else {}


def _format_failed_export_message(failed: List[Dict[str, Any]]) -> str:
    labels = []
    for item in failed[:3]:
        path = str(item.get("episode_path") or "")
        labels.append(f"{os.path.basename(path) or 'episode'}: {item.get('error') or 'unknown error'}")
    suffix = "" if len(failed) <= 3 else f"; ... {len(failed) - 3} more"
    return f"export failed: {len(failed)} episode(s) failed; " + "; ".join(labels) + suffix


def _is_relative_to(path: str, parent: str) -> bool:
    try:
        return os.path.commonpath([path, parent]) == parent
    except ValueError:
        return False


def _required_metadata_value(
    meta: Dict[str, Any],
    metadata: Dict[str, Any],
    key: str,
    meta_path: str,
) -> str:
    value = meta.get(key)
    if value is None:
        value = metadata.get(key)
    text = str(value or "").strip()
    if not text:
        raise ValueError(f"{key} is required in collection metadata: {meta_path}")
    return text
