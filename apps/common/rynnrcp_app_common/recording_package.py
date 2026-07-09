"""Package encoded raw captures into export zip files."""

from __future__ import annotations

import json
import os
import time
import zipfile
from typing import Any, Dict, List, Optional

from rynnrcp.utils import safe_name


def package_encoded_captures(
    encoded_results: List[Dict[str, Any]],
    package_dir: str,
    zip_name: Optional[str] = None,
    include_original_videos: bool = False,
) -> Dict[str, Any]:
    """Package encoded captures into a zip without raw streams."""

    os.makedirs(package_dir, exist_ok=True)
    safe_zip_name = safe_name(zip_name or f"rynnrcp_export_{int(time.time())}.zip")
    if not safe_zip_name.endswith(".zip"):
        safe_zip_name += ".zip"
    package_path = _unique_zip_path(package_dir, safe_zip_name)
    package_root = safe_name(os.path.splitext(os.path.basename(package_path))[0] or "rynnrcp_export")
    episodes = []

    with zipfile.ZipFile(package_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for encoded in encoded_results:
            output_dir = os.path.abspath(str(encoded["output_dir"]))
            if not os.path.isdir(output_dir):
                raise FileNotFoundError(f"encoded output_dir not found: {output_dir}")

            capture_dir = os.path.abspath(str(encoded.get("capture_dir") or os.path.dirname(output_dir)))
            episode_name = safe_name(os.path.basename(capture_dir) or os.path.basename(output_dir))
            episode_prefix = os.path.join(package_root, episode_name)
            encoded_meta = _read_json_file(os.path.join(output_dir, "metadata.json"))
            dataset_meta = _dataset_metadata_for_package(encoded, encoded_meta)

            zf.writestr(
                os.path.join(episode_prefix, "metadata.json"),
                json.dumps(dataset_meta, ensure_ascii=False, indent=2),
            )
            timeseries_name = str(encoded_meta.get("timeseries") or "timeseries.parquet")
            if os.path.basename(timeseries_name) != "timeseries.parquet":
                raise ValueError(f"encoded timeseries must be timeseries.parquet: {timeseries_name}")
            timeseries_path = os.path.join(output_dir, timeseries_name)
            if not os.path.isfile(timeseries_path):
                raise FileNotFoundError(f"encoded timeseries not found: {timeseries_path}")
            zf.write(timeseries_path, os.path.join(episode_prefix, "timeseries.parquet"))
            file_count = 2

            videos = encoded_meta.get("videos") if isinstance(encoded_meta.get("videos"), dict) else encoded.get("videos", {})
            for rel in videos.values():
                path = os.path.join(output_dir, str(rel))
                if not os.path.isfile(path):
                    raise FileNotFoundError(f"encoded video not found: {path}")
                zf.write(path, os.path.join(episode_prefix, os.path.basename(path)))
                file_count += 1

            if include_original_videos:
                original_videos = (
                    encoded_meta.get("original_videos")
                    if isinstance(encoded_meta.get("original_videos"), dict)
                    else encoded.get("original_videos", {})
                )
                for rel in original_videos.values():
                    path = os.path.join(output_dir, str(rel))
                    if os.path.isfile(path):
                        zf.write(path, os.path.join(episode_prefix, "original_videos", os.path.basename(path)))
                        file_count += 1

            episodes.append({
                "episode_name": episode_name,
                "capture_dir": capture_dir,
                "encoded_dir": output_dir,
                "total_frames": encoded.get("total_frames"),
                "videos": encoded.get("videos", {}),
                "original_videos_included": bool(include_original_videos),
                "files": file_count,
            })

    return {
        "package_path": package_path,
        "zip_path": package_path,
        "zip_name": os.path.basename(package_path),
        "package_root": package_root,
        "include_raw": False,
        "include_original_videos": bool(include_original_videos),
        "episodes": episodes,
        "size_bytes": os.path.getsize(package_path),
    }


def _unique_zip_path(package_dir: str, zip_name: str) -> str:
    base_name = safe_name(zip_name)
    stem, ext = os.path.splitext(base_name)
    if ext.lower() != ".zip":
        stem = base_name
        ext = ".zip"
    stem = stem or "rynnrcp_export"
    candidate = os.path.abspath(os.path.join(package_dir, f"{stem}{ext}"))
    suffix = 2
    while os.path.exists(candidate):
        candidate = os.path.abspath(os.path.join(package_dir, f"{stem}_{suffix}{ext}"))
        suffix += 1
    return candidate


def _read_json_file(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return data if isinstance(data, dict) else {}


def _dataset_metadata_for_package(encoded: Dict[str, Any], encoded_meta: Dict[str, Any]) -> Dict[str, Any]:
    capture_dir = os.path.abspath(str(encoded.get("capture_dir") or encoded_meta.get("capture_dir") or ""))
    capture_meta: Dict[str, Any] = {}
    meta_path = os.path.join(capture_dir, "capture_meta.json")
    if not os.path.isfile(meta_path):
        meta_path = os.path.join(capture_dir, "collection_meta.json")
    if os.path.isfile(meta_path):
        capture_meta = _read_json_file(meta_path)
    user_meta = capture_meta.get("metadata") if isinstance(capture_meta.get("metadata"), dict) else {}
    fps_value = float(encoded_meta.get("fps") or capture_meta.get("fps") or 30.0)
    fps: int | float = int(fps_value) if fps_value.is_integer() else fps_value
    alignment = encoded_meta.get("alignment") if isinstance(encoded_meta.get("alignment"), dict) else {}
    reused_count_raw = alignment.get("reused_sample_count_by_key")
    reused_rate_raw = alignment.get("reused_sample_rate_by_key")
    if not isinstance(reused_count_raw, dict):
        reused_count_raw = {}
    if not isinstance(reused_rate_raw, dict):
        reused_rate_raw = {}
    return {
        "task_description": str(user_meta.get("task_description") or capture_meta.get("task_description") or ""),
        "task_prompt": str(user_meta.get("task_prompt") or capture_meta.get("task_prompt") or ""),
        "total_frames": int(encoded.get("total_frames") or encoded_meta.get("total_frames") or 0),
        "fps": fps,
        "collection_time": int(capture_meta.get("started_at_unix") or capture_meta.get("created_at_unix") or time.time()),
        "reused_sample_count_by_key": {str(k): int(v or 0) for k, v in reused_count_raw.items()},
        "reused_sample_rate_by_key": {str(k): float(v or 0.0) for k, v in reused_rate_raw.items()},
    }

