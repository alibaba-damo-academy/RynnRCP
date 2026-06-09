# rcp_core/data_server/data_server.py

"""
Data collection and export server.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Simple data recording: store each key as separate stream, align on export.

Recording:
  - Each key (action, state, images) saved to its own file pair
  - Arrays: key.npz (timestamps + data)
  - Images: key.bin (raw bytes) + key.json (timestamp index)
  - No frame alignment during recording

Export:
  - Load all streams, align by timestamp
  - Generate timeseries.parquet + videos

Storage structure:
  <episode_dir>/
    episode_meta.json
    streams/
      action.npz, action.json
      state.npz, state.json
      observation.images.xxx.bin, observation.images.xxx.json
"""
import os
import json
import time
import threading
import re
import subprocess
from typing import Any, Dict, List, Tuple, Optional

import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq
import av
from PIL import Image
from io import BytesIO
import zipfile

from ..common.server.base_server import BaseServer
from ..common.bus.rcp_bus import RcpBus
from ..common.bus.progress import ProgressStage, ProgressTracker, ProgressCallback
from rcp_core.common.utils.hardware_codec import (
    get_pyav_codec,
    get_system_vaapi_encoder,
    get_video_encoder_mode,
)
from rcp_core.common.utils.logger import server_logger

logger = server_logger()


# Export pipeline stage definitions - owned by DataServer, not by progress.py
_EXPORT_STAGES: list[ProgressStage] = [
    ProgressStage("scan",            0.05, "Scanning episode directories"),
    ProgressStage("export_episodes", 0.65, "Exporting episodes"),
    ProgressStage("build_zip",       0.25, "Building import_dataset.zip"),
    ProgressStage("finalize",        0.05, "Finalizing"),
]


def _safe_filename(key: str) -> str:
    s = str(key).strip().replace("/", "_")
    return re.sub(r"[^0-9a-zA-Z._-]+", "_", s)


def _safe_path_parts(path: Optional[str]) -> List[str]:
    """Split a user path and sanitize every component."""
    if not path:
        return []
    parts: List[str] = []
    for p in str(path).replace("\\", "/").split("/"):
        p = p.strip()
        if not p or p in (".", ".."):
            continue
        parts.append(_safe_filename(p))
    return parts


class DataServer(BaseServer):
    """
    Simple data recorder: store streams independently, align on export.
    """

    ROOT = "~/.cache/RynnRCPData"

    def __init__(self, config: Dict[str, Any]):
        """Initialize the data server."""
        super().__init__(config, "data_server")
        self._record_lock = threading.Lock()
        self._record_thread: Optional[threading.Thread] = None
        self._record_stop = threading.Event()
        self._record_running = False
        self._record_info: Dict[str, Any] = {}
        
        # Stream writers for each key
        self._stream_writers: Dict[str, Any] = {}

    def _root_dir(self) -> str:
        return os.path.expanduser(self.ROOT)

    def _is_image_key(self, key: str) -> bool:
        return str(key).startswith("observation.images.")

    def _read_json(self, path: str) -> Any:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)

    def _write_json(self, path: str, obj: Any) -> None:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(obj, f, ensure_ascii=False, indent=2)

    def _encode_video_pyav(
        self,
        output_path: str,
        frame_payloads: List[bytes],
        image_meta: Dict[str, Any],
        fps: int,
        width: int,
        height: int,
        codec_name: str,
        codec_pix_fmt: str,
    ) -> None:
        with av.open(output_path, mode="w") as out:
            stream_enc = out.add_stream(codec_name, rate=fps)
            stream_enc.pix_fmt = codec_pix_fmt
            stream_enc.width = width
            stream_enc.height = height

            for img_data in frame_payloads:
                try:
                    im = self._decode_image_to_pil(img_data, image_meta)
                    if im.size != (width, height):
                        im = im.resize((width, height), Image.Resampling.BILINEAR)
                    vf = av.VideoFrame.from_image(im)
                    for packet in stream_enc.encode(vf):
                        out.mux(packet)
                except Exception as e:
                    logger.warning(f"[DataServer] encode frame failed: {e}")

            for packet in stream_enc.encode():
                out.mux(packet)

    def _encode_video_ffmpeg_vaapi(
        self,
        output_path: str,
        frame_payloads: List[bytes],
        image_meta: Dict[str, Any],
        fps: int,
        width: int,
        height: int,
        vaapi_device: str,
    ) -> None:
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "warning",
            "-y",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-s",
            f"{width}x{height}",
            "-r",
            str(fps),
            "-i",
            "pipe:0",
            "-vaapi_device",
            vaapi_device,
            "-vf",
            "format=nv12,hwupload",
            "-c:v",
            "h264_vaapi",
            "-movflags",
            "+faststart",
            output_path,
        ]
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        assert proc.stdin is not None
        try:
            for img_data in frame_payloads:
                im = self._decode_image_to_pil(img_data, image_meta).convert("RGB")
                if im.size != (width, height):
                    im = im.resize((width, height), Image.Resampling.BILINEAR)
                proc.stdin.write(im.tobytes())
            proc.stdin.close()
            stderr = proc.stderr.read() if proc.stderr else b""
            returncode = proc.wait()
        except Exception:
            if proc.stdin is not None and not proc.stdin.closed:
                proc.stdin.close()
            proc.kill()
            proc.communicate()
            raise
        if returncode != 0:
            err = stderr.decode("utf-8", errors="replace") if stderr else ""
            raise RuntimeError(f"ffmpeg vaapi failed with code {returncode}: {err}")

    def _encode_video(
        self,
        output_path: str,
        frame_payloads: List[bytes],
        image_meta: Dict[str, Any],
        fps: float,
        width: int,
        height: int,
        label: str,
    ) -> str:
        fps_int = max(1, int(round(fps)))
        codec_name, codec_pix_fmt = get_pyav_codec()
        vaapi = get_system_vaapi_encoder()
        mode = get_video_encoder_mode()
        if vaapi and mode != "pyav" and codec_name != "h264_vaapi":
            vaapi_device, vaapi_codec = vaapi
            try:
                logger.info(
                    f"[DataServer] encoding {label} via ffmpeg VAAPI frames={len(frame_payloads)} "
                    f"fps={fps_int} codec={vaapi_codec} device={vaapi_device}"
                )
                self._encode_video_ffmpeg_vaapi(
                    output_path,
                    frame_payloads,
                    image_meta,
                    fps_int,
                    width,
                    height,
                    vaapi_device,
                )
                return vaapi_codec
            except Exception as e:
                if mode == "ffmpeg_vaapi":
                    raise
                logger.warning(f"[DataServer] ffmpeg VAAPI encode failed, falling back to PyAV: {e}")

        if mode == "software":
            codec_name, codec_pix_fmt = "libx264", "yuv420p"
        logger.info(
            f"[DataServer] encoding {label} via PyAV frames={len(frame_payloads)} "
            f"fps={fps_int} codec={codec_name}"
        )
        self._encode_video_pyav(
            output_path,
            frame_payloads,
            image_meta,
            fps_int,
            width,
            height,
            codec_name,
            codec_pix_fmt,
        )
        return codec_name

    def _make_episode_dir(
        self, task_prompt: str, round_number: int, data_coll_id: Optional[str] = None
    ) -> str:
        """Construct episode directory path from collection parameters.

        Storage structure:
            ~/.cache/RynnRCPData/data_coll/<data_coll_id>/<task_prompt>/episode_NNNNNN/
        """
        parts = [self._root_dir(), "data_coll", *_safe_path_parts(data_coll_id)]
        parts.append(_safe_filename(task_prompt))
        parts.append(f"episode_{int(round_number):06d}")
        out = os.path.join(*parts)
        logger.info(
            f"[DataServer] make_episode_dir data_coll_id={data_coll_id!r} "
            f"task_prompt={task_prompt!r} round={round_number} -> {out}"
        )
        return out

    def _make_record_dir(self, record_id: str) -> str:
        """Construct record directory path for skill execution record collection.

        Storage structure:
            ~/.cache/RynnRCPData/skill_execute_record/<record_id>/
        """
        out = os.path.join(self._root_dir(), "skill_execute_record", _safe_filename(record_id))
        logger.info(f"[DataServer] make_record_dir record_id={record_id!r} -> {out}")
        return out

    # ------------------------ Stream storage helpers ------------------------
    
    def _open_stream_writers(self, episode_dir: str, keys: List[str]):
        """Open stream writers for all keys."""
        streams_dir = os.path.join(episode_dir, "streams")
        os.makedirs(streams_dir, exist_ok=True)
        
        for key in keys:
            if self._is_image_key(key):
                short = key.replace("observation.images.", "")
                bin_path = os.path.join(streams_dir, f"images_{short}.bin")
                self._stream_writers[key] = {
                    "type": "image",
                    "file": open(bin_path, "wb"),
                    "timestamps": [],
                    "offsets": [],
                    "lengths": [],
                    "image_meta": None,  # filled from first frame
                }
            else:
                self._stream_writers[key] = {
                    "type": "array",
                    "timestamps": [],
                    "data": [],
                }
    
    def _write_to_stream(self, key: str, ts: float, value: Any):
        """Write data to stream, skipping if timestamp unchanged (no new data)."""
        writer = self._stream_writers.get(key)
        if not writer:
            return

        # Dedup: skip if same timestamp as last written (data hasn't updated)
        if writer["timestamps"] and writer["timestamps"][-1] == ts:
            return

        if writer["type"] == "image":
            data = value.get("data") if isinstance(value, dict) else None
            if data:
                # Capture image metadata from the first frame
                if writer["image_meta"] is None and isinstance(value, dict):
                    writer["image_meta"] = {
                        "type": value.get("type", "compressed"),
                        "encoding": value.get("encoding"),
                        "width": value.get("width"),
                        "height": value.get("height"),
                    }
                
                # Convert raw RGB/BGR to JPEG if needed
                img_type = (value.get("type") or "").lower() if isinstance(value, dict) else ""
                img_encoding = (value.get("encoding") or "").lower() if isinstance(value, dict) else ""
                
                if img_type == "image" and img_encoding in ("rgb8", "bgr8"):
                    # Raw image data - convert to JPEG
                    w = int(value.get("width", 640))
                    h = int(value.get("height", 480))
                    raw_bytes = bytes(data)
                    if len(raw_bytes) == w * h * 3:
                        arr = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((h, w, 3))
                        if img_encoding == "bgr8":
                            arr = arr[..., ::-1]  # BGR to RGB
                        pil_img = Image.fromarray(arr, mode="RGB")
                        buf = BytesIO()
                        pil_img.save(buf, format="JPEG", quality=95)
                        data = buf.getvalue()
                        # Update metadata to reflect compression
                        if writer["image_meta"] is None:
                            writer["image_meta"] = {}
                        writer["image_meta"]["type"] = "compressed"
                        writer["image_meta"]["encoding"] = None
                
                writer["timestamps"].append(ts)
                offset = writer["file"].tell()
                writer["file"].write(bytes(data))
                writer["offsets"].append(offset)
                writer["lengths"].append(len(data))
        else:
            arr = value if isinstance(value, np.ndarray) else np.array(value)
            writer["timestamps"].append(ts)
            writer["data"].append(arr)
    
    def _close_stream_writers(self, episode_dir: str):
        """Close stream writers and save metadata."""
        streams_dir = os.path.join(episode_dir, "streams")
        
        for key, writer in self._stream_writers.items():
            if writer["type"] == "image":
                writer["file"].close()
                short = key.replace("observation.images.", "")
                json_path = os.path.join(streams_dir, f"images_{short}.json")
                self._write_json(json_path, {
                    "timestamps": writer["timestamps"],
                    "offsets": writer["offsets"],
                    "lengths": writer["lengths"],
                    "image_meta": writer.get("image_meta") or {},
                })
            else:
                timestamps = np.array(writer["timestamps"])
                data = np.array(writer["data"]) if writer["data"] else np.array([])
                npz_path = os.path.join(streams_dir, f"{key}.npz")
                np.savez_compressed(npz_path, timestamps=timestamps, data=data)
        
        self._stream_writers.clear()

    # ------------------------ Core recording logic ------------------------

    def _start_recording(
        self,
        keys: List[str],
        fps: float,
        target_dir: str,
        meta: Dict[str, Any],
        log_prefix: str = "",
    ) -> Dict[str, Any]:
        """
        Core recording logic shared by all recording modes.

        Args:
            keys: Buffer keys to record.
            fps: Target recording frame rate.
            target_dir: Directory to save recording data.
            meta: Metadata dictionary to save.
            log_prefix: Prefix for log messages (e.g., "skill" or empty).

        Returns:
            Result dict.
        """
        os.makedirs(target_dir, exist_ok=True)
        logger.info(f"[DataServer] {log_prefix}target_dir={target_dir}".strip())

        self._write_json(os.path.join(target_dir, "episode_meta.json"), meta)

        with self._record_lock:
            self._record_stop.clear()  # ensure event is clear before starting thread
            self._record_info = {
                "target_dir": target_dir,
                "fps": float(fps),
                "frames_written": 0,
            }

        # Open stream writers
        self._open_stream_writers(target_dir, list(keys))

        period = 1.0 / float(fps)

        def loop():
            count = 0
            next_t = time.perf_counter()
            last_stat_count = 0

            logger.info(
                f"[DataServer] {log_prefix}record thread started target_dir={target_dir} period={period:.6f}s"
            )

            last_stat_t = time.time()

            try:
                while not self._record_stop.is_set():
                    sleep_s = next_t - time.perf_counter()
                    if sleep_s > 0:
                        time.sleep(sleep_s)

                    now2 = time.perf_counter()
                    if now2 - next_t > period:
                        next_t = now2
                    next_t += period

                    # Get and write latest data
                    all_buf = self.get_buffer_global() or {}
                    sensor_buf = all_buf.get("sensor_server", {}) or {}
                    action_buf = all_buf.get("action_server", {}) or {}
                    merged = {**sensor_buf, **action_buf}

                    for key in keys:
                        buf = merged.get(key)
                        if buf:
                            latest_ts, latest_val = buf[-1]
                            self._write_to_stream(key, latest_ts, latest_val)

                    count += 1

                    # periodic stats
                    now_wall = time.time()
                    if now_wall - last_stat_t >= 5.0:
                        logger.info(
                            f"[DataServer] {log_prefix}recording: count={count} (+{count - last_stat_count}/5s)"
                        )
                        last_stat_t = now_wall
                        last_stat_count = count

            finally:
                stop_time = time.time()
                try:
                    self._close_stream_writers(target_dir)

                    with self._record_lock:
                        self._record_running = False
                        self._record_info.update({
                            "frames_written": count,
                            "stop_time_unix": stop_time,
                        })

                    logger.info(
                        f"[DataServer] {log_prefix}record thread done: count={count}"
                    )
                except Exception as e:
                    logger.error(f"[DataServer] {log_prefix}cleanup failed: {e}", exc_info=True)

                # Update meta
                try:
                    meta_path = os.path.join(target_dir, "episode_meta.json")
                    m = self._read_json(meta_path)
                    m.update({
                        "stop_time_unix": stop_time,
                        "frames_written": count,
                    })
                    self._write_json(meta_path, m)
                except Exception as e:
                    logger.error(
                        f"[DataServer] {log_prefix}cleanup: update episode_meta.json failed: {e}",
                        exc_info=True,
                    )

        th = threading.Thread(target=loop, daemon=True)
        with self._record_lock:
            self._record_thread = th
        th.start()

        return self.bus.make_result(
            True, result={"target_dir": target_dir, "meta": meta}, message="OK"
        )

    def _stop_recording(self, log_prefix: str = "") -> Dict[str, Any]:
        """Stop the background recording thread."""
        logger.info(f"[DataServer] {log_prefix}stop requested".strip())

        with self._record_lock:
            if not self._record_running:
                logger.warning(f"[DataServer] {log_prefix}stop requested but not running".strip())
                return self.bus.make_result(
                    False, result=self._record_info, message="recording not running"
                )
            self._record_stop.set()
            th = self._record_thread

        if th:
            th.join(timeout=5)

        with self._record_lock:
            if self._record_running:
                logger.error(f"[DataServer] {log_prefix}stop timeout; still stopping".strip())
                return self.bus.make_result(
                    False,
                    result=self._record_info,
                    message="stop timeout; still stopping",
                )
            return self.bus.make_result(True, result=self._record_info, message="OK")

    # ------------------------ public APIs: record ------------------------

    def start_data_collection(
        self,
        keys: List[str],
        task_description: str,
        task_prompt: str,
        fps: float,
        round_number: int,
        data_coll_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        """
        Start background data collection at specified frame rate.

        Spawns a daemon thread that continuously syncs specified keys and writes
        frames to disk. Recording continues until :meth:`stop_data_collection` is called.

        Args:
            keys: Buffer keys to record (e.g., ["observation.images.cam_head", "action"]).
            task_description: Human-readable task description.
            task_prompt: Task identifier for directory naming.
            fps: Target recording frame rate (frames per second).
            round_number: Episode number for this task.
            data_coll_id: Optional collection ID for organizing multiple tasks.

        Returns:
            Result dict with structure:
            {
                "success": bool,
                "message": str,
                "result": {
                    "episode_dir": str,  # path to episode directory
                    "meta": dict,        # episode metadata
                }
            }
        """
        logger.info(
            f"[DataServer] start_data_collection at {time.strftime('%Y-%m-%d %H:%M:%S')} "
            f"data_coll_id={data_coll_id!r} task_prompt={task_prompt!r} "
            f"round_number={round_number} fps={fps} keys={keys}"
        )

        if not isinstance(keys, (list, tuple)) or not keys:
            return self.bus.make_result(
                False, result={}, message="keys must be a non-empty list"
            )
        if not task_prompt:
            return self.bus.make_result(
                False, result={}, message="task_prompt required"
            )
        if fps <= 0:
            return self.bus.make_result(False, result={}, message="fps must be > 0")

        with self._record_lock:
            if self._record_running:
                logger.warning("[DataServer] start requested but already running")
                return self.bus.make_result(
                    False, result=self._record_info, message="recording already running"
                )
            self._record_running = True
            self._record_stop.clear()

        episode_dir = self._make_episode_dir(task_prompt, round_number, data_coll_id)

        meta = {
            "task_description": task_description,
            "task_prompt": task_prompt,
            "round_number": int(round_number),
            "fps": float(fps),
            "keys": list(keys),
            "data_coll_id": data_coll_id,
            "start_time_unix": time.time(),
            "format": {
                "storage": "streams",
                "streams_dir": "streams/",
                "array_format": "key.npz (timestamps + data)",
                "image_format": "images_xxx.bin + images_xxx.json",
            },
        }

        result = self._start_recording(keys, fps, episode_dir, meta, log_prefix="")
        # Rename target_dir to episode_dir for backward compatibility
        if result.get("success"):
            result["result"]["episode_dir"] = result["result"].pop("target_dir")
        return result

    def stop_data_collection(self) -> Dict[str, Any]:
        """Stop the background recording thread."""
        result = self._stop_recording(log_prefix="")
        # Rename target_dir to episode_dir for backward compatibility
        if result.get("success") and result.get("result"):
            result["result"]["episode_dir"] = result["result"].pop("target_dir", None)
        return result

    def start_skill_record(
        self,
        record_id: str,
        task_keys: List[str],
        fps: float,
        task_prompt: str,
        task_description: str,
    ) -> Dict[str, Any]:
        """
        Start background skill execution record collection.

        Similar to start_data_collection but uses flat directory structure:
            ~/.cache/RynnRCPData/<record_id>/

        Args:
            record_id: Unique identifier for this record.
            task_keys: Buffer keys to record (e.g., ["observation.images.front", "action"]).
            fps: Target recording frame rate (frames per second).
            task_prompt: Task identifier.
            task_description: Human-readable task description.

        Returns:
            Result dict with structure:
            {
                "success": bool,
                "message": str,
                "result": {
                    "record_dir": str,  # path to record directory
                    "meta": dict,       # record metadata
                }
            }
        """
        logger.info(
            f"[DataServer] start_skill_record at {time.strftime('%Y-%m-%d %H:%M:%S')} "
            f"record_id={record_id!r} fps={fps} task_keys={task_keys}"
        )

        if not isinstance(task_keys, (list, tuple)) or not task_keys:
            return self.bus.make_result(
                False, result={}, message="task_keys must be a non-empty list"
            )
        if not record_id:
            return self.bus.make_result(
                False, result={}, message="record_id required"
            )
        if fps <= 0:
            return self.bus.make_result(False, result={}, message="fps must be > 0")

        with self._record_lock:
            if self._record_running:
                logger.warning("[DataServer] start_skill_record requested but already running")
                return self.bus.make_result(
                    False, result=self._record_info, message="recording already running"
                )
            self._record_running = True
            self._record_stop.clear()

        record_dir = self._make_record_dir(record_id)

        meta = {
            "record_id": record_id,
            "task_description": task_description,
            "task_prompt": task_prompt,
            "fps": float(fps),
            "keys": list(task_keys),
            "start_time_unix": time.time(),
            "record_type": "skill_execution",
            "format": {
                "storage": "streams",
                "streams_dir": "streams/",
                "array_format": "key.npz (timestamps + data)",
                "image_format": "images_xxx.bin + images_xxx.json",
            },
        }

        result = self._start_recording(task_keys, fps, record_dir, meta, log_prefix="skill ")
        # Rename target_dir to record_dir for backward compatibility
        if result.get("success"):
            result["result"]["record_dir"] = result["result"].pop("target_dir")
        return result

    def stop_skill_record(self) -> Dict[str, Any]:
        """Stop the background skill recording thread."""
        result = self._stop_recording(log_prefix="skill ")
        # Rename target_dir to record_dir for backward compatibility
        if result.get("success") and result.get("result"):
            result["result"]["record_dir"] = result["result"].pop("target_dir", None)
        return result

    # ------------------------ export helpers ------------------------

    def _scan_episode_dirs(self, data_coll_id: str) -> List[Tuple[str, str]]:
        """
        Recursively scan for episode directories under the data collection root.

        Args:
            data_coll_id: Collection identifier for base path.

        Returns:
            List of (episode_dir, task_prompt_dir) tuples, sorted by path.
        """
        base = os.path.join(self._root_dir(), "data_coll", *_safe_path_parts(data_coll_id))
        logger.info(f"[DataServer] scan episodes under base={base}")
        if not os.path.isdir(base):
            logger.warning(f"[DataServer] base not found: {base}")
            return []

        out: List[Tuple[str, str]] = []
        for cur, dirs, _files in os.walk(base):
            for d in dirs:
                if not d.startswith("episode_"):
                    continue
                ep_dir = os.path.join(cur, d)
                if os.path.exists(os.path.join(ep_dir, "episode_meta.json")):
                    out.append((ep_dir, cur))
        out.sort()
        logger.info(f"[DataServer] scan found episodes={len(out)}")
        return out

    def _get_episode_state(self, ep_path: str) -> Dict[str, Any]:
        """
        Check episode directory state and return status info.

        Episode states:
        - State 1 (raw): has streams/ directory -> can be encoded or exported directly
        - State 2 (encoded): has metadata.json + timeseries.parquet + .mp4 videos, no streams/ -> can only be exported
        - Invalid: any other format -> skip/recommend delete

        Returns:
            Dict with keys:
            - state: "raw" | "encoded" | "invalid"
            - has_streams: bool
            - has_ep_meta: bool
            - has_meta: bool
            - has_parquet: bool
            - videos: List[str]
            - error: str (only for invalid state)
        """
        has_streams = os.path.isdir(os.path.join(ep_path, "streams"))
        has_ep_meta = os.path.exists(os.path.join(ep_path, "episode_meta.json"))
        has_meta = os.path.exists(os.path.join(ep_path, "metadata.json"))
        has_parquet = os.path.exists(os.path.join(ep_path, "timeseries.parquet"))
        videos = [f for f in os.listdir(ep_path) if f.endswith(".mp4")] if os.path.isdir(ep_path) else []

        # State 2: Already fully encoded
        if not has_streams and has_meta and has_parquet and videos:
            return {
                "state": "encoded",
                "has_streams": has_streams,
                "has_ep_meta": has_ep_meta,
                "has_meta": has_meta,
                "has_parquet": has_parquet,
                "videos": videos,
            }

        # State 1: Raw data with streams
        if has_streams and has_ep_meta:
            return {
                "state": "raw",
                "has_streams": has_streams,
                "has_ep_meta": has_ep_meta,
                "has_meta": has_meta,
                "has_parquet": has_parquet,
                "videos": videos,
            }

        # Invalid state
        return {
            "state": "invalid",
            "has_streams": has_streams,
            "has_ep_meta": has_ep_meta,
            "has_meta": has_meta,
            "has_parquet": has_parquet,
            "videos": videos,
            "error": f"Invalid episode format: streams={has_streams}, ep_meta={has_ep_meta}, meta={has_meta}, parquet={has_parquet}, videos={len(videos)}. Recommend delete.",
        }

    def _process_episode_for_export(
        self,
        ep_path: str,
        ep_name: str,
        state: Dict[str, Any],
    ) -> Optional[Dict[str, Any]]:
        """
        Process a single episode for export based on its state.

        Args:
            ep_path: Episode directory path.
            ep_name: Episode name (basename).
            state: Episode state dict from _get_episode_state().

        Returns:
            Export entry dict if successful, None if failed.
        """
        if state["state"] == "encoded":
            # State 2: Already fully encoded -> export directly
            meta = self._read_json(os.path.join(ep_path, "metadata.json"))
            return {
                "episode_path": ep_path,
                "episode": ep_name,
                "videos": state["videos"],
                "original_videos": meta.get("original_videos", []),
                "total_frames": meta.get("total_frames", 0),
                "fps": meta.get("fps", 30),
            }

        if state["state"] == "raw":
            # State 1: Raw data with streams -> encode then export
            try:
                r = self._export_one_episode(ep_path)
                # Verify encoding actually produced videos
                if not r.get("videos"):
                    logger.warning(f"[DataServer] export failed (no videos) episode={ep_name} path={ep_path}")
                    return None
                return {
                    "episode_path": ep_path,
                    "episode": ep_name,
                    "videos": r.get("videos", []),
                    "original_videos": r.get("original_videos", []),
                    "total_frames": r["total_frames"],
                    "fps": r["fps"],
                }
            except Exception as e:
                logger.exception(f"[DataServer] export failed episode={ep_name} path={ep_path}")
                return None

        # Invalid state
        logger.warning(f"[DataServer] export skipped (invalid format) episode={ep_name} path={ep_path}")
        return None

    def _decode_image_to_pil(self, b: bytes, image_meta: Dict[str, Any]) -> Image.Image:
        """Decode image bytes to PIL Image based on metadata type/encoding."""
        t = (image_meta.get("type") or "").lower()
        if t == "compressed":
            return Image.open(BytesIO(b)).convert("RGB")

        if t == "image":
            enc = (image_meta.get("encoding") or "").lower()
            w = int(image_meta["width"])
            h = int(image_meta["height"])
            if enc not in ("rgb8", "bgr8"):
                raise ValueError(f"unsupported raw encoding={enc}")
            arr = np.frombuffer(b, dtype=np.uint8).reshape((h, w, 3))
            if enc == "bgr8":
                arr = arr[..., ::-1]
            return Image.fromarray(arr, mode="RGB")

        raise ValueError(f"unsupported image type={t}")

    def _cleanup_empty_dirs(self, base_dir: str) -> None:
        """
        Recursively remove empty directories up from base_dir.
        Stops at data_coll level to avoid removing the root.
        """
        import shutil
        current = base_dir
        data_coll_root = os.path.join(self._root_dir(), "data_coll")

        while current and current.startswith(data_coll_root):
            if current == data_coll_root:
                break
            try:
                if os.path.isdir(current) and not os.listdir(current):
                    parent = os.path.dirname(current)
                    os.rmdir(current)
                    logger.info(f"[DataServer] removed empty dir: {current}")
                    current = parent
                else:
                    break
            except Exception as e:
                logger.debug(f"[DataServer] cleanup dir {current}: {e}")
                break

    def _build_import_zip_from_base(
        self,
        base_dir: str,
        zip_name: str = "import_dataset",
        tracker: Optional[ProgressTracker] = None,
    ) -> Tuple[Optional[str], List[Dict[str, Any]]]:
        """
        Package exported episodes into <zip_name>.zip.
        Structure: <task_prompt>/<episode_name>/files...
        """
        pack_failed: List[Dict[str, Any]] = []
        logger.info(f"[DataServer] build import zip from base_dir={base_dir} zip_name={zip_name}")

        # Find all episode dirs and their task_prompt parent
        episode_entries = []  # List of (ep_dir, task_prompt_dir)
        for root, dirs, files in os.walk(base_dir):
            for d in dirs:
                if d.startswith("episode_"):
                    ep_dir = os.path.join(root, d)
                    if os.path.exists(os.path.join(ep_dir, "metadata.json")):
                        # task_prompt_dir is the parent of episode_dir
                        task_prompt_dir = os.path.dirname(ep_dir)
                        episode_entries.append((ep_dir, task_prompt_dir))

        episode_entries.sort(key=lambda x: x[0])
        if not episode_entries:
            return None, [{"stage": "zip", "error": "no episodes found"}]

        zip_path = os.path.join(base_dir, f"{zip_name}.zip")
        with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
            for idx, (ep_dir, task_prompt_dir) in enumerate(episode_entries):
                ep_name = os.path.basename(ep_dir)
                task_prompt_name = os.path.basename(task_prompt_dir)
                if tracker:
                    tracker.report(idx, len(episode_entries), f"Packaging {task_prompt_name}/{ep_name}")

                # Build arcname with task_prompt as outer folder
                arc_prefix = os.path.join(task_prompt_name, ep_name)

                for fn in ["metadata.json", "timeseries.parquet"]:
                    fp = os.path.join(ep_dir, fn)
                    if os.path.exists(fp):
                        zf.write(fp, arcname=os.path.join(arc_prefix, fn))

                for fn in sorted([f for f in os.listdir(ep_dir) if f.endswith(".mp4")]):
                    zf.write(os.path.join(ep_dir, fn), arcname=os.path.join(arc_prefix, fn))

        logger.info(f"[DataServer] zip created: {zip_path}")
        return os.path.abspath(zip_path), pack_failed

    # ------------------------ export: one episode ------------------------

    def _export_one_episode(self, ep_dir: str) -> Dict[str, Any]:
        """
        Export a single episode to standard format.
        Simple version: load streams, align by timestamp, generate outputs.
        """
        logger.info(f"[DataServer] export one episode: {ep_dir}")
        ep_meta = self._read_json(os.path.join(ep_dir, "episode_meta.json"))
        fps = int(round(float(ep_meta.get("fps") or 30)))
        start_time_unix = ep_meta.get("start_time_unix") or time.time()

        # Load all streams from streams/
        streams_dir = os.path.join(ep_dir, "streams")
        if not os.path.isdir(streams_dir):
            raise RuntimeError(f"streams dir not found: {streams_dir}")

        # Load array streams
        array_streams = {}
        for npz_file in os.listdir(streams_dir):
            if npz_file.endswith(".npz"):
                key = npz_file[:-4]  # remove .npz
                with np.load(os.path.join(streams_dir, npz_file), allow_pickle=False) as f:
                    timestamps = f["timestamps"]
                    data = f["data"]
                    array_streams[key] = {"timestamps": timestamps, "data": data}

        # Load image streams
        image_streams = {}
        for json_file in os.listdir(streams_dir):
            if json_file.startswith("images_") and json_file.endswith(".json"):
                key = "observation.images." + json_file.replace("images_", "").replace(".json", "")
                meta = self._read_json(os.path.join(streams_dir, json_file))
                bin_file = json_file.replace(".json", ".bin")
                image_streams[key] = {
                    "timestamps": meta["timestamps"],
                    "offsets": meta["offsets"],
                    "lengths": meta["lengths"],
                    "image_meta": meta.get("image_meta") or {},
                    "bin_path": os.path.join(streams_dir, bin_file),
                }

        if not array_streams and not image_streams:
            raise RuntimeError("no streams found")

        # Determine alignment time range:
        # t_start = max of each key's first timestamp (all keys must have data)
        # t_end   = min of each key's last timestamp  (stop when any key ends)
        all_streams = {**array_streams, **image_streams}
        t_start = max(np.min(s["timestamps"]) for s in all_streams.values())
        t_end   = min(np.max(s["timestamps"]) for s in all_streams.values())
        if t_start >= t_end:
            raise RuntimeError(f"no valid overlap between streams: t_start={t_start} t_end={t_end}")

        interval = 1.0 / float(fps)  # e.g. 33.3ms at 30fps

        # Build aligned frame grid and validate each frame
        # A frame is kept only if ALL keys have a sample within <interval> of the target time
        candidate_times = np.arange(t_start, t_end + interval * 0.5, interval)
        logger.info(
            f"[DataServer] alignment: t_start={t_start:.3f} t_end={t_end:.3f} "
            f"interval={interval*1000:.1f}ms candidates={len(candidate_times)}"
        )

        aligned_rows = []  # (frame_index, t, {key: idx})
        for t in candidate_times:
            frame_ok = True
            key_indices: Dict[str, int] = {}
            for key, stream in all_streams.items():
                ts_arr = np.asarray(stream["timestamps"])
                idx = int(np.argmin(np.abs(ts_arr - t)))
                if abs(ts_arr[idx] - t) >= interval:
                    frame_ok = False
                    break
                key_indices[key] = idx
            if frame_ok:
                aligned_rows.append((t, key_indices))

        total_frames = len(aligned_rows)
        logger.info(f"[DataServer] {total_frames} aligned frames kept (dropped {len(candidate_times) - total_frames})")

        if total_frames == 0:
            raise RuntimeError("no frames survived alignment")

        # Build timeseries.parquet from aligned frames
        # timestamp column uses index-based time (0, interval, 2*interval, ...)
        # rather than real wall-clock time, ensuring uniform spacing
        rows = []
        for frame_index, (t, key_indices) in enumerate(aligned_rows):
            row: Dict[str, Any] = {"frame_index": frame_index, "timestamp": round(frame_index * interval, 9)}
            for key, stream in array_streams.items():
                idx = key_indices[key]
                val = stream["data"][idx]
                row[key] = val.tolist() if hasattr(val, "tolist") else val
            rows.append(row)

        df = pd.DataFrame(rows)
        pq_path = os.path.join(ep_dir, "timeseries.parquet")
        pq.write_table(pa.Table.from_pandas(df, preserve_index=False), pq_path)
        logger.info(f"[DataServer] wrote timeseries.parquet rows={len(df)} path={pq_path}")

        # Encode videos for image streams
        videos = []
        original_videos = []

        # Check if this is a skill record (has record_type field)
        is_skill_record = ep_meta.get("record_type") == "skill_execution"

        for key, stream in image_streams.items():
            vp = os.path.join(ep_dir, f"{key}.mp4")

            # Read ALL frames from bin file into memory (used for both videos)
            with open(stream["bin_path"], "rb") as bf:
                all_frames = []  # list of (ts, img_bytes)
                for ts, offset, length in zip(stream["timestamps"], stream["offsets"], stream["lengths"]):
                    bf.seek(offset)
                    all_frames.append((ts, bf.read(length)))

            if not all_frames:
                continue

            # Build a ts->index map for fast lookup
            img_ts_arr = np.array([f[0] for f in all_frames])

            # Determine image format from stored metadata; fall back to auto-detect
            stored_meta = stream.get("image_meta") or {}
            img_type = (stored_meta.get("type") or "").lower()

            if img_type in ("image", "compressed") and (
                img_type == "compressed" or (stored_meta.get("width") and stored_meta.get("height"))
            ):
                first_meta = stored_meta
                try:
                    im0 = self._decode_image_to_pil(all_frames[0][1], first_meta)
                    w, h = im0.size
                    logger.info(
                        f"[DataServer] image format from metadata: type={img_type} "
                        f"encoding={stored_meta.get('encoding')} size={w}x{h} for {key}"
                    )
                except Exception as e:
                    logger.warning(f"[DataServer] metadata decode failed for {key}: {e}, trying auto-detect")
                    first_meta = None
            else:
                first_meta = None

            if first_meta is None:
                # Auto-detect: try compressed first, then rgb8
                first_meta = {"type": "compressed"}
                try:
                    im0 = self._decode_image_to_pil(all_frames[0][1], first_meta)
                    w, h = im0.size
                    logger.info(f"[DataServer] image format=compressed (auto) for {key}, size={w}x{h}")
                except Exception as compress_err:
                    first_meta = {"type": "image", "encoding": "rgb8",
                                  "width": stored_meta.get("width", 640),
                                  "height": stored_meta.get("height", 480)}
                    try:
                        im0 = self._decode_image_to_pil(all_frames[0][1], first_meta)
                        w, h = im0.size
                        logger.info(f"[DataServer] image format=rgb8 (auto) for {key}, size={w}x{h}")
                    except Exception as e:
                        logger.warning(
                            f"[DataServer] failed to decode first frame for {key}: "
                            f"compressed_err={compress_err}, rgb8_err={e}, "
                            f"frame_size={len(all_frames[0][1])} bytes, "
                            f"first_bytes={all_frames[0][1][:16].hex()}"
                        )
                        continue

            # --- Aligned video: one frame per aligned_rows entry, selected by nearest timestamp ---
            aligned_payloads = [all_frames[key_indices[key]][1] for _t, key_indices in aligned_rows]
            used_codec = self._encode_video(vp, aligned_payloads, first_meta, fps, w, h, f"aligned video for {key}")

            videos.append(os.path.basename(vp))
            logger.info(f"[DataServer] encoded aligned video {vp} codec={used_codec}")

            # --- Original video: all captured frames, fps inferred from actual timestamps ---
            if is_skill_record:
                original_dir = os.path.join(ep_dir, "original_videos")
                os.makedirs(original_dir, exist_ok=True)
                original_vp = os.path.join(original_dir, f"original.{key}.mp4")

                if len(all_frames) > 1:
                    duration = all_frames[-1][0] - all_frames[0][0]
                    original_fps = len(all_frames) / duration if duration > 0 else fps
                else:
                    original_fps = fps

                original_payloads = [img_data for _ts, img_data in all_frames]
                used_original_codec = self._encode_video(
                    original_vp,
                    original_payloads,
                    first_meta,
                    original_fps,
                    w,
                    h,
                    f"original video for {key}",
                )

                original_videos.append(os.path.basename(original_vp))
                logger.info(f"[DataServer] encoded original video {original_vp} codec={used_original_codec}")

        # Write metadata.json
        meta_out = {
            "task_description": ep_meta.get("task_description"),
            "task_prompt": ep_meta.get("task_prompt"),
            "total_frames": total_frames,
            "fps": int(fps),
            "collection_time": int(start_time_unix),
        }
        if is_skill_record:
            meta_out["record_id"] = ep_meta.get("record_id")
            meta_out["record_type"] = "skill_execution"
            if original_videos:
                meta_out["original_videos"] = {"dir": "original_videos", "files": original_videos}
        self._write_json(os.path.join(ep_dir, "metadata.json"), meta_out)
        logger.info(f"[DataServer] wrote metadata.json episode={ep_dir}")

        # Remove raw streams directory now that videos and parquet are generated
        if videos:
            import shutil as _shutil
            streams_dir = os.path.join(ep_dir, "streams")
            if os.path.isdir(streams_dir):
                try:
                    _shutil.rmtree(streams_dir)
                    logger.info(f"[DataServer] removed raw streams dir: {streams_dir}")
                except Exception as e:
                    logger.warning(f"[DataServer] failed to remove streams dir: {e}")

        result = {"videos": videos, "total_frames": total_frames, "fps": int(fps)}
        if is_skill_record and original_videos:
            result["original_videos"] = original_videos
        return result

    # ------------------------ public API: export ------------------------

    def export_task_episodes(
        self,
        data_coll_id: str,
        _progress_callback: Optional[ProgressCallback] = None,
    ) -> Dict[str, Any]:
        """
        Export all episodes under a data collection to standard format.

        For each episode:
        1. Validates frame alignment and consistency
        2. Exports to metadata.json + timeseries.parquet + videos
        3. Removes raw frames/ and episode_meta.json on success

        After all episodes are processed:
        - Creates import_dataset.zip containing all packable episodes
        - Deletes the original directory structure after successful packaging

        Args:
            data_coll_id: Collection identifier to locate episodes.
            _progress_callback: Optional ``(current, total, message) -> None`` callable.
                                 When provided, the caller receives incremental progress
                                 updates (0–100 %) as the export pipeline advances through
                                 its stages.  The parameter name is prefixed with ``_`` to
                                 signal that it is an infrastructure concern, not a business
                                 input; :class:`~rcp_core.common.bus.rcp_bus.RcpBus` detects
                                 and forwards it automatically.

        Returns:
            Result dict with structure:
            {
                "success": bool,
                "message": str,
                "result": {
                    "base_dir": str,
                    "exported": List[dict],  # successfully exported episodes
                    "failed": List[dict],    # failed episodes with errors
                    "zip_path": str | None,  # path to created zip
                }
            }
        """
        if not data_coll_id:
            return self.bus.make_result(False, result={}, message="data_coll_id required")

        tracker = ProgressTracker(_EXPORT_STAGES, _progress_callback)

        base_dir = os.path.abspath(
            os.path.join(self._root_dir(), "data_coll", *_safe_path_parts(data_coll_id))
        )
        logger.info(
            f"[DataServer] export_task_episodes data_coll_id={data_coll_id!r} base_dir={base_dir}"
        )

        # Stage: scan
        tracker.enter_stage("scan")
        episodes = self._scan_episode_dirs(data_coll_id)
        if not episodes:
            return self.bus.make_result(False, result={}, message="no episodes found")

        total_episodes = len(episodes)
        exported: List[Dict[str, Any]] = []
        failed: List[Dict[str, Any]] = []

        # Stage: export each episode
        tracker.enter_stage("export_episodes")
        for i, (ep_dir, _task_prompt_dir) in enumerate(episodes):
            ep_name = os.path.basename(ep_dir)
            tracker.report(i, total_episodes, f"Exporting {ep_name} ({i + 1}/{total_episodes})")
            try:
                r = self._export_one_episode(ep_dir)
                exported.append(
                    {
                        "episode_dir": ep_dir,
                        "episode": ep_name,
                        "videos": r["videos"],
                        "total_frames": r["total_frames"],
                        "fps": r["fps"],
                    }
                )
                logger.info(
                    f"[DataServer] export success episode={ep_name} dir={ep_dir}"
                )
            except Exception as e:
                logger.exception(
                    f"[DataServer] export failed episode={ep_name} dir={ep_dir}"
                )
                failed.append(
                    {"episode_dir": ep_dir, "episode": ep_name, "error": str(e)}
                )

        # Stage: build zip
        tracker.enter_stage("build_zip")
        # Use timestamp-based name: teleop_dataset_XXXXXX.zip
        zip_name = f"teleop_dataset_{int(time.time())}"
        zip_path, pack_failed = self._build_import_zip_from_base(base_dir, zip_name=zip_name, tracker=tracker)
        failed.extend(pack_failed)

        # Stage: finalize
        tracker.enter_stage("finalize")
        logger.info(
            f"[DataServer] export_task_episodes done exported={len(exported)} failed={len(failed)} zip={zip_path}"
        )

        # Move zip to export_dir and cleanup original data on success
        final_zip_path = zip_path
        if zip_path and os.path.exists(zip_path) and len(exported) > 0:
            export_dir = os.path.join(self._root_dir(), "data_coll", "export_dir")
            os.makedirs(export_dir, exist_ok=True)
            final_zip_path = os.path.join(export_dir, os.path.basename(zip_path))
            # Remove existing file if present
            if os.path.exists(final_zip_path):
                os.remove(final_zip_path)
            os.rename(zip_path, final_zip_path)
            logger.info(f"[DataServer] moved zip to {final_zip_path}")

            # Delete original data for successfully exported episodes
            import shutil
            deleted_episodes = []
            for entry in exported:
                ep_dir = entry.get("episode_dir")
                if ep_dir and os.path.exists(ep_dir):
                    try:
                        shutil.rmtree(ep_dir)
                        deleted_episodes.append(ep_dir)
                        logger.info(f"[DataServer] deleted original episode data: {ep_dir}")
                    except Exception as e:
                        logger.warning(f"[DataServer] failed to delete {ep_dir}: {e}")

            # Try to cleanup empty task_prompt and data_coll directories
            # Start from task_prompt level so it walks up through data_coll_id dir
            cleaned = set()
            for entry in exported:
                ep_dir = entry.get("episode_dir")
                if ep_dir:
                    task_prompt_dir = os.path.dirname(ep_dir)  # .../data_coll_id/task_prompt
                    if task_prompt_dir not in cleaned:
                        cleaned.add(task_prompt_dir)
                        self._cleanup_empty_dirs(task_prompt_dir)

        success = len(exported) > 0
        message = "OK" if success else "all episodes failed"
        tracker.complete(f"Export done: {len(exported)} succeeded, {len(failed)} failed")

        return self.bus.make_result(
            success,
            result={
                "base_dir": base_dir,
                "exported": exported,
                "failed": failed,
                "zip_path": final_zip_path,
            },
            message=message,
        )

    def encode_episodes(
        self,
        episode_paths: List[str],
        _progress_callback: Optional[ProgressCallback] = None,
    ) -> Dict[str, Any]:
        """
        Encode (export in-place) specific episodes without creating ZIP or deleting.
        This converts raw stream files to metadata.json + timeseries.parquet + videos.

        Episode states:
        - State 1 (raw): has streams/ directory -> can be encoded or exported directly
        - State 2 (encoded): has metadata.json + timeseries.parquet + .mp4 videos, no streams/ -> can only be exported
        - Invalid: any other format -> skip/recommend delete

        Args:
            episode_paths: List of episode directory paths to encode.
            _progress_callback: Optional progress callback.

        Returns:
            Result dict with structure:
            {
                "success": bool,
                "message": str,
                "result": {
                    "encoded": List[dict],
                    "failed": List[dict],
                }
            }
        """
        if not isinstance(episode_paths, (list, tuple)) or not episode_paths:
            return self.bus.make_result(False, result={}, message="episode_paths must be a non-empty list")

        logger.info(f"[DataServer] encode_episodes paths={episode_paths}")

        encoded: List[Dict[str, Any]] = []
        failed: List[Dict[str, Any]] = []

        # Encode each episode
        for ep_path in episode_paths:
            ep_path = os.path.expanduser(ep_path)
            if not os.path.isdir(ep_path):
                failed.append({"episode_path": ep_path, "error": "directory not found"})
                continue

            ep_name = os.path.basename(ep_path)

            # Check episode state using unified method
            state = self._get_episode_state(ep_path)

            # State 2: Already fully encoded (no streams, has metadata + parquet + videos)
            if state["state"] == "encoded":
                # Already encoded: return success with existing info
                meta = self._read_json(os.path.join(ep_path, "metadata.json"))
                encoded.append({
                    "episode_path": ep_path,
                    "episode": ep_name,
                    "videos": state["videos"],
                    "original_videos": meta.get("original_videos", []),
                    "total_frames": meta.get("total_frames", 0),
                    "fps": meta.get("fps", 30),
                })
                logger.info(f"[DataServer] encode success (already encoded) episode={ep_name} path={ep_path}")
                continue

            # State 1: Raw data with streams directory -> encode
            if state["state"] == "raw":
                try:
                    r = self._export_one_episode(ep_path)
                    # Verify encoding actually produced videos
                    if not r.get("videos"):
                        failed.append({
                            "episode_path": ep_path,
                            "episode": ep_name,
                            "error": "Encoding produced no videos (image stream may be empty or corrupt)"
                        })
                        logger.warning(f"[DataServer] encode failed (no videos) episode={ep_name} path={ep_path}")
                        continue
                    encoded.append({
                        "episode_path": ep_path,
                        "episode": ep_name,
                        "videos": r.get("videos", []),
                        "original_videos": r.get("original_videos", []),
                        "total_frames": r["total_frames"],
                        "fps": r["fps"],
                    })
                    logger.info(f"[DataServer] encode success (encoded) episode={ep_name} path={ep_path}")
                except Exception as e:
                    logger.exception(f"[DataServer] encode failed episode={ep_name} path={ep_path}")
                    failed.append({"episode_path": ep_path, "episode": ep_name, "error": str(e)})
                continue

            # Invalid state: recommend delete
            failed.append({
                "episode_path": ep_path,
                "episode": ep_name,
                "error": state.get("error", "Invalid episode format")
            })
            logger.warning(f"[DataServer] encode skipped (invalid format) episode={ep_name} path={ep_path}")

        success = len(encoded) > 0
        message = f"Encoded {len(encoded)} episodes, {len(failed)} failed"

        return self.bus.make_result(
            success,
            result={
                "encoded": encoded,
                "failed": failed,
            },
            message=message,
        )

    def export_specific_episodes(
        self,
        episode_paths: List[str],
        zip_name: Optional[str] = None,
        _progress_callback: Optional[ProgressCallback] = None,
    ) -> Dict[str, Any]:
        """
        Export specific episodes by their full paths.

        Episode states:
        - State 1 (raw): has streams/ directory -> encode then export
        - State 2 (encoded): has metadata.json + timeseries.parquet + .mp4 videos, no streams/ -> export directly
        - Invalid: any other format -> skip

        Args:
            episode_paths: List of episode directory paths (e.g., 
                ["~/.cache/RynnRCPData/data_coll/teleop_123/teleop_demo/episode_000001", ...])
            zip_name: Optional name for the output zip file (without extension).
                     If not provided, uses "exported_episodes".
            _progress_callback: Optional progress callback.

        Returns:
            Result dict with structure:
            {
                "success": bool,
                "message": str,
                "result": {
                    "zip_path": str,
                    "exported": List[dict],
                    "failed": List[dict],
                }
            }
        """
        if not isinstance(episode_paths, (list, tuple)) or not episode_paths:
            return self.bus.make_result(False, result={}, message="episode_paths must be a non-empty list")

        # Check all episodes are from the same data_coll_id
        data_coll_ids = set()
        for ep_path in episode_paths:
            # Path: .../data_coll/<data_coll_id>/<task_prompt>/episode_xxx
            parts = os.path.normpath(ep_path).split(os.sep)
            if len(parts) >= 2:
                # Find data_coll_id (parent of task_prompt)
                data_coll_idx = -1
                for i, p in enumerate(parts):
                    if p == "data_coll" and i + 1 < len(parts):
                        data_coll_idx = i + 1
                        break
                if data_coll_idx >= 0 and data_coll_idx < len(parts):
                    data_coll_ids.add(parts[data_coll_idx])
        if len(data_coll_ids) > 1:
            return self.bus.make_result(
                False,
                result={},
                message=f"一次只能导出同一个数采批次的数据，当前选择了 {len(data_coll_ids)} 个批次: {', '.join(sorted(data_coll_ids))}"
            )

        logger.info(f"[DataServer] export_specific_episodes paths={episode_paths}")

        export_dir = os.path.join(self._root_dir(), "data_coll", "export_dir")
        os.makedirs(export_dir, exist_ok=True)

        exported: List[Dict[str, Any]] = []
        failed: List[Dict[str, Any]] = []

        # Export each episode
        for ep_path in episode_paths:
            ep_path = os.path.expanduser(ep_path)
            if not os.path.isdir(ep_path):
                failed.append({"episode_path": ep_path, "error": "directory not found"})
                continue

            ep_name = os.path.basename(ep_path)

            # Check episode state using unified method
            state = self._get_episode_state(ep_path)

            # Process episode based on state
            result = self._process_episode_for_export(ep_path, ep_name, state)
            if result:
                exported.append(result)
                status = "pre-encoded" if state["state"] == "encoded" else "encoded"
                logger.info(f"[DataServer] export success ({status}) episode={ep_name} path={ep_path}")
            else:
                if state["state"] == "invalid":
                    failed.append({
                        "episode_path": ep_path,
                        "episode": ep_name,
                        "error": state.get("error", "Invalid episode format")
                    })
                else:
                    failed.append({
                        "episode_path": ep_path,
                        "episode": ep_name,
                        "error": "Encoding produced no videos (image stream may be empty or corrupt)"
                    })

        if not exported:
            return self.bus.make_result(False, result={}, message="all episodes failed to export")

        # Build zip with structure: <task_prompt>/<episode_name>/files...
        # Use timestamp-based name: teleop_dataset_XXXXXX.zip
        ts_suffix = int(time.time())
        zip_name = zip_name or f"teleop_dataset_{ts_suffix}"
        zip_name = _safe_filename(zip_name)
        zip_path = os.path.join(export_dir, f"{zip_name}.zip")

        # Remove existing file if present
        if os.path.exists(zip_path):
            os.remove(zip_path)

        try:
            with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
                for entry in exported:
                    ep_path = entry["episode_path"]
                    # Determine task_prompt from path structure: .../data_coll/<data_coll_id>/<task_prompt>/<episode>
                    # Use normpath + split to handle both / and \ separators consistently
                    normalized_path = os.path.normpath(ep_path)
                    parts = normalized_path.split(os.sep)
                    if len(parts) >= 2:
                        task_prompt = parts[-2] if parts[-2] != "data_coll" else "unknown"
                    else:
                        task_prompt = "unknown"
                    ep_name = entry["episode"]
                    arc_prefix = os.path.join(task_prompt, ep_name)

                    for root, dirs, files in os.walk(ep_path):
                        for fn in files:
                            # Skip episode_meta.json (raw state metadata, not needed after encoding)
                            if fn == "episode_meta.json":
                                continue
                            if fn.endswith(".npz") or fn.endswith(".bin") or fn.endswith(".json"):
                                # Skip raw stream files, only include exported files
                                if "streams" in root:
                                    continue
                            fp = os.path.join(root, fn)
                            rel = os.path.relpath(fp, ep_path)
                            zf.write(fp, arcname=os.path.join(arc_prefix, rel))

            logger.info(f"[DataServer] created zip: {zip_path}")

            # Delete original data for successfully exported episodes
            import shutil
            for entry in exported:
                ep_path = entry.get("episode_path")
                if ep_path and os.path.exists(ep_path):
                    try:
                        shutil.rmtree(ep_path)
                        logger.info(f"[DataServer] deleted original episode data: {ep_path}")
                    except Exception as e:
                        logger.warning(f"[DataServer] failed to delete {ep_path}: {e}")

            # Cleanup empty directories - start from task_prompt level so it walks up through data_coll_id
            cleaned = set()
            for entry in exported:
                ep_path = entry.get("episode_path")
                if ep_path:
                    task_prompt_dir = os.path.dirname(ep_path)  # .../data_coll_id/task_prompt
                    if task_prompt_dir not in cleaned:
                        cleaned.add(task_prompt_dir)
                        self._cleanup_empty_dirs(task_prompt_dir)

        except Exception as e:
            logger.exception(f"[DataServer] failed to create zip: {e}")
            return self.bus.make_result(False, result={}, message=f"zip creation failed: {e}")

        success = len(exported) > 0
        message = f"Exported {len(exported)} episodes, {len(failed)} failed"

        return self.bus.make_result(
            success,
            result={
                "zip_path": zip_path,
                "exported": exported,
                "failed": failed,
            },
            message=message,
        )

    def export_skill_record(
        self,
        record_ids: List[str],
        _progress_callback: Optional[ProgressCallback] = None,
    ) -> Dict[str, Any]:
        """
        Export skill execution records to standard format and package into zip.

        Args:
            record_ids: List of record identifiers to locate the record directories.
            _progress_callback: Optional progress callback.

        Returns:
            Result dict with structure:
            {
                "success": bool,
                "message": str,
                "result": {
                    "zip_path": str,
                    "exported": List[dict],
                    "failed": List[dict],
                }
            }
        """
        if not isinstance(record_ids, (list, tuple)) or not record_ids:
            return self.bus.make_result(False, result={}, message="record_ids must be a non-empty list")

        logger.info(f"[DataServer] export_skill_record record_ids={record_ids}")

        base_dir = os.path.join(self._root_dir(), "skill_execute_record")
        exported: List[Dict[str, Any]] = []
        failed: List[Dict[str, Any]] = []

        # Export each record
        for record_id in record_ids:
            record_dir = os.path.abspath(
                os.path.join(base_dir, _safe_filename(record_id))
            )

            if not os.path.isdir(record_dir):
                failed.append({"record_id": record_id, "error": f"directory not found: {record_dir}"})
                continue

            if not os.path.exists(os.path.join(record_dir, "episode_meta.json")):
                failed.append({"record_id": record_id, "error": "episode_meta.json not found"})
                continue

            try:
                r = self._export_one_episode(record_dir)
                exported.append({
                    "record_id": record_id,
                    "record_dir": record_dir,
                    "videos": r.get("videos", []),
                    "original_videos": r.get("original_videos", []),
                    "total_frames": r["total_frames"],
                    "fps": r["fps"],
                })
            except Exception as e:
                logger.exception(f"[DataServer] export failed for record_id={record_id}")
                failed.append({"record_id": record_id, "error": str(e)})

        if not exported:
            return self.bus.make_result(False, result={}, message="all records failed to export")

        # Build single zip with structure: skill_execute_record/<record_id>/files...
        zip_name = "skill_execute_record"
        zip_path = os.path.join(base_dir, f"{zip_name}.zip")

        try:
            with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
                for entry in exported:
                    record_id = entry["record_id"]
                    record_dir = entry["record_dir"]

                    for root, dirs, files in os.walk(record_dir):
                        for fn in files:
                            # Skip any existing zip files
                            if fn.endswith(".zip"):
                                continue
                            full = os.path.join(root, fn)
                            arcname = os.path.join(zip_name, record_id, os.path.relpath(full, record_dir))
                            zf.write(full, arcname=arcname)

            logger.info(f"[DataServer] export_skill_record zip created: {zip_path}")
        except Exception as e:
            logger.exception(f"[DataServer] failed to create zip: {e}")
            return self.bus.make_result(False, result={}, message=f"zip creation failed: {e}")

        return self.bus.make_result(
            True,
            result={
                "zip_path": zip_path,
                "exported": exported,
                "failed": failed,
            },
            message="OK",
        )

    # ------------------------ public API: get record info ------------------------

    def get_record_info(self) -> Dict[str, Any]:
        """Get current recording status."""
        with self._record_lock:
            recording = self._record_running
            info = dict(self._record_info) if self._record_info else {}

        result = {
            "recording": recording,
            "frames_written": info.get("frames_written", 0),
            "target_dir": info.get("target_dir"),
            "fps": info.get("fps"),
        }
        return self.bus.make_result(True, result=result, message="OK")

    # ------------------------ public API: load episode for playback ------------------------

    def load_episode_for_playback(self, episode_path: str) -> Dict[str, Any]:
        """
        Load episode data for playback.
        
        Supports both formats:
        - Encoded: timeseries.parquet + videos
        - Raw: streams/action.npz + streams/state.npz
        
        Returns:
            {
                "success": bool,
                "message": str,
                "result": {
                    "timestamps": List[float],  # original timestamps
                    "actions": List[List[float]],  # action sequence
                    "first_state": List[float],  # first frame observation.state for pre-move
                    "fps": float,
                    "total_frames": int,
                    "format": "encoded" | "raw",
                }
            }
        """
        import pandas as pd
        
        episode_path = os.path.expanduser(episode_path)
        if not os.path.isdir(episode_path):
            return self.bus.make_result(False, result={}, message=f"Episode directory not found: {episode_path}")
        
        try:
            # Check if encoded format (has timeseries.parquet)
            parquet_path = os.path.join(episode_path, "timeseries.parquet")
            if os.path.exists(parquet_path):
                # Encoded format
                df = pd.read_parquet(parquet_path)
                
                # Extract timestamps and actions
                timestamps = df['timestamp'].tolist() if 'timestamp' in df.columns else []
                if not timestamps:
                    # Try to reconstruct from index
                    timestamps = df.index.tolist()
                
                # Extract actions - action column contains list of floats
                if 'action' in df.columns:
                    actions = df['action'].tolist()
                else:
                    return self.bus.make_result(False, result={}, message="No action column found in parquet")
                
                # Get first state for pre-move
                first_state = None
                if 'observation.state' in df.columns:
                    first_state = df['observation.state'].iloc[0]
                    if isinstance(first_state, (list, tuple, np.ndarray)):
                        first_state = list(first_state)
                    else:
                        first_state = None
                
                # Get fps from metadata if available
                fps = 30.0
                metadata_path = os.path.join(episode_path, "metadata.json")
                if os.path.exists(metadata_path):
                    meta = self._read_json(metadata_path)
                    fps = meta.get("fps", 30.0)
                
                result = {
                    "timestamps": timestamps,
                    "actions": actions,
                    "first_state": first_state,
                    "fps": fps,
                    "total_frames": len(timestamps),
                    "format": "encoded",
                }
                return self.bus.make_result(True, result=result, message=f"Loaded encoded episode: {len(timestamps)} frames")
            
            # Check if raw format (has streams/ directory)
            streams_dir = os.path.join(episode_path, "streams")
            if os.path.isdir(streams_dir):
                # Raw format - load from npz files
                action_path = os.path.join(streams_dir, "action.npz")
                state_path = os.path.join(streams_dir, "observation.state.npz")
                
                if not os.path.exists(action_path):
                    return self.bus.make_result(False, result={}, message=f"Action file not found: {action_path}")
                
                # Load action data
                with np.load(action_path, allow_pickle=False) as f:
                    action_timestamps = f["timestamps"]
                    action_data = f["data"]
                
                # Load state data for first frame position
                first_state = None
                if os.path.exists(state_path):
                    with np.load(state_path, allow_pickle=False) as f:
                        state_timestamps = f["timestamps"]
                        state_data = f["data"]
                        if len(state_data) > 0:
                            first_state = state_data[0].tolist() if hasattr(state_data[0], 'tolist') else list(state_data[0])
                
                # Get fps from episode_meta.json
                fps = 30.0
                meta_path = os.path.join(episode_path, "episode_meta.json")
                if os.path.exists(meta_path):
                    meta = self._read_json(meta_path)
                    fps = meta.get("fps", 30.0)
                
                # Convert actions to list of lists
                actions = [a.tolist() if hasattr(a, 'tolist') else list(a) for a in action_data]
                timestamps = action_timestamps.tolist() if hasattr(action_timestamps, 'tolist') else list(action_timestamps)
                
                result = {
                    "timestamps": timestamps,
                    "actions": actions,
                    "first_state": first_state,
                    "fps": fps,
                    "total_frames": len(timestamps),
                    "format": "raw",
                }
                return self.bus.make_result(True, result=result, message=f"Loaded raw episode: {len(timestamps)} frames")
            
            return self.bus.make_result(False, result={}, message="Unknown episode format (no timeseries.parquet or streams/)")
            
        except Exception as e:
            logger.exception(f"[DataServer] Failed to load episode: {e}")
            return self.bus.make_result(False, result={}, message=f"Failed to load episode: {e}")

    # ------------------------ bind bus ------------------------

    def bind_bus(self, bus: RcpBus):
        super().bind_bus(bus)

        bus.add_tool(
            "start_data_collection",
            self.start_data_collection,
            input_schema={
                "keys": "List[str]",
                "task_description": "str",
                "task_prompt": "str",
                "fps": "float",
                "round_number": "int",
                "data_coll_id": "str | None",
            },
            output_schema={"success": "bool", "message": "str", "result": "dict"},
            description=(
                "Start background recording: sync frames at fps and save to "
                "~/.cache/RynnRCPData/data_coll/<data_coll_id>/<task_prompt>/episode_<round>/. "
                "Only writes a frame when data has a new timestamp (dedup). "
                "Arrays saved as streams/<key>.npz, images as streams/images_<name>.bin+json."
            ),
        )

        bus.add_tool(
            "stop_data_collection",
            self.stop_data_collection,
            input_schema={},
            output_schema={"success": "bool", "message": "str", "result": "dict"},
            description="Stop background recording started by start_data_collection.",
        )

        bus.add_tool(
            "export_task_episodes",
            self.export_task_episodes,
            input_schema={"data_coll_id": "str"},
            output_schema={"success": "bool", "message": "str", "result": "dict"},
            description=(
                "Scan ~/.cache/RynnRCPData/data_coll/<data_coll_id>/ for all episode_* and export each episode IN-PLACE "
                "(write metadata.json, timeseries.parquet, observation.images.*.mp4 into each episode directory). "
                "Alignment: t_start=max(first_ts per key), t_end=min(last_ts per key), interval=1/fps; "
                "frames where any key sample is >= interval away are dropped. "
                "Also builds <data_coll_id>.zip under export_dir and deletes original data on success."
            ),
        )

        bus.add_tool(
            "encode_episodes",
            self.encode_episodes,
            input_schema={"episode_paths": "List[str]"},
            output_schema={"success": "bool", "message": "str", "result": "dict"},
            description=(
                "Encode (export in-place) specific episodes without creating ZIP or deleting. "
                "Converts raw stream files to metadata.json + timeseries.parquet + videos. "
                "Input: list of episode directory paths."
            ),
        )

        bus.add_tool(
            "export_specific_episodes",
            self.export_specific_episodes,
            input_schema={"episode_paths": "List[str]", "zip_name": "Optional[str]"},
            output_schema={"success": "bool", "message": "str", "result": "dict"},
            description=(
                "Export specific episodes by their full paths. "
                "Input: list of episode directory paths. "
                "Output zip is saved to ~/.cache/RynnRCPData/data_coll/export_dir/<zip_name>.zip "
                "and original data is deleted on success."
            ),
        )

        bus.add_tool(
            "get_record_info",
            self.get_record_info,
            input_schema={},
            output_schema={"success": "bool", "message": "str", "result": "dict"},
            description="Get current recording status and statistics.",
        )

        # Skill execution record collection tools
        bus.add_tool(
            "start_skill_record",
            self.start_skill_record,
            input_schema={
                "record_id": "str",
                "task_keys": "List[str]",
                "fps": "float",
                "task_prompt": "str",
                "task_description": "str",
            },
            output_schema={"success": "bool", "message": "str", "result": "dict"},
            description=(
                "Start background skill execution recording: sync frames at fps and save to "
                "~/.cache/RynnRCPData/skill_execute_record/<record_id>/. "
                "Only writes a frame when data has a new timestamp (dedup). "
                "Arrays saved as streams/<key>.npz, images as streams/images_<name>.bin+json."
            ),
        )

        bus.add_tool(
            "stop_skill_record",
            self.stop_skill_record,
            input_schema={},
            output_schema={"success": "bool", "message": "str", "result": "dict"},
            description="Stop background skill recording started by start_skill_record.",
        )

        bus.add_tool(
            "export_skill_record",
            self.export_skill_record,
            input_schema={"record_ids": "List[str]"},
            output_schema={"success": "bool", "message": "str", "result": "dict"},
            description=(
                "Export multiple skill execution records and package into a single zip. "
                "Input: list of record_ids. "
                "Output zip structure: skill_execute_record/<record_id>/metadata.json, timeseries.parquet, "
                "observation_images_*.mp4, original_videos/orignal_*.mp4. "
                "Zip is saved to ~/.cache/RynnRCPData/skill_execute_record/skill_execute_record.zip"
            ),
        )

        # Playback tool
        bus.add_tool(
            "load_episode_for_playback",
            self.load_episode_for_playback,
            input_schema={"episode_path": "str"},
            output_schema={"success": "bool", "message": "str", "result": "dict"},
            description=(
                "Load episode data for playback. Supports both encoded (timeseries.parquet) and raw (streams/*.npz) formats. "
                "Returns timestamps, actions, first_state for pre-move, fps, and total_frames. "
                "Input: episode directory path."
            ),
        )
