"""Video encoding helpers for encoded raw captures."""

from __future__ import annotations

import math
import os
import platform
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from fractions import Fraction
from functools import lru_cache
from typing import Any, Callable, Dict, Optional, Sequence

import numpy as np

from rynnrcp_app_common.collection_reader import RawCaptureReader, RawCaptureSample, RawCaptureSampleRef
from rynnrcp.utils import safe_name
from rynnrcp_app_common.hardware_codec import (
    discover_ffmpeg_video_encoder,
    discover_video_encoder,
    ffmpeg_encoder_runtime_args,
    gst_h264_pipeline_args,
)


def _encode_image_videos_parallel(
    reader: RawCaptureReader,
    streams: Dict[str, Sequence[RawCaptureSampleRef]],
    output_dir: str,
    default_fps: float,
    *,
    video_backend: str,
    video_encoder: Optional[str],
    filename_prefix: str,
    rel_dir: str,
    fps_by_key: Optional[Dict[str, float]] = None,
    progress_callback: Optional[Callable[[float, float, str], None]] = None,
) -> tuple[Dict[str, str], Dict[str, Dict[str, Any]]]:
    items = [(key, refs) for key, refs in streams.items() if refs]
    if not items:
        return {}, {}

    progress_totals = {key: float(max(1, len(refs))) for key, refs in items}
    progress_done = {key: 0.0 for key, _refs in items}
    progress_total = sum(progress_totals.values())
    progress_lock = threading.Lock()

    def emit_combined_progress(key: str, current: float, message: str) -> None:
        if progress_callback is None:
            return
        with progress_lock:
            progress_done[key] = min(progress_totals[key], max(0.0, float(current)))
            combined = sum(progress_done.values())
        _emit_progress(progress_callback, combined, progress_total, message)

    def encode_one(key: str, refs: Sequence[RawCaptureSampleRef]) -> tuple[str, str, Dict[str, Any]]:
        fps = float((fps_by_key or {}).get(key, default_fps))
        video_path = os.path.join(output_dir, f"{filename_prefix}{safe_name(key)}.mp4")
        codec = _encode_video_refs(
            reader,
            refs,
            video_path,
            fps,
            video_backend=video_backend,
            video_encoder=video_encoder,
            progress_callback=lambda current, _total, message: emit_combined_progress(key, current, message),
        )
        rel_path = os.path.basename(video_path)
        if rel_dir:
            rel_path = os.path.join(rel_dir, rel_path)
        return key, rel_path, codec

    videos: Dict[str, str] = {}
    codecs: Dict[str, Dict[str, Any]] = {}
    max_workers = min(len(items), max(1, os.cpu_count() or 1))
    with ThreadPoolExecutor(max_workers=max_workers, thread_name_prefix="raw-video-encode") as executor:
        futures = [executor.submit(encode_one, key, refs) for key, refs in items]
        for future in as_completed(futures):
            key, rel_path, codec = future.result()
            videos[key] = rel_path
            codecs[key] = codec
            emit_combined_progress(key, progress_totals[key], f"encoded {os.path.basename(rel_path)}")
    return videos, codecs


def encode_bgr_frames_to_h264(
    frames: Sequence[np.ndarray],
    output_path: str,
    fps: float,
    *,
    video_encoder: Optional[str] = None,
    progress_callback: Optional[Callable[[float, float, str], None]] = None,
) -> Dict[str, Any]:
    """Encode already-aligned BGR frames as H.264."""

    if not frames:
        return {"backend": "none", "encoder": "none", "hardware": False, "reason": "no frames"}
    selected = discover_ffmpeg_video_encoder(preferred=video_encoder or None, smoke_test=True)
    if selected.get("backend") != "ffmpeg":
        return _encode_bgr_frames_h264_fallback(
            frames,
            output_path,
            fps,
            reason=str(selected.get("reason") or "ffmpeg unavailable"),
            preferred_encoder=str(selected.get("encoder") or video_encoder or "") or None,
            progress_callback=progress_callback,
        )

    first = frames[0]
    height, width = first.shape[:2]
    cmd = [
        str(selected["ffmpeg"]),
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "bgr24",
        "-s",
        f"{width}x{height}",
        "-r",
        str(float(fps)),
        "-i",
        "pipe:0",
        "-an",
        "-c:v",
        str(selected["encoder"]),
        "-pix_fmt",
        str(selected.get("pix_fmt") or "yuv420p"),
        *list(selected.get("runtime_args") or ffmpeg_encoder_runtime_args(str(selected["encoder"]))),
        *list(selected.get("movflags") or ["-movflags", "+faststart"]),
        output_path,
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    assert proc.stdin is not None
    try:
        progress_pulse = _ProgressPulse(len(frames))
        for index, frame in enumerate(frames, start=1):
            if frame.shape[:2] != (height, width):
                frame = _resize_frame(frame, width, height)
            proc.stdin.write(np.ascontiguousarray(frame).tobytes())
            if progress_pulse.should_emit(index):
                _emit_progress(progress_callback, index, len(frames), f"encoding {os.path.basename(output_path)} {index}/{len(frames)}")
    except Exception:
        proc.kill()
        proc.wait(timeout=5)
        raise
    finally:
        try:
            proc.stdin.close()
        except Exception:
            pass
    _check_process(proc)
    return {
        "backend": "ffmpeg",
        "encoder": selected.get("encoder"),
        "hardware": bool(selected.get("hardware")),
        "platform": selected.get("platform"),
        "pix_fmt": selected.get("pix_fmt"),
        "runtime_args": selected.get("runtime_args") or [],
    }


def _encode_bgr_frames_h264_fallback(
    frames: Sequence[np.ndarray],
    output_path: str,
    fps: float,
    *,
    reason: str,
    preferred_encoder: Optional[str] = None,
    progress_callback: Optional[Callable[[float, float, str], None]] = None,
) -> Dict[str, Any]:
    if not frames:
        return {"backend": "none", "encoder": "none", "hardware": False, "reason": "no frames"}
    errors: list[str] = []
    if (preferred_encoder or "").strip().lower() != "opencv":
        try:
            return _encode_bgr_frames_pyav_h264(
                frames,
                output_path,
                fps,
                reason=reason,
                preferred_encoder=preferred_encoder,
                progress_callback=progress_callback,
            )
        except Exception as exc:
            errors.append(f"pyav: {exc}")
    opencv_reason = reason if not errors else f"{reason}; pyav fallback failed: {errors[-1]}"
    try:
        return _encode_bgr_frames_opencv_h264(
            frames,
            output_path,
            fps,
            reason=opencv_reason,
            progress_callback=progress_callback,
        )
    except Exception as exc:
        errors.append(f"opencv: {exc}")
    raise RuntimeError("no platform-compatible H.264 encoder available: " + "; ".join(errors))


def _encode_bgr_frames_pyav_h264(
    frames: Sequence[np.ndarray],
    output_path: str,
    fps: float,
    *,
    reason: str,
    preferred_encoder: Optional[str] = None,
    progress_callback: Optional[Callable[[float, float, str], None]] = None,
) -> Dict[str, Any]:
    try:
        import av
    except Exception as exc:
        raise RuntimeError("PyAV is not available") from exc

    first = frames[0]
    height, width = first.shape[:2]
    pix_fmt = "yuv420p"
    rate = Fraction(*_fps_fraction(fps))
    errors: list[str] = []
    for codec_name in _pyav_h264_codec_chain(preferred=preferred_encoder):
        if codec_name not in av.codecs_available:
            errors.append(f"{codec_name}: not available")
            continue
        try:
            with av.open(output_path, mode="w") as out:
                stream = out.add_stream(codec_name, rate=rate)
                stream.pix_fmt = pix_fmt
                stream.width = width
                stream.height = height
                progress_pulse = _ProgressPulse(len(frames))
                for index, frame in enumerate(frames, start=1):
                    if frame.shape[:2] != (height, width):
                        frame = _resize_frame(frame, width, height)
                    video_frame = av.VideoFrame.from_ndarray(np.ascontiguousarray(frame), format="bgr24")
                    for packet in stream.encode(video_frame):
                        out.mux(packet)
                    if progress_pulse.should_emit(index):
                        _emit_progress(progress_callback, index, len(frames), f"encoding {os.path.basename(output_path)} {index}/{len(frames)}")
                for packet in stream.encode():
                    out.mux(packet)
            return {
                "backend": "pyav",
                "encoder": codec_name,
                "hardware": codec_name != "libx264",
                "pix_fmt": pix_fmt,
                "reason": reason,
            }
        except Exception as exc:
            errors.append(f"{codec_name}: {exc}")
    raise RuntimeError("; ".join(errors) or "no PyAV H.264 codec available")


def _pyav_h264_codec_chain(preferred: Optional[str] = None) -> tuple[str, ...]:
    requested = (preferred or "").strip()
    if requested:
        return (requested,)
    system = platform.system()
    if system == "Windows":
        return ("h264_mf", "h264_nvenc", "libx264")
    if system == "Darwin":
        return ("h264_videotoolbox", "libx264")
    if system == "Linux":
        return ("h264_nvenc", "h264_vaapi", "libx264")
    return ("libx264",)


def _encode_bgr_frames_opencv_h264(
    frames: Sequence[np.ndarray],
    output_path: str,
    fps: float,
    *,
    reason: str,
    progress_callback: Optional[Callable[[float, float, str], None]] = None,
) -> Dict[str, Any]:
    try:
        import cv2
    except Exception as exc:
        raise RuntimeError("OpenCV is not available") from exc
    errors = []
    for fourcc_name in ("avc1", "H264"):
        try:
            _encode_bgr_frames_opencv_fourcc(frames, output_path, fps, fourcc_name, progress_callback=progress_callback)
            return {"backend": "opencv", "encoder": fourcc_name, "hardware": False, "reason": reason}
        except Exception as exc:
            errors.append(f"{fourcc_name}: {exc}")
    raise RuntimeError("no platform-compatible H.264 encoder available: " + "; ".join(errors))


def _encode_bgr_frames_opencv_fourcc(
    frames: Sequence[np.ndarray],
    output_path: str,
    fps: float,
    fourcc_name: str,
    *,
    progress_callback: Optional[Callable[[float, float, str], None]] = None,
) -> None:
    import cv2

    first = frames[0]
    height, width = first.shape[:2]
    writer = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*fourcc_name), float(fps), (width, height))
    if not writer.isOpened():
        raise RuntimeError("writer did not open")
    try:
        progress_pulse = _ProgressPulse(len(frames))
        for index, frame in enumerate(frames, start=1):
            if frame.shape[:2] != (height, width):
                frame = _resize_frame(frame, width, height)
            writer.write(np.ascontiguousarray(frame))
            if progress_pulse.should_emit(index):
                _emit_progress(progress_callback, index, len(frames), f"encoding {os.path.basename(output_path)} {index}/{len(frames)}")
    finally:
        writer.release()


def _infer_fps(samples: Sequence[RawCaptureSample | RawCaptureSampleRef], fallback: float) -> float:
    if len(samples) < 2:
        return float(fallback)
    duration = samples[-1].timestamp - samples[0].timestamp
    if duration <= 0:
        return float(fallback)
    return max(1.0, float(len(samples) - 1) / float(duration))


def _emit_progress(
    callback: Optional[Callable[[float, float, str], None]],
    current: float,
    total: float,
    message: str,
) -> None:
    if callback is None:
        return
    try:
        callback(current, total, message)
    except Exception:
        return


class _ProgressPulse:
    def __init__(self, total: int, interval_s: float = 1.0) -> None:
        self.total = int(total)
        self.interval_s = float(interval_s)
        self._last_emit = 0.0

    def should_emit(self, index: int) -> bool:
        if self.total <= 0:
            return False
        if index <= 1 or index >= self.total:
            self._last_emit = time.monotonic()
            return True
        now = time.monotonic()
        if now - self._last_emit >= self.interval_s:
            self._last_emit = now
            return True
        return False


@lru_cache(maxsize=16)
def _discover_video_encoder_cached(preferred: str) -> tuple[tuple[str, Any], ...]:
    selected = discover_video_encoder(preferred=preferred or None, smoke_test=True)
    return tuple(sorted(selected.items()))


def _select_video_encoder(preferred: Optional[str] = None) -> Dict[str, Any]:
    return dict(_discover_video_encoder_cached(preferred or ""))


def _encode_video_refs(
    reader: RawCaptureReader,
    refs: Sequence[RawCaptureSampleRef],
    output_path: str,
    fps: float,
    video_backend: str = "auto",
    video_encoder: Optional[str] = None,
    progress_callback: Optional[Callable[[float, float, str], None]] = None,
) -> Dict[str, Any]:
    selected = _select_video_encoder(video_encoder if video_backend != "opencv" else "opencv")
    if video_backend == "opencv":
        return _encode_video_refs_h264_fallback(
            reader,
            refs,
            output_path,
            fps,
            preferred_encoder="opencv",
            progress_callback=progress_callback,
        )
    if video_backend == "ffmpeg" and selected["backend"] != "ffmpeg":
        selected = discover_ffmpeg_video_encoder(preferred=video_encoder or None, smoke_test=True)
        if selected["backend"] != "ffmpeg":
            raise RuntimeError(f"ffmpeg video backend requested but unavailable: {selected.get('reason')}")
    if video_backend == "gstreamer" and selected["backend"] != "gstreamer":
        raise RuntimeError(f"gstreamer video backend requested but unavailable: {selected.get('reason')}")

    if selected["backend"] == "gstreamer":
        try:
            _encode_video_gst_refs(reader, refs, output_path, fps, selected, progress_callback=progress_callback)
            _emit_progress(progress_callback, len(refs), len(refs), f"encoded {os.path.basename(output_path)}")
            return {k: v for k, v in selected.items() if k != "gst_launch"}
        except Exception as exc:
            if video_backend == "gstreamer":
                raise
            selected = discover_ffmpeg_video_encoder(preferred=video_encoder or None, smoke_test=True)
            selected["reason"] = f"gstreamer fallback: {exc}"

    if selected["backend"] == "ffmpeg":
        try:
            _encode_video_ffmpeg_refs(reader, refs, output_path, fps, selected, progress_callback=progress_callback)
            _emit_progress(progress_callback, len(refs), len(refs), f"encoded {os.path.basename(output_path)}")
            return {k: v for k, v in selected.items() if k != "ffmpeg"}
        except Exception as exc:
            if video_backend == "ffmpeg":
                raise
            return _encode_video_refs_h264_fallback(
                reader,
                refs,
                output_path,
                fps,
                reason=f"ffmpeg fallback: {exc}",
                preferred_encoder=video_encoder,
                progress_callback=progress_callback,
            )

    return _encode_video_refs_h264_fallback(
        reader,
        refs,
        output_path,
        fps,
        reason=str(selected.get("reason") or "ffmpeg unavailable"),
        preferred_encoder=str(selected.get("encoder") or video_encoder or "") or None,
        progress_callback=progress_callback,
    )


def _encode_video_refs_h264_fallback(
    reader: RawCaptureReader,
    refs: Sequence[RawCaptureSampleRef],
    output_path: str,
    fps: float,
    *,
    reason: str = "requested",
    preferred_encoder: Optional[str] = None,
    progress_callback: Optional[Callable[[float, float, str], None]] = None,
) -> Dict[str, Any]:
    if not refs:
        raise RuntimeError("no samples to encode")
    with reader.open_stream_data(refs[0].key) as data_file:
        frames = [_image_to_bgr(reader.read_ref(ref, data_file=data_file)) for ref in refs]
    return _encode_bgr_frames_h264_fallback(
        frames,
        output_path,
        fps,
        reason=reason,
        preferred_encoder=preferred_encoder,
        progress_callback=progress_callback,
    )


def _encode_video_ffmpeg_refs(
    reader: RawCaptureReader,
    refs: Sequence[RawCaptureSampleRef],
    output_path: str,
    fps: float,
    selected: Dict[str, Any],
    progress_callback: Optional[Callable[[float, float, str], None]] = None,
) -> None:
    if not refs:
        raise RuntimeError("no samples to encode")
    input_args = _ffmpeg_ref_input_args(refs, fps)
    if input_args is None:
        _encode_video_ffmpeg_refs_bgr(reader, refs, output_path, fps, selected, progress_callback=progress_callback)
        return

    cmd = [
        str(selected["ffmpeg"]),
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        *input_args,
        "-an",
        "-c:v",
        str(selected["encoder"]),
        "-pix_fmt",
        "yuv420p",
        *ffmpeg_encoder_runtime_args(str(selected["encoder"])),
        "-movflags",
        "+faststart",
        output_path,
    ]
    _pipe_encoded_samples(reader, refs, cmd, progress_callback)


def _encode_video_ffmpeg_refs_bgr(
    reader: RawCaptureReader,
    refs: Sequence[RawCaptureSampleRef],
    output_path: str,
    fps: float,
    selected: Dict[str, Any],
    progress_callback: Optional[Callable[[float, float, str], None]] = None,
) -> None:
    with reader.open_stream_data(refs[0].key) as data_file:
        first = _image_to_bgr(reader.read_ref(refs[0], data_file=data_file))
        height, width = first.shape[:2]
        cmd = [
            str(selected["ffmpeg"]),
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-s",
            f"{width}x{height}",
            "-r",
            str(float(fps)),
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            str(selected["encoder"]),
            "-pix_fmt",
            "yuv420p",
            *ffmpeg_encoder_runtime_args(str(selected["encoder"])),
            "-movflags",
            "+faststart",
            output_path,
        ]
        _pipe_bgr_frames(reader, refs, data_file, first, width, height, cmd, progress_callback)


def _encode_video_gst_refs(
    reader: RawCaptureReader,
    refs: Sequence[RawCaptureSampleRef],
    output_path: str,
    fps: float,
    selected: Dict[str, Any],
    progress_callback: Optional[Callable[[float, float, str], None]] = None,
) -> None:
    if not refs:
        raise RuntimeError("no samples to encode")

    gst_launch = str(selected["gst_launch"])
    bitrate_bps = int(selected.get("bitrate_bps") or 4_000_000)
    jpeg_shape = _jpeg_ref_shape(refs)
    if jpeg_shape is not None:
        cmd = [gst_launch, "-q", "-e"] + _gst_jpeg_h264_pipeline_args(
            width=jpeg_shape[0],
            height=jpeg_shape[1],
            fps=float(fps),
            output_path=output_path,
            bitrate_bps=bitrate_bps,
        )
        _pipe_encoded_samples(reader, refs, cmd, progress_callback)
        return

    with reader.open_stream_data(refs[0].key) as data_file:
        first = _image_to_bgr(reader.read_ref(refs[0], data_file=data_file))
        height, width = first.shape[:2]
        cmd = [gst_launch, "-q", "-e"] + gst_h264_pipeline_args(
            width=width,
            height=height,
            fps=float(fps),
            output_path=output_path,
            bitrate_bps=bitrate_bps,
            input_format="bgr",
        )
        _pipe_bgr_frames(reader, refs, data_file, first, width, height, cmd, progress_callback)


def _pipe_encoded_samples(
    reader: RawCaptureReader,
    refs: Sequence[RawCaptureSampleRef],
    cmd: Sequence[str],
    progress_callback: Optional[Callable[[float, float, str], None]],
) -> None:
    proc = subprocess.Popen(list(cmd), stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    assert proc.stdin is not None
    try:
        progress_pulse = _ProgressPulse(len(refs))
        with reader.open_stream_data(refs[0].key) as data_file:
            for index, ref in enumerate(refs, start=1):
                proc.stdin.write(reader.read_ref(ref, data_file=data_file).data)
                if progress_pulse.should_emit(index):
                    _emit_progress(progress_callback, index, len(refs), f"encoding {index}/{len(refs)}")
    except Exception:
        proc.kill()
        proc.wait(timeout=5)
        raise
    finally:
        try:
            proc.stdin.close()
        except Exception:
            pass
    _check_process(proc)


def _pipe_bgr_frames(
    reader: RawCaptureReader,
    refs: Sequence[RawCaptureSampleRef],
    data_file: Any,
    first: np.ndarray,
    width: int,
    height: int,
    cmd: Sequence[str],
    progress_callback: Optional[Callable[[float, float, str], None]],
) -> None:
    proc = subprocess.Popen(list(cmd), stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    assert proc.stdin is not None
    try:
        progress_pulse = _ProgressPulse(len(refs))
        proc.stdin.write(first.tobytes())
        if progress_pulse.should_emit(1):
            _emit_progress(progress_callback, 1, len(refs), f"encoding 1/{len(refs)}")
        for index, ref in enumerate(refs[1:], start=2):
            frame = _image_to_bgr(reader.read_ref(ref, data_file=data_file))
            if frame.shape[:2] != (height, width):
                frame = _resize_frame(frame, width, height)
            proc.stdin.write(frame.tobytes())
            if progress_pulse.should_emit(index):
                _emit_progress(progress_callback, index, len(refs), f"encoding {index}/{len(refs)}")
    except Exception:
        proc.kill()
        proc.wait(timeout=5)
        raise
    finally:
        try:
            proc.stdin.close()
        except Exception:
            pass
    _check_process(proc)


def _check_process(proc: subprocess.Popen[bytes]) -> None:
    stderr = proc.stderr.read().decode("utf-8", errors="replace") if proc.stderr else ""
    return_code = proc.wait()
    if return_code != 0:
        raise RuntimeError(stderr.strip() or f"encoder exited with {return_code}")


def _resize_frame(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    try:
        import cv2
    except Exception as exc:
        raise RuntimeError("opencv-python is required for frame resizing") from exc
    return cv2.resize(frame, (width, height))


def _jpeg_ref_shape(refs: Sequence[RawCaptureSampleRef]) -> Optional[tuple[int, int]]:
    if not refs:
        return None
    first = refs[0]
    encoding = str(first.meta.get("encoding") or "").lower()
    if encoding not in ("jpg", "jpeg"):
        return None
    width = int(first.meta.get("width") or 0)
    height = int(first.meta.get("height") or 0)
    channels = int(first.meta.get("channels") or 3)
    if width <= 0 or height <= 0 or not _refs_have_same_shape(refs, encoding, width, height, channels):
        return None
    return width, height


def _gst_jpeg_h264_pipeline_args(
    *,
    width: int,
    height: int,
    fps: float,
    output_path: str,
    bitrate_bps: int,
) -> list[str]:
    fps_num, fps_den = _fps_fraction(fps)
    return [
        "fdsrc",
        "fd=0",
        "!",
        f"image/jpeg,width={int(width)},height={int(height)},framerate={fps_num}/{fps_den}",
        "!",
        "jpegparse",
        "!",
        "jpegdec",
        "!",
        "videoconvert",
        "!",
        "video/x-raw,format=NV12",
        "!",
        "mpph264enc",
        f"bps={int(bitrate_bps)}",
        "!",
        "h264parse",
        "!",
        "mp4mux",
        "faststart=true",
        "!",
        "filesink",
        f"location={output_path}",
    ]


def _ffmpeg_ref_input_args(refs: Sequence[RawCaptureSampleRef], fps: float) -> Optional[list[str]]:
    first = refs[0]
    encoding = str(first.meta.get("encoding") or "").lower()
    width = int(first.meta["width"])
    height = int(first.meta["height"])
    channels = int(first.meta.get("channels") or (1 if encoding in ("mono8", "gray8") else 3))
    if not _refs_have_same_shape(refs, encoding, width, height, channels):
        return None
    if encoding in ("jpg", "jpeg"):
        return ["-f", "image2pipe", "-vcodec", "mjpeg", "-r", str(float(fps)), "-i", "pipe:0"]
    if encoding == "png":
        return ["-f", "image2pipe", "-vcodec", "png", "-r", str(float(fps)), "-i", "pipe:0"]
    pix_fmt = {"bgr8": "bgr24", "rgb8": "rgb24", "mono8": "gray", "gray8": "gray"}.get(encoding)
    if pix_fmt is None:
        return None
    return [
        "-f",
        "rawvideo",
        "-pix_fmt",
        pix_fmt,
        "-s",
        f"{width}x{height}",
        "-r",
        str(float(fps)),
        "-i",
        "pipe:0",
    ]


def _refs_have_same_shape(
    refs: Sequence[RawCaptureSampleRef],
    encoding: str,
    width: int,
    height: int,
    channels: int,
) -> bool:
    for ref in refs:
        meta = ref.meta
        ref_encoding = str(meta.get("encoding") or "").lower()
        ref_channels = int(meta.get("channels") or (1 if ref_encoding in ("mono8", "gray8") else 3))
        if (
            ref_encoding != encoding
            or int(meta.get("width") or 0) != width
            or int(meta.get("height") or 0) != height
            or ref_channels != channels
        ):
            return False
    return True


def _fps_fraction(fps: float) -> tuple[int, int]:
    try:
        value = float(fps)
    except Exception:
        value = 30.0
    if not math.isfinite(value) or value <= 0:
        value = 30.0
    denominator = 1000
    numerator = max(1, int(round(value * denominator)))
    divisor = math.gcd(numerator, denominator)
    return numerator // divisor, denominator // divisor


def _image_to_bgr(sample: RawCaptureSample) -> np.ndarray:
    try:
        import cv2
    except Exception as exc:
        raise RuntimeError("opencv-python is required for image decoding") from exc
    encoding = str(sample.meta.get("encoding") or "").lower()
    width = int(sample.meta["width"])
    height = int(sample.meta["height"])
    channels = int(sample.meta.get("channels") or 3)
    if encoding in ("jpg", "jpeg", "png"):
        arr = np.frombuffer(sample.data, dtype=np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"failed to decode compressed image sample: {sample.key}")
        return image
    raw = np.frombuffer(sample.data, dtype=np.uint8).reshape((height, width, channels))
    if channels == 1:
        return cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
    if encoding == "rgb8":
        return cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
    if encoding == "bgr8":
        return raw
    raise ValueError(f"unsupported raw image encoding: {encoding}")
