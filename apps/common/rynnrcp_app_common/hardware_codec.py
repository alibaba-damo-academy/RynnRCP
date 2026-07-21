"""Hardware video encoder discovery.

This module separates platform detection from the concrete video writer.
Dataset encoders write video through ffmpeg or GStreamer subprocesses, so
encoder names are validated against the selected backend, not only inferred
from CPU architecture.
"""

from __future__ import annotations

import os
import platform
import shutil
import subprocess
import tempfile
from enum import Enum
import logging
from typing import Any, Dict, List, Optional, Tuple


class PlatformType(Enum):
    JETSON = "jetson"
    RK = "rk"
    RDK = "rdk"
    X86_NVIDIA = "x86_nvidia"
    MACOS = "macos"
    LINUX_CPU = "linux_cpu"
    WINDOWS = "windows"
    UNKNOWN = "unknown"


FFmpegEncoder = Dict[str, Any]


# Pixel format and container options applied to every ffmpeg encode invocation.
FFMPEG_COMMON_OUTPUT_ARGS: List[str] = ["-pix_fmt", "yuv420p"]
FFMPEG_MOVFLAGS: List[str] = ["-movflags", "+faststart"]


def ffmpeg_encoder_runtime_args(encoder: str) -> List[str]:
    """Return the rate-control / quality args used at real encoding time.

    Keeping this in a single place ensures that the startup self-check log and
    the actual encode subprocess always agree on what parameters are used.
    """

    if encoder == "libx264":
        return ["-preset", "veryfast", "-crf", "23"]
    if _is_hardware_encoder(encoder):
        return ["-b:v", "4M"]
    return []


# Default H.264 bitrate used by the GStreamer hardware encoder path. Mirrors
# the 4 Mbps target used for ffmpeg hardware encoders.
GST_DEFAULT_BITRATE_BPS: int = 4_000_000


def gst_h264_pipeline_args(
    *,
    width: int,
    height: int,
    fps: float,
    output_path: str,
    bitrate_bps: int = GST_DEFAULT_BITRATE_BPS,
    input_format: str = "bgr",
) -> List[str]:
    """Build the gst-launch-1.0 argv that encodes raw frames piped on stdin.

    The pipeline ingests raw video from fd 0, hands NV12 frames to
    ``mpph264enc`` (Rockchip MPP hardware H.264 encoder), then muxes the result
    into a faststart MP4. The caller is responsible for writing the raw frames
    to the subprocess stdin and closing it (so the pipeline reaches EOS and
    mp4mux can finalize the moov atom).
    """

    fps_num, fps_den = _fps_to_fraction(fps)
    return [
        "fdsrc",
        "fd=0",
        "!",
        "rawvideoparse",
        f"format={input_format}",
        f"width={int(width)}",
        f"height={int(height)}",
        f"framerate={fps_num}/{fps_den}",
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


def _fps_to_fraction(fps: float) -> Tuple[int, int]:
    """Approximate ``fps`` as a small integer fraction for GStreamer caps."""

    try:
        f = float(fps)
    except Exception:
        return 30, 1
    if f <= 0:
        return 30, 1
    # Round to milli-fps; e.g. 29.97 -> 29970/1000 -> reduced to 2997/100.
    num = int(round(f * 1000))
    den = 1000
    from math import gcd

    g = gcd(num, den) or 1
    return max(1, num // g), max(1, den // g)


def detect_platform() -> PlatformType:
    """Detect the current platform type for video encoding decisions."""

    system = platform.system()
    if system == "Darwin":
        return PlatformType.MACOS
    if system == "Windows":
        if _has_nvidia_gpu():
            return PlatformType.X86_NVIDIA
        return PlatformType.WINDOWS
    if system == "Linux":
        if _is_jetson():
            return PlatformType.JETSON
        if _is_rockchip():
            return PlatformType.RK
        if _is_rdk():
            return PlatformType.RDK
        if _has_nvidia_gpu():
            return PlatformType.X86_NVIDIA
        return PlatformType.LINUX_CPU
    return PlatformType.UNKNOWN


def discover_video_encoder(
    preferred: Optional[str] = None,
    smoke_test: bool = True,
) -> FFmpegEncoder:
    """Top-level video encoder discovery.

    On RK platforms we first probe the GStreamer ``mpph264enc`` (Rockchip MPP)
    hardware path, because the system ffmpeg 4.x ships with rkmpp **decode-only**
    and ``h264_v4l2m2m`` does not bind to the RK encoder hardware. If the
    GStreamer path is not available, we fall back to ffmpeg subprocess
    discovery, and finally to the PyAV/OpenCV H.264 fallback writer.
    """

    requested = (preferred or "").strip()
    requested_lower = requested.lower()

    # Explicit caller overrides bypass auto detection.
    if requested_lower == "opencv":
        return _opencv_result("requested")
    if requested_lower in ("gstreamer", "mpph264enc", "gstreamer:mpph264enc"):
        gst = discover_gstreamer_video_encoder(smoke_test=smoke_test, reason="requested")
        if gst is not None:
            return gst
        return _opencv_result("gstreamer mpph264enc not usable")

    # Auto path: prefer GStreamer hardware on RK; otherwise go straight to ffmpeg.
    if not requested and detect_platform() == PlatformType.RK:
        gst = discover_gstreamer_video_encoder(smoke_test=smoke_test, reason="auto")
        if gst is not None:
            return gst

    return discover_ffmpeg_video_encoder(preferred=preferred, smoke_test=smoke_test)


def discover_gstreamer_video_encoder(
    smoke_test: bool = True,
    reason: str = "auto",
) -> Optional[FFmpegEncoder]:
    """Return a backend=gstreamer descriptor if mpph264enc is usable, else None.

    The descriptor mirrors the shape of :func:`discover_ffmpeg_video_encoder`
    so the recording pipeline can switch on ``backend`` uniformly.
    """

    gst_launch = shutil.which("gst-launch-1.0")
    gst_inspect = shutil.which("gst-inspect-1.0")
    if gst_launch is None or gst_inspect is None:
        return None
    if not _gst_has_element(gst_inspect, "mpph264enc"):
        return None
    if smoke_test:
        ok, smoke_reason = _smoke_test_gst_mpph264enc(gst_launch)
        if not ok:
            return None
    else:
        smoke_reason = "skipped"

    return {
        "backend": "gstreamer",
        "encoder": "mpph264enc",
        "hardware": True,
        "platform": detect_platform().value,
        "reason": reason,
        "gst_launch": gst_launch,
        "bitrate_bps": GST_DEFAULT_BITRATE_BPS,
        "input_format": "bgr",
        "pix_fmt": "NV12 (internal)",
        "smoke_test_reason": smoke_reason,
        "failed_candidates": [],
    }


def _gst_has_element(gst_inspect: str, element: str) -> bool:
    try:
        proc = subprocess.run(
            [gst_inspect, element],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5,
        )
    except Exception:
        return False
    return proc.returncode == 0


def _smoke_test_gst_mpph264enc(gst_launch: str) -> Tuple[bool, str]:
    """Run a tiny videotestsrc -> mpph264enc -> fakesink pipeline.

    We use ``videotestsrc`` rather than fdsrc to avoid coupling the smoke test
    with raw-frame plumbing — this isolates the question "is the Rockchip MPP
    encoder actually wired up to the kernel?" from any pipeline construction
    issues at the real encode path.
    """

    cmd = [
        gst_launch,
        "-q",
        "videotestsrc",
        "num-buffers=5",
        "!",
        "video/x-raw,format=NV12,width=320,height=240,framerate=30/1",
        "!",
        "mpph264enc",
        "!",
        "fakesink",
    ]
    try:
        proc = subprocess.run(
            cmd,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=8,
        )
    except Exception as exc:
        return False, str(exc)
    if proc.returncode != 0:
        return False, proc.stderr.decode("utf-8", errors="replace").strip() or f"gst-launch exited with {proc.returncode}"
    return True, "OK"


def discover_ffmpeg_video_encoder(preferred: Optional[str] = None, smoke_test: bool = True) -> FFmpegEncoder:
    """Return the first usable ffmpeg encoder for this machine.

    Discovery is intentionally two-stage:
    1. ``ffmpeg -encoders`` is used to list candidates compiled into ffmpeg.
    2. A tiny rawvideo encode is attempted so drivers, permissions, pixel
       formats, and container support are tested before the encoder is trusted.
    """

    requested = (preferred or "").strip()
    if requested.lower() == "opencv":
        return _opencv_result("requested")

    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        return _h264_fallback_result("ffmpeg not found", preferred=requested or None)

    encoders_text = _ffmpeg_encoders(ffmpeg)
    if requested:
        candidates = [requested] if requested in encoders_text else []
        missing_reason = f"requested encoder not found: {requested}"
    else:
        candidates = _ffmpeg_encoder_chain(detect_platform())
        missing_reason = "no supported ffmpeg encoder found"

    failed: List[Dict[str, str]] = []
    for encoder in candidates:
        if encoder not in encoders_text:
            failed.append({"encoder": encoder, "reason": "not listed by ffmpeg"})
            continue
        if smoke_test:
            ok, reason = _smoke_test_ffmpeg_encoder(ffmpeg, encoder)
            if not ok:
                failed.append({"encoder": encoder, "reason": reason})
                continue
        return {
            "backend": "ffmpeg",
            "encoder": encoder,
            "hardware": _is_hardware_encoder(encoder),
            "ffmpeg": ffmpeg,
            "platform": detect_platform().value,
            "reason": "requested" if requested else "auto",
            "failed_candidates": failed,
            "pix_fmt": "yuv420p",
            "runtime_args": ffmpeg_encoder_runtime_args(encoder),
            "movflags": list(FFMPEG_MOVFLAGS),
        }

    fallback = _h264_fallback_result(missing_reason, preferred=requested or None)
    fallback["failed_candidates"] = failed
    return fallback


def log_video_encoder_self_check(
    logger: logging.Logger,
    preferred: Optional[str] = None,
    smoke_test: bool = True,
    prefix: str = "VideoEncoder",
) -> FFmpegEncoder:
    """Discover and log the video encoder selected for this process."""

    result = discover_video_encoder(preferred=preferred, smoke_test=smoke_test)
    backend = result.get("backend")
    encoder = result.get("encoder")
    platform_name = result.get("platform")
    hardware = result.get("hardware")
    reason = result.get("reason")
    if backend == "gstreamer":
        logger.info(
            "[%s] self-check: platform=%s backend=gstreamer encoder=%s hardware=%s gst_launch=%s reason=%s",
            prefix,
            platform_name,
            encoder,
            hardware,
            result.get("gst_launch"),
            reason,
        )
        logger.info(
            "[%s] encode params: pipeline=fdsrc!rawvideoparse(format=%s)!videoconvert!NV12!mpph264enc(bps=%s)!h264parse!mp4mux(faststart) hardware_accelerated=%s",
            prefix,
            result.get("input_format"),
            result.get("bitrate_bps"),
            hardware,
        )
    elif backend == "ffmpeg":
        runtime_args = result.get("runtime_args") or []
        params_text = " ".join(runtime_args) if runtime_args else "(encoder defaults)"
        logger.info(
            "[%s] self-check: platform=%s backend=ffmpeg encoder=%s hardware=%s ffmpeg=%s reason=%s",
            prefix,
            platform_name,
            encoder,
            hardware,
            result.get("ffmpeg"),
            reason,
        )
        logger.info(
            "[%s] encode params: pix_fmt=%s movflags=%s rate_control=[%s] hardware_accelerated=%s",
            prefix,
            result.get("pix_fmt"),
            " ".join(result.get("movflags") or []),
            params_text,
            hardware,
        )
    elif backend == "pyav":
        logger.warning(
            "[%s] self-check: platform=%s no usable ffmpeg encoder, fallback backend=pyav encoder=%s hardware=%s reason=%s",
            prefix,
            platform_name,
            encoder,
            hardware,
            reason,
        )
        logger.warning(
            "[%s] encode params: backend=pyav pix_fmt=%s hardware_accelerated=%s",
            prefix,
            result.get("pix_fmt"),
            hardware,
        )
    else:
        logger.warning(
            "[%s] self-check: platform=%s no usable hardware encoder, fallback backend=%s encoder=%s reason=%s",
            prefix,
            platform_name,
            backend,
            encoder,
            reason,
        )
        logger.warning(
            "[%s] encode params: backend=opencv fourcc=%s hardware_accelerated=False",
            prefix,
            encoder,
        )

    for failed in result.get("failed_candidates", [])[:4]:
        logger.info(
            "[%s] candidate failed: encoder=%s reason=%s",
            prefix,
            failed.get("encoder"),
            _compact_reason(str(failed.get("reason") or "")),
        )

    # Only surface fix-it advice when we did NOT already pick a hardware backend.
    if not hardware:
        hint = _diagnose_hardware_encoder_access(
            platform_type=detect_platform(),
            failed_candidates=result.get("failed_candidates") or [],
        )
        if hint is not None:
            logger.warning("[%s] %s", prefix, hint["headline"])
            for line in hint["actions"]:
                logger.warning("[%s]   %s", prefix, line)
    return result


def _ffmpeg_encoder_chain(platform_type: PlatformType) -> List[str]:
    chains = {
        PlatformType.X86_NVIDIA: ["h264_nvenc", "hevc_nvenc", "libx264"],
        PlatformType.JETSON: ["h264_nvenc", "hevc_nvenc", "libx264"],
        PlatformType.MACOS: ["h264_videotoolbox", "hevc_videotoolbox", "libx264"],
        PlatformType.WINDOWS: ["h264_mf", "h264_nvenc", "libx264"],
        PlatformType.LINUX_CPU: ["h264_vaapi", "h264_qsv", "libx264"],
        # NOTE: On Rockchip, ffmpeg 4.x ships rkmpp as decode-only and
        # h264_v4l2m2m does not bind to the RK encoder. The reliable HW path
        # is the GStreamer mpph264enc backend, which is selected before the
        # ffmpeg chain runs (see ``discover_video_encoder``). Keep libx264 as
        # the ffmpeg-tier fallback when GStreamer is unavailable.
        PlatformType.RK: ["libx264"],
        PlatformType.RDK: ["h264_v4l2m2m", "libx264"],
        PlatformType.UNKNOWN: ["libx264"],
    }
    return chains.get(platform_type, chains[PlatformType.UNKNOWN])


def _ffmpeg_encoders(ffmpeg: str) -> str:
    try:
        proc = subprocess.run(
            [ffmpeg, "-hide_banner", "-encoders"],
            check=False,
            text=True,
            capture_output=True,
            timeout=5,
        )
        return proc.stdout + proc.stderr
    except Exception:
        return ""


def _smoke_test_ffmpeg_encoder(ffmpeg: str, encoder: str) -> Tuple[bool, str]:
    with tempfile.TemporaryDirectory() as td:
        out_path = os.path.join(td, "smoke.mp4")
        # Use a common video size. Some hardware encoders reject tiny frames
        # such as 16x16 even though they work for real camera resolutions.
        width, height, fps, frames = 320, 240, 5, 3
        raw_frame = bytes(width * height * 3)
        cmd = [
            ffmpeg,
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
            str(fps),
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            encoder,
            "-pix_fmt",
            "yuv420p",
        ]
        if encoder == "libx264":
            cmd.extend(["-preset", "ultrafast", "-crf", "30"])
        elif _is_hardware_encoder(encoder):
            cmd.extend(["-b:v", "1M"])
        cmd.append(out_path)
        try:
            proc = subprocess.run(
                cmd,
                input=raw_frame * frames,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=8,
            )
        except Exception as exc:
            return False, str(exc)
        if proc.returncode != 0:
            return False, proc.stderr.decode("utf-8", errors="replace").strip()
        if not os.path.exists(out_path) or os.path.getsize(out_path) <= 0:
            return False, "encoder produced no output"
        return True, "OK"


def _has_nvidia_gpu() -> bool:
    try:
        proc = subprocess.run(
            ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=5,
        )
        return proc.returncode == 0 and bool(proc.stdout.strip())
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def _is_jetson() -> bool:
    model = _read_devtree_model()
    return "NVIDIA" in model or "Jetson" in model


def _is_rockchip() -> bool:
    model = _read_devtree_model()
    return "Rockchip" in model or "RK" in model


def _is_rdk() -> bool:
    model = _read_devtree_model()
    return "X5" in model or "S100" in model


def _read_devtree_model() -> str:
    path = "/sys/firmware/devicetree/base/model"
    if not os.path.exists(path):
        return ""
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read()
    except Exception:
        return ""


def _is_hardware_encoder(encoder: str) -> bool:
    return any(token in encoder for token in ("videotoolbox", "nvenc", "qsv", "vaapi", "amf", "mf", "rkmpp", "v4l2m2m"))


# Substrings in ffmpeg stderr that strongly suggest the encoder is present but
# the current user does not have permission to use the underlying device node.
_PERMISSION_HINT_TOKENS = (
    "permission denied",
    "could not find a valid device",
    "no such device",
    "no such file or directory",
    "operation not permitted",
)


def _diagnose_hardware_encoder_access(
    *,
    platform_type: PlatformType,
    failed_candidates: List[Dict[str, str]],
) -> Optional[Dict[str, Any]]:
    """Return a copy-pastable hint when the user *could* enable HW encoding.

    Returns ``{"headline": str, "actions": [str, ...]}`` or None when no
    actionable fix is detected.
    """

    if platform_type == PlatformType.RK:
        return _diagnose_rk_hardware_encoder()

    return _diagnose_v4l2_group_block(failed_candidates)


def _diagnose_rk_hardware_encoder() -> Optional[Dict[str, Any]]:
    """RK-specific advice. The reliable HW path on RK is GStreamer mpph264enc.

    If the Rockchip GStreamer plugin is not installed, recommend installing it.
    If it *is* installed but smoke test still failed, recommend checking
    /dev/mpp_service permissions.
    """

    gst_inspect = shutil.which("gst-inspect-1.0")
    gst_has_mpp = bool(gst_inspect and _gst_has_element(gst_inspect, "mpph264enc"))
    if not gst_has_mpp:
        return {
            "headline": (
                "RK3566 hardware H.264 encoder (Rockchip MPP) is not enabled "
                "because the GStreamer Rockchip plugin is missing. Install it:"
            ),
            "actions": [
                "sudo apt update",
                "sudo apt install -y gstreamer1.0-rockchip1 gstreamer1.0-plugins-base gstreamer1.0-plugins-good librockchip-mpp1",
                "(no reboot needed; just restart this service)",
            ],
        }

    # mpph264enc is registered but the smoke test failed. Most common cause:
    # /dev/mpp_service is not accessible to the current user.
    mpp_dev = "/dev/mpp_service"
    if os.path.exists(mpp_dev):
        try:
            st = os.stat(mpp_dev)
        except Exception:
            st = None
        if st is not None:
            mode = st.st_mode & 0o777
            if mode & 0o006:  # world-rw, the standard RK setup
                return {
                    "headline": (
                        "Rockchip GStreamer plugin is installed but the hardware "
                        "encoder smoke test failed. Check kernel/driver state:"
                    ),
                    "actions": [
                        "ls -l /dev/mpp_service /dev/rga",
                        "dmesg | grep -iE 'mpp|rkvenc' | tail",
                        "gst-launch-1.0 videotestsrc num-buffers=10 ! video/x-raw,format=NV12,width=320,height=240 ! mpph264enc ! fakesink",
                    ],
                }
            try:
                import grp
                group_name = grp.getgrgid(st.st_gid).gr_name
            except Exception:
                group_name = str(st.st_gid)
            try:
                import pwd
                user_name = pwd.getpwuid(os.getuid()).pw_name
            except Exception:
                user_name = str(os.getuid())
            return {
                "headline": (
                    f"RK hardware encoder is present but {mpp_dev} is not accessible. "
                    f"Add user '{user_name}' to group '{group_name}':"
                ),
                "actions": [
                    f"sudo usermod -aG {group_name} {user_name}",
                    f"(then logout/login, or run `newgrp {group_name}` in this shell, then restart this service)",
                ],
            }

    return {
        "headline": "RK hardware encoder smoke test failed; /dev/mpp_service is missing.",
        "actions": [
            "ls -l /dev/mpp_service /dev/rga",
            "Check that the Rockchip vendor kernel is booted (uname -a should mention 'rockchip').",
            "Verify devicetree exposes the encoder: grep -i rkvenc /proc/device-tree/* 2>/dev/null",
        ],
    }


def _diagnose_v4l2_group_block(
    failed_candidates: List[Dict[str, str]],
) -> Optional[Dict[str, Any]]:
    """Generic Linux V4L2 device-node group blocker (non-RK boards)."""

    looks_like_hw_block = any(
        ("v4l2m2m" in (c.get("encoder") or "") or "rkmpp" in (c.get("encoder") or ""))
        and any(tok in str(c.get("reason") or "").lower() for tok in _PERMISSION_HINT_TOKENS)
        for c in failed_candidates
    )
    if not looks_like_hw_block:
        return None

    try:
        import grp
        import pwd
    except Exception:
        return None

    devices: List[str] = []
    try:
        for name in sorted(os.listdir("/dev")):
            if name.startswith("video-enc") or name.startswith("video-dec"):
                full = "/dev/" + name
                try:
                    if stat_is_chr(full):
                        devices.append(full)
                except Exception:
                    pass
    except Exception:
        pass
    if not devices:
        try:
            for name in sorted(os.listdir("/dev")):
                if name.startswith("video"):
                    full = "/dev/" + name
                    try:
                        if stat_is_chr(full):
                            devices.append(full)
                    except Exception:
                        pass
        except Exception:
            pass

    user_groups = set(os.getgroups())
    try:
        user_name = pwd.getpwuid(os.getuid()).pw_name
    except KeyError:
        user_name = str(os.getuid())

    for path in devices:
        try:
            st = os.stat(path)
        except Exception:
            continue
        if st.st_gid in user_groups:
            continue
        try:
            group_name = grp.getgrgid(st.st_gid).gr_name
        except KeyError:
            group_name = str(st.st_gid)
        return {
            "headline": (
                f"Hardware encoder is present but {path} belongs to group "
                f"'{group_name}', current user '{user_name}' is NOT in it. Run:"
            ),
            "actions": [
                f"sudo usermod -aG {group_name} {user_name}",
                f"(then logout/login, or run `newgrp {group_name}` in this shell, then restart this service)",
            ],
        }
    return None


def stat_is_chr(path: str) -> bool:
    """Return True only when ``path`` is a real character device node."""

    import stat as _stat

    try:
        return _stat.S_ISCHR(os.stat(path).st_mode)
    except Exception:
        return False


def _compact_reason(reason: str, max_len: int = 220) -> str:
    text = " ".join(reason.split())
    if len(text) <= max_len:
        return text
    return text[: max_len - 3] + "..."


def _h264_fallback_result(reason: str, preferred: Optional[str] = None) -> FFmpegEncoder:
    pyav = _pyav_result(reason, preferred=preferred)
    if pyav is not None:
        return pyav
    return _opencv_result(reason)


def _pyav_result(reason: str, preferred: Optional[str] = None) -> Optional[FFmpegEncoder]:
    try:
        import av
    except Exception:
        return None
    failed: List[Dict[str, str]] = []
    for encoder in _pyav_h264_codec_chain(detect_platform(), preferred=preferred):
        if encoder in av.codecs_available:
            return {
                "backend": "pyav",
                "encoder": encoder,
                "hardware": encoder != "libx264",
                "platform": detect_platform().value,
                "reason": reason,
                "failed_candidates": failed,
                "pix_fmt": "yuv420p",
            }
        failed.append({"encoder": encoder, "reason": "not listed by PyAV"})
    return None


def _pyav_h264_codec_chain(platform_type: PlatformType, preferred: Optional[str] = None) -> List[str]:
    requested = (preferred or "").strip()
    if requested:
        return [requested]
    if platform_type == PlatformType.WINDOWS:
        return ["h264_mf", "h264_nvenc", "libx264"]
    if platform_type == PlatformType.MACOS:
        return ["h264_videotoolbox", "libx264"]
    if platform_type in (PlatformType.X86_NVIDIA, PlatformType.JETSON):
        return ["h264_nvenc", "libx264"]
    if platform_type == PlatformType.LINUX_CPU:
        return ["h264_vaapi", "libx264"]
    return ["libx264"]


def _opencv_result(reason: str) -> FFmpegEncoder:
    return {
        "backend": "opencv",
        "encoder": "avc1/H264",
        "hardware": False,
        "platform": detect_platform().value,
        "reason": reason,
    }
