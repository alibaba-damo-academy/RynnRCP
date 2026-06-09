"""
Hardware-accelerated video codec selection for PyAV.

Platform detection logic shared with encode_multi_platform.py (GStreamer path).
This module selects the best available PyAV/FFmpeg encoder for the current platform:

  - X86_NVIDIA  -> h264_nvenc   (NVENC hardware encoding)
  - MACOS       -> h264_videotoolbox (VideoToolbox hardware encoding)
  - JETSON      -> h264_nvenc   (Jetson NVENC)
  - LINUX_CPU   -> h264_vaapi / libx264 (VAAPI hardware or software fallback)
  - WINDOWS     -> h264_mf / libx264 (Media Foundation or software)
  - RK / RDK    -> libx264      (no PyAV hardware encoder for Rockchip/RDK)
  - UNKNOWN     -> libx264      (safe default)

Usage:
    from rcp_core.common.utils.hardware_codec import get_pyav_codec, detect_platform

    codec_name = get_pyav_codec()  # e.g. "h264_nvenc" or "libx264"
    with av.open(output_path, mode="w") as out:
        stream = out.add_stream(codec_name, rate=fps)
        stream.pix_fmt = "yuv420p"
        ...
"""

from __future__ import annotations

import os
import platform
import shutil
import subprocess
from enum import Enum
from typing import Optional, Tuple

from rcp_core.common.utils.logger import server_logger

logger = server_logger()


class PlatformType(Enum):
    JETSON = 1
    RK = 2
    RDK = 3
    X86_NVIDIA = 4
    MACOS = 5
    LINUX_CPU = 6
    WINDOWS = 7
    UNKNOWN = 8


# ---------------------------------------------------------------------------
# Platform detection helpers
# ---------------------------------------------------------------------------

def _run_cmd(cmd: str) -> str:
    """Run a shell command and return stdout, empty string on failure."""
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return result.stdout.strip()
    except Exception:
        return ""


def _has_nvidia_gpu() -> bool:
    """Check if an NVIDIA discrete GPU is present (non-Jetson)."""
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=5,
        )
        return result.returncode == 0 and len(result.stdout.strip()) > 0
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def _is_jetson() -> bool:
    if not os.path.exists("/sys/firmware/devicetree/base/model"):
        return False
    try:
        devtree = _run_cmd("cat /sys/firmware/devicetree/base/model")
        return "NVIDIA" in devtree or "Jetson" in devtree
    except Exception:
        return False


def _is_rockchip() -> bool:
    if not os.path.exists("/sys/firmware/devicetree/base/model"):
        return False
    try:
        devtree = _run_cmd("cat /sys/firmware/devicetree/base/model")
        return "Rockchip" in devtree or "RK" in devtree
    except Exception:
        return False


def _is_rdk() -> bool:
    if not os.path.exists("/sys/firmware/devicetree/base/model"):
        return False
    try:
        with open("/sys/firmware/devicetree/base/model", "r", encoding="utf-8") as f:
            tree = f.read()
        return "X5" in tree or "S100" in tree
    except Exception:
        return False


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


# ---------------------------------------------------------------------------
# PyAV codec selection
# ---------------------------------------------------------------------------

# Ordered fallback chains per platform.
# Each entry: (codec_name, pix_fmt, notes)
_CODEC_CHAINS: dict[PlatformType, list[Tuple[str, str, str]]] = {
    PlatformType.X86_NVIDIA: [
        ("h264_nvenc", "yuv420p", "NVIDIA NVENC hardware encoding"),
        ("libx264", "yuv420p", "software fallback"),
    ],
    PlatformType.JETSON: [
        ("h264_nvenc", "yuv420p", "Jetson NVENC hardware encoding"),
        ("libx264", "yuv420p", "software fallback"),
    ],
    PlatformType.MACOS: [
        ("h264_videotoolbox", "yuv420p", "macOS VideoToolbox hardware encoding"),
        ("libx264", "yuv420p", "software fallback"),
    ],
    PlatformType.WINDOWS: [
        ("h264_mf", "yuv420p", "Windows Media Foundation hardware encoding"),
        ("h264_nvenc", "yuv420p", "NVIDIA NVENC (if GPU present)"),
        ("libx264", "yuv420p", "software fallback"),
    ],
    PlatformType.LINUX_CPU: [
        ("h264_vaapi", "yuv420p", "VAAPI hardware encoding (Intel/AMD GPU)"),
        ("libx264", "yuv420p", "software fallback"),
    ],
    PlatformType.RK: [
        # Rockchip MPP is not available as a PyAV encoder; GStreamer mpph264enc is the HW path
        ("libx264", "yuv420p", "software encoding (use GStreamer for HW accel)"),
    ],
    PlatformType.RDK: [
        # RDK V4L2 is not available as a PyAV encoder; GStreamer v4l2h264enc is the HW path
        ("libx264", "yuv420p", "software encoding (use GStreamer for HW accel)"),
    ],
    PlatformType.UNKNOWN: [
        ("libx264", "yuv420p", "software fallback (unknown platform)"),
    ],
}

# Cache the result so we only probe once per process
_codec_cache: Optional[Tuple[str, str]] = None
_system_vaapi_probed = False
_system_vaapi_cache: Optional[Tuple[str, str]] = None


def _try_av_codec(codec_name: str) -> bool:
    """Test if a PyAV codec is available by attempting to probe it."""
    try:
        import av

        # av.codecs_available is a frozenset of all available codec names
        return codec_name in av.codecs_available
    except Exception:
        return False


def get_pyav_codec() -> Tuple[str, str]:
    """
    Get the best available PyAV video codec and pixel format for this platform.

    Returns:
        (codec_name, pix_fmt) tuple, e.g. ("h264_nvenc", "yuv420p")
    """
    global _codec_cache
    if _codec_cache is not None:
        return _codec_cache

    pt = detect_platform()
    chain = _CODEC_CHAINS.get(pt, _CODEC_CHAINS[PlatformType.UNKNOWN])

    for codec_name, pix_fmt, note in chain:
        if _try_av_codec(codec_name):
            logger.info(
                f"[HWCodec] platform={pt.name} codec={codec_name} pix_fmt={pix_fmt} ({note})"
            )
            _codec_cache = (codec_name, pix_fmt)
            return _codec_cache
        logger.debug(f"[HWCodec] codec {codec_name} not available, trying next ({note})")

    # Absolute fallback - libx264 is always present in PyAV
    logger.warning("[HWCodec] no preferred codec available, falling back to libx264")
    _codec_cache = ("libx264", "yuv420p")
    return _codec_cache


def get_video_encoder_mode() -> str:
    """Return requested video encoder mode: auto, pyav, ffmpeg_vaapi, or software."""
    mode = os.environ.get("RYNNRCP_VIDEO_ENCODER", "auto").strip().lower()
    if mode not in {"auto", "pyav", "ffmpeg_vaapi", "software"}:
        logger.warning(f"[HWCodec] unknown RYNNRCP_VIDEO_ENCODER={mode!r}, using auto")
        return "auto"
    return mode


def _ffmpeg_has_encoder(codec_name: str) -> bool:
    if shutil.which("ffmpeg") is None:
        return False
    try:
        result = subprocess.run(
            ["ffmpeg", "-hide_banner", "-encoders"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=10,
        )
        return result.returncode == 0 and codec_name in result.stdout
    except Exception:
        return False


def get_system_vaapi_encoder() -> Optional[Tuple[str, str]]:
    """
    Return (device_path, codec_name) when system FFmpeg VAAPI H.264 is usable.

    This covers devices where the system ffmpeg supports h264_vaapi but the PyAV
    wheel does not expose that encoder.
    """
    global _system_vaapi_probed, _system_vaapi_cache
    mode = get_video_encoder_mode()
    if mode in {"pyav", "software"}:
        return None
    if _system_vaapi_probed:
        return _system_vaapi_cache

    _system_vaapi_probed = True
    device_path = os.environ.get("RYNNRCP_VAAPI_DEVICE", "/dev/dri/renderD128").strip()
    if detect_platform() != PlatformType.LINUX_CPU and mode != "ffmpeg_vaapi":
        return None
    if not device_path or not os.path.exists(device_path):
        logger.debug(f"[HWCodec] VAAPI device not found: {device_path}")
        return None
    if not _ffmpeg_has_encoder("h264_vaapi"):
        logger.debug("[HWCodec] system ffmpeg h264_vaapi encoder not available")
        return None

    _system_vaapi_cache = (device_path, "h264_vaapi")
    logger.info(f"[HWCodec] system ffmpeg VAAPI available device={device_path} codec=h264_vaapi")
    return _system_vaapi_cache


def reset_codec_cache() -> None:
    """Reset cached codec selections (useful for testing)."""
    global _codec_cache, _system_vaapi_probed, _system_vaapi_cache
    _codec_cache = None
    _system_vaapi_probed = False
    _system_vaapi_cache = None
