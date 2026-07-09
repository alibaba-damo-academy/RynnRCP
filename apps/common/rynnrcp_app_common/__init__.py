"""Common helpers shared by RynnRCP apps."""

from .hardware_codec import (
    PlatformType,
    detect_platform,
    discover_ffmpeg_video_encoder,
    discover_video_encoder,
    log_video_encoder_self_check,
)
from .lifecycle import AppLifecycle

__all__ = [
    "AppLifecycle",
    "PlatformType",
    "detect_platform",
    "discover_ffmpeg_video_encoder",
    "discover_video_encoder",
    "log_video_encoder_self_check",
]
