"""Subprocess entry point for raw capture encoding."""

from __future__ import annotations

from .recording import encode_raw_capture


def encode_from_config(config: dict) -> dict:
    """Encode a raw capture from a structured config dict.

    This is the importable callable used by ``rynnrcp.process.run_python_function_task``.
    It keeps the production worker path inside Python's multiprocessing APIs
    instead of passing results through stdout.
    """

    return encode_raw_capture(
        capture_dir=config["capture_dir"],
        output_dir=config.get("output_dir"),
        keys=config.get("keys"),
        fps=config.get("fps"),
        include_original_videos=bool(config.get("include_original_videos", False)),
        key_mapping=config.get("key_mapping"),
        video_backend=config.get("video_backend", "auto"),
        video_encoder=config.get("video_encoder"),
    )
