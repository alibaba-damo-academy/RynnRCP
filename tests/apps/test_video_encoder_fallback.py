"""Video encoder fallback regression tests."""

from __future__ import annotations

import sys
from types import SimpleNamespace

from rynnrcp_app_common import hardware_codec, video_writers


def test_explicit_encoder_is_preserved_when_ffmpeg_is_unavailable(monkeypatch) -> None:
    monkeypatch.setattr(hardware_codec.shutil, "which", lambda _name: None)
    monkeypatch.setattr(hardware_codec, "detect_platform", lambda: hardware_codec.PlatformType.X86_NVIDIA)
    monkeypatch.setitem(
        sys.modules,
        "av",
        SimpleNamespace(codecs_available={"h264_nvenc", "libx264"}),
    )

    selected = hardware_codec.discover_video_encoder(preferred="libx264", smoke_test=False)

    assert selected["backend"] == "pyav"
    assert selected["encoder"] == "libx264"
    assert selected["hardware"] is False


def test_pyav_codec_chain_keeps_explicit_encoder_first_and_deduplicates(monkeypatch) -> None:
    monkeypatch.setattr(video_writers.platform, "system", lambda: "Darwin")

    assert video_writers._pyav_h264_codec_chain(preferred="libx264") == (
        "libx264",
        "h264_videotoolbox",
    )


def test_opencv_preference_skips_pyav(monkeypatch) -> None:
    monkeypatch.setattr(
        video_writers,
        "_encode_bgr_frames_pyav_h264",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("PyAV should not be used")),
    )
    monkeypatch.setattr(
        video_writers,
        "_encode_bgr_frames_opencv_h264",
        lambda *_args, **_kwargs: {"backend": "opencv"},
    )

    result = video_writers._encode_bgr_frames_h264_fallback(
        [object()],
        "output.mp4",
        30.0,
        reason="requested",
        preferred_encoder="opencv",
    )

    assert result == {"backend": "opencv"}


def test_selected_pyav_encoder_reaches_video_writer(monkeypatch) -> None:
    seen = {}
    monkeypatch.setattr(
        video_writers,
        "_select_video_encoder",
        lambda _preferred: {
            "backend": "pyav",
            "encoder": "libx264",
            "hardware": False,
            "reason": "ffmpeg not found",
        },
    )

    def fake_fallback(_reader, _refs, _output_path, _fps, **kwargs):
        seen.update(kwargs)
        return {"backend": "pyav", "encoder": kwargs["preferred_encoder"]}

    monkeypatch.setattr(video_writers, "_encode_video_refs_h264_fallback", fake_fallback)

    result = video_writers._encode_video_refs(
        object(),
        [object()],
        "output.mp4",
        30.0,
        video_backend="auto",
        video_encoder="libx264",
    )

    assert seen["preferred_encoder"] == "libx264"
    assert result == {"backend": "pyav", "encoder": "libx264"}
