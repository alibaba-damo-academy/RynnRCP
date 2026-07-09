"""RynnBot capture upload regression tests."""

from __future__ import annotations

import sys
import types
from typing import Any, Dict

from rynnrcp_app_common import recording, recording_package
from rynnrcp_app_rynnbot.capture_manager import RynnBotCaptureManager


def _manager(tmp_path, post_upload_event=None) -> RynnBotCaptureManager:
    return RynnBotCaptureManager(
        config={"max_capture_duration_s": 600, "video_backend": "auto", "video_encoder": None},
        raw_capture_root=tmp_path,
        server_client_provider=lambda: None,
        mqtt_send=lambda _topic, _payload, qos=0: None,
        parse_json_or_reply_error=lambda _payload, _send_topic, _kind: {},
        make_base_response=lambda _request: {"result": {}},
        start_ws_state_stream=lambda: None,
        stop_ws_state_stream=lambda: None,
        post_upload_event=post_upload_event or (lambda _event, _params, qos=0: None),
        post_occupancy_error=lambda **_kwargs: None,
        occupied_id_provider=lambda: None,
        clear_occupancy=lambda: None,
        executor=None,
    )


def test_encode_capture_upload_job_imports_package_helper(tmp_path, monkeypatch) -> None:
    manager = _manager(tmp_path)
    seen: Dict[str, Any] = {}

    def fake_encode_and_upload(kind, params, encode_raw_capture_subprocess, package_encoded_captures):
        seen.update(
            {
                "kind": kind,
                "params": params,
                "encoder": encode_raw_capture_subprocess,
                "packager": package_encoded_captures,
            }
        )
        return {"ok": True}

    monkeypatch.setattr(manager, "_encode_and_upload_capture", fake_encode_and_upload)

    result = manager._encode_capture_upload_job("tele_data_coll", {"data_coll_id": "demo"})

    assert result == {"ok": True}
    assert seen["kind"] == "tele_data_coll"
    assert seen["params"] == {"data_coll_id": "demo"}
    assert seen["encoder"] is recording.encode_raw_capture_subprocess
    assert seen["packager"] is recording_package.package_encoded_captures


def test_encode_and_upload_posts_completion_event(tmp_path, monkeypatch) -> None:
    events = []
    manager = _manager(
        tmp_path,
        post_upload_event=lambda event, params, qos=0: events.append((event, params, qos)),
    )
    capture_dir = tmp_path / "tele_data_coll" / "demo" / "episode_000001"
    capture_dir.mkdir(parents=True)
    (capture_dir / "collection_meta.json").write_text("{}", encoding="utf-8")

    fake_oss_manager = types.SimpleNamespace(
        upload_package_if_requested=lambda _zip_path, *, params, kind: None,
    )
    monkeypatch.setitem(sys.modules, "rynnrcp_app_rynnbot.oss_manager", fake_oss_manager)

    result = manager._encode_and_upload_capture(
        "tele_data_coll",
        {"capture_dir": str(capture_dir)},
        lambda capture_dir, **_kwargs: {
            "capture_dir": capture_dir,
            "output_dir": str(tmp_path / "encoded"),
            "total_frames": 3,
            "videos": {},
        },
        lambda encoded, **_kwargs: {
            "zip_path": str(tmp_path / "package.zip"),
            "episodes": encoded,
            "size_bytes": 123,
        },
    )

    assert result["zip_path"] == str(tmp_path / "package.zip")
    assert events == [("occupancy_complete", {"occupancy_id": ""}, 1)]


def test_encode_and_upload_can_skip_completion_event(tmp_path, monkeypatch) -> None:
    events = []
    manager = _manager(
        tmp_path,
        post_upload_event=lambda event, params, qos=0: events.append((event, params, qos)),
    )
    capture_dir = tmp_path / "tele_data_coll" / "demo" / "episode_000001"
    capture_dir.mkdir(parents=True)
    (capture_dir / "collection_meta.json").write_text("{}", encoding="utf-8")
    monkeypatch.setitem(
        sys.modules,
        "rynnrcp_app_rynnbot.oss_manager",
        types.SimpleNamespace(upload_package_if_requested=lambda _zip_path, *, params, kind: None),
    )

    manager._encode_and_upload_capture(
        "tele_data_coll",
        {"capture_dir": str(capture_dir), "post_upload_event": "false"},
        lambda capture_dir, **_kwargs: {"capture_dir": capture_dir, "output_dir": str(tmp_path / "encoded")},
        lambda _encoded, **_kwargs: {"zip_path": str(tmp_path / "package.zip"), "size_bytes": 123},
    )

    assert events == []


def test_encode_and_upload_deletes_uploaded_artifacts_by_default(tmp_path, monkeypatch) -> None:
    manager = _manager(tmp_path)
    capture_dir = tmp_path / "tele_data_coll" / "demo" / "episode_000001"
    capture_dir.mkdir(parents=True)
    (capture_dir / "collection_meta.json").write_text("{}", encoding="utf-8")
    package_path = tmp_path / "package.zip"
    package_path.write_bytes(b"zip")
    monkeypatch.setitem(
        sys.modules,
        "rynnrcp_app_rynnbot.oss_manager",
        types.SimpleNamespace(upload_package_if_requested=lambda _zip_path, *, params, kind: {"ok": True}),
    )

    manager._encode_and_upload_capture(
        "tele_data_coll",
        {"capture_dir": str(capture_dir)},
        lambda capture_dir, **_kwargs: {"capture_dir": capture_dir, "output_dir": str(tmp_path / "encoded")},
        lambda _encoded, **_kwargs: {"zip_path": str(package_path), "size_bytes": 3},
    )

    assert not package_path.exists()
    assert not capture_dir.exists()
    assert not (tmp_path / "tele_data_coll" / "demo").exists()


def test_encode_and_upload_can_keep_uploaded_artifacts_for_debugging(tmp_path, monkeypatch) -> None:
    manager = _manager(tmp_path)
    capture_dir = tmp_path / "tele_data_coll" / "demo" / "episode_000001"
    capture_dir.mkdir(parents=True)
    (capture_dir / "collection_meta.json").write_text("{}", encoding="utf-8")
    package_path = tmp_path / "package.zip"
    package_path.write_bytes(b"zip")
    monkeypatch.setitem(
        sys.modules,
        "rynnrcp_app_rynnbot.oss_manager",
        types.SimpleNamespace(upload_package_if_requested=lambda _zip_path, *, params, kind: {"ok": True}),
    )

    manager._encode_and_upload_capture(
        "tele_data_coll",
        {
            "capture_dir": str(capture_dir),
            "delete_uploaded_package": False,
            "delete_uploaded_raw_captures": False,
        },
        lambda capture_dir, **_kwargs: {"capture_dir": capture_dir, "output_dir": str(tmp_path / "encoded")},
        lambda _encoded, **_kwargs: {"zip_path": str(package_path), "size_bytes": 3},
    )

    assert package_path.is_file()
    assert capture_dir.is_dir()
