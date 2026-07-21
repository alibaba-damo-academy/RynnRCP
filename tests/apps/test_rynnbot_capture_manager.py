"""RynnBot capture upload regression tests."""

from __future__ import annotations

import json
import sys
import types
from types import SimpleNamespace
from typing import Any, Dict

from rynnrcp_app_common import recording, recording_package
from rynnrcp_app_rynnbot.capture_manager import RynnBotCaptureManager, _encoded_capture_diagnostics


def _manager(
    tmp_path,
    post_upload_event=None,
    server_client=None,
    stop_ws_state_stream=None,
    mqtt_send=None,
) -> RynnBotCaptureManager:
    return RynnBotCaptureManager(
        config={"max_capture_duration_s": 600, "video_backend": "auto", "video_encoder": None},
        raw_capture_root=tmp_path,
        server_client_provider=lambda: server_client,
        mqtt_send=mqtt_send or (lambda _topic, _payload, qos=0: None),
        parse_json_or_reply_error=lambda _payload, _send_topic, _kind: {},
        make_base_response=lambda _request: {"result": {}},
        start_ws_state_stream=lambda: None,
        stop_ws_state_stream=stop_ws_state_stream or (lambda: None),
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


def test_encoded_capture_diagnostics_exposes_timing_and_reuse_stats(tmp_path) -> None:
    output_dir = tmp_path / "encoded"
    output_dir.mkdir()
    (output_dir / "metadata.json").write_text(
        """{
          "fps": 30,
          "timestamp_policy": "relative_frame_index",
          "total_frames": 301,
          "alignment_policy": "nearest",
          "alignment": {
            "recording_duration_s": 10.0,
            "frames_recorded_by_key": {"action": 601},
            "recording_duration_by_key": {"action": 10.0},
            "reused_sample_rate_by_key": {"action": 0.0}
          }
        }""",
        encoding="utf-8",
    )

    assert _encoded_capture_diagnostics({"output_dir": str(output_dir)}) == {
        "fps": 30,
        "timestamp_policy": "relative_frame_index",
        "total_frames": 301,
        "alignment_policy": "nearest",
        "recording_duration_s": 10.0,
        "frames_recorded_by_key": {"action": 601},
        "recording_duration_by_key": {"action": 10.0},
        "reused_sample_rate_by_key": {"action": 0.0},
    }


def test_release_force_stop_downloads_and_remembers_active_capture(tmp_path, monkeypatch) -> None:
    capture_dir = tmp_path / "tele_data_coll" / "demo" / "episode_000001"
    capture_dir.mkdir(parents=True)
    (capture_dir / "collection_meta.json").write_text("{}", encoding="utf-8")
    status = {
        "collection_id": "demo",
        "episode_id": "episode_000001",
        "duration": 2.0,
        "collection_resource": {"resource_id": "resource-1"},
        "per_name_counts": {"action": 60},
        "stream_stats": {"action": {"fps": 30.0}},
    }

    class Client:
        def __init__(self) -> None:
            self.stop_calls = 0

        def stop_collection(self):
            self.stop_calls += 1
            return SimpleNamespace(ok=True, payload=status, message="")

    client = Client()
    stop_state_calls = []
    manager = _manager(
        tmp_path,
        server_client=client,
        stop_ws_state_stream=lambda: stop_state_calls.append(True),
    )
    manager._set_capture_state(
        "tele_data_coll",
        "episode_000001",
        True,
        None,
        5.0,
        {"data_coll_id": "demo", "episode_number": 1},
    )
    monkeypatch.setattr(
        manager,
        "_download_collection_resource",
        lambda kind, stopped_status: str(capture_dir),
    )

    cleanup = manager.force_stop_collection_on_release()

    state = manager.capture_state("tele_data_coll")
    assert client.stop_calls == 1
    assert cleanup[0]["kind"] == "tele_data_coll"
    assert cleanup[0]["capture_dir"] == str(capture_dir)
    assert state["recording"] is False
    assert state["last_episode_dir"] == str(capture_dir)
    assert manager._server_resource_by_capture_dir[str(capture_dir)] == "resource-1"
    assert stop_state_calls


def test_stop_round_replies_without_downloading_and_is_idempotent(tmp_path, monkeypatch) -> None:
    status = {
        "collection_id": "demo",
        "episode_id": "episode_000001",
        "duration": 2.0,
        "collection_resource": {"resource_id": "resource-1"},
    }

    class Client:
        def __init__(self) -> None:
            self.stop_calls = 0

        def stop_collection(self):
            self.stop_calls += 1
            return SimpleNamespace(ok=True, payload=status, message="")

    client = Client()
    sent = []
    stop_state_calls = []
    manager = _manager(
        tmp_path,
        server_client=client,
        mqtt_send=lambda topic, payload, qos=0: sent.append((topic, json.loads(payload), qos)),
        stop_ws_state_stream=lambda: stop_state_calls.append(True),
    )
    manager._set_capture_state(
        "tele_data_coll",
        "episode_000001",
        True,
        None,
        5.0,
        {"data_coll_id": "demo", "episode_number": 1},
    )
    monkeypatch.setattr(
        manager,
        "_download_collection_resource",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("stop_round must not download")),
    )

    topic = "sys/pk/dn/rrpc/request/0/tele_data_coll:stop_round"
    manager.stop_collection_from_mqtt(topic, "{}", "tele_data_coll")
    manager.stop_collection_from_mqtt(topic, "{}", "tele_data_coll")

    assert client.stop_calls == 1
    assert manager.capture_state("tele_data_coll")["recording"] is False
    assert manager.capture_state("tele_data_coll")["last_episode_dir"] is None
    assert manager._latest_stopped_collection("tele_data_coll")["status"] == status
    assert [item[0] for item in sent] == [
        "sys/pk/dn/rrpc/response/0/tele_data_coll:stop_round",
        "sys/pk/dn/rrpc/response/0/tele_data_coll:stop_round",
    ]
    assert all("error" not in item[1] for item in sent)
    assert stop_state_calls == [True]


def test_upload_downloads_deferred_capture_then_forgets_it_after_success(tmp_path, monkeypatch) -> None:
    status = {
        "collection_id": "demo",
        "episode_id": "episode_000001",
        "collection_resource": {"resource_id": "resource-1"},
    }
    manager = _manager(tmp_path)
    manager._finalize_stopped_collection(
        "tele_data_coll",
        status,
        trigger="stop_round",
        defer_download=True,
    )
    capture_dir = tmp_path / "tele_data_coll" / "demo" / "episode_000001"
    download_calls = []

    def download(kind, stopped_status):
        download_calls.append((kind, stopped_status))
        capture_dir.mkdir(parents=True)
        (capture_dir / "collection_meta.json").write_text("{}", encoding="utf-8")
        return str(capture_dir)

    monkeypatch.setattr(manager, "_download_collection_resource", download)
    monkeypatch.setitem(
        sys.modules,
        "rynnrcp_app_rynnbot.oss_manager",
        types.SimpleNamespace(upload_package_if_requested=lambda _zip_path, *, params, kind: {"ok": True}),
    )
    encoded_dirs = []

    manager._encode_and_upload_capture(
        "tele_data_coll",
        {
            "data_coll_id": "demo",
            "delete_uploaded_package": False,
            "delete_uploaded_raw_captures": False,
        },
        lambda raw_dir, **_kwargs: encoded_dirs.append(raw_dir)
        or {"capture_dir": raw_dir, "output_dir": str(tmp_path / "encoded")},
        lambda _encoded, **_kwargs: {"zip_path": str(tmp_path / "package.zip"), "size_bytes": 123},
    )

    assert download_calls == [("tele_data_coll", status)]
    assert encoded_dirs == [str(capture_dir)]
    assert manager._latest_stopped_collection("tele_data_coll") is None


def test_explicit_capture_dir_does_not_download_deferred_capture(tmp_path, monkeypatch) -> None:
    manager = _manager(tmp_path)
    manager._remember_stopped_collection(
        "tele_data_coll",
        {
            "collection_id": "deferred",
            "episode_id": "episode_000001",
            "collection_resource": {"resource_id": "resource-1"},
        },
    )
    explicit_dir = tmp_path / "explicit"
    explicit_dir.mkdir()
    (explicit_dir / "collection_meta.json").write_text("{}", encoding="utf-8")
    monkeypatch.setattr(
        manager,
        "_download_collection_resource",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("explicit upload must not download")),
    )
    monkeypatch.setitem(
        sys.modules,
        "rynnrcp_app_rynnbot.oss_manager",
        types.SimpleNamespace(upload_package_if_requested=lambda _zip_path, *, params, kind: None),
    )

    result = manager._encode_and_upload_capture(
        "tele_data_coll",
        {"capture_dir": str(explicit_dir)},
        lambda raw_dir, **_kwargs: {"capture_dir": raw_dir, "output_dir": str(tmp_path / "encoded")},
        lambda _encoded, **_kwargs: {"zip_path": str(tmp_path / "package.zip"), "size_bytes": 123},
    )

    assert result["capture_dirs"] == [str(explicit_dir)]
