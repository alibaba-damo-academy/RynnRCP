"""Tests for CollectionService and its background session."""

from __future__ import annotations

import io
import os
import time
from pathlib import Path
from typing import Any, Dict

import msgpack
import pytest

from rynnrcp.config.runner_config import RunnerInputSpec
from rynnrcp.runtime.tool_bus import ToolBus
from rynnrcp.services.base_service import BaseService
from rynnrcp.services.collection_service import (
    CollectionService,
    _CollectionSession,
    _pack_safe,
)
from rynnrcp.services.resource_service import ResourceRegistry
from rynnrcp.utils.payload import json_observation_payload, pack_channel_message


class _FakeSubscriber:
    def __init__(self) -> None:
        self.raw: bytes | None = None

    def read_latest(self) -> bytes | None:
        return self.raw


def _joint_state_spec() -> RunnerInputSpec:
    return RunnerInputSpec(
        name="observation.robot.joint_state",
        runner_name="robot",
        protocol="module",
        adapter="",
        channel="observation.robot.joint_state",
        msg_size=4096,
        object_name="observation.robot.joint_state",
        info={
            "rcp_observation_type": "joint_state",
            "rcp_observation_name": "observation.robot.joint_state",
            "frame_rate": 30,
            "description": "robot joints",
        },
    )


def _action_spec() -> RunnerInputSpec:
    return RunnerInputSpec(
        name="action.robot.joint_position",
        runner_name="robot",
        protocol="module",
        adapter="",
        channel="action.robot.joint_position",
        msg_size=4096,
        object_name="action.robot.joint_position",
        info={
            "rcp_action_name": "action.robot.joint_position",
            "rcp_action_type": "joint_position",
        },
    )


@pytest.fixture
def service(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    monkeypatch.setattr(
        BaseService,
        "subscribe_channel",
        lambda self, channel, msg_size, transport="shm": None,
    )
    registry = ResourceRegistry(str(tmp_path / "tmp"))
    specs = [_joint_state_spec(), _action_spec()]
    service = CollectionService(ToolBus(), specs, str(tmp_path / "datasets"), registry)
    subs = {spec.channel: _FakeSubscriber() for spec in specs}
    service._subscribers = dict(subs)
    yield service, subs, registry
    service._session.stop()


def _publish(sub: _FakeSubscriber, object_name: str, value: Any, timestamp: float) -> None:
    payload = json_observation_payload(value, object_name, timestamp)
    sub.raw = pack_channel_message(timestamp, payload)


def _start_args(**overrides: Any) -> Dict[str, Any]:
    args: Dict[str, Any] = {
        "names": ["observation.robot.joint_state"],
        "collection_id": "col-1",
        "episode_id": "ep-1",
        "task_prompt": "pick",
        "task_description": "pick the cube",
    }
    args.update(overrides)
    return args


# ---------------------------------------------------------------------------
# _pack_safe
# ---------------------------------------------------------------------------

def test_pack_safe_supports_msgpack_friendly_values() -> None:
    value = {"a": [1, 2.0, True, None, b"raw", ("t",)], 5: "five"}
    assert _pack_safe(value) == {"a": [1, 2.0, True, None, b"raw", ["t"]], "5": "five"}
    with pytest.raises(TypeError, match="not msgpack compatible"):
        _pack_safe({"bad": object()})


# ---------------------------------------------------------------------------
# _CollectionSession
# ---------------------------------------------------------------------------

def _session_samples(names, last_ts):
    now = time.time()
    return {name: {"timestamp": now, "value": {"joint_positions": [1.0]}} for name in names}


def test_session_start_validates_arguments(tmp_path: Path) -> None:
    session = _CollectionSession(_session_samples, str(tmp_path))
    base = dict(
        collection_id="c",
        episode_id="e",
        names=["n"],
        items=[],
        fps=30.0,
        max_duration_s=None,
        metadata={},
    )
    with pytest.raises(ValueError, match="collection_id is required"):
        session.start(**{**base, "collection_id": " "})
    with pytest.raises(ValueError, match="episode_id is required"):
        session.start(**{**base, "episode_id": ""})
    with pytest.raises(ValueError, match="frame_rate must be greater than 0"):
        session.start(**{**base, "fps": 0})
    with pytest.raises(ValueError, match="max_duration must be greater than 0"):
        session.start(**{**base, "max_duration_s": -1})


def test_session_records_samples_and_writes_streams(tmp_path: Path) -> None:
    session = _CollectionSession(_session_samples, str(tmp_path))
    status = session.start(
        collection_id="col",
        episode_id="ep",
        names=["observation.robot.joint_state"],
        items=[{"protocol_name": "observation.robot.joint_state"}],
        fps=100.0,
        max_duration_s=None,
        metadata={"task_prompt": "p"},
    )
    assert status["running"] is True
    time.sleep(0.2)
    status = session.stop()

    assert status["running"] is False
    assert status["stop_reason"] == "manual"
    assert status["counts"]["observation.robot.joint_state"] > 0
    stats = status["stream_stats"]["observation.robot.joint_state"]
    assert stats["count"] == status["counts"]["observation.robot.joint_state"]
    assert stats["first_ts"] is not None

    output_dir = Path(status["output_dir"])
    assert (output_dir / "collection_meta.json").is_file()
    samples_file = (
        output_dir / "streams" / "observation.robot.joint_state" / "samples.msgpack"
    )
    unpacker = msgpack.Unpacker(io.BytesIO(samples_file.read_bytes()), raw=False)
    first = next(iter(unpacker))
    assert first["value"] == {"joint_positions": [1.0]}

    # Second start on the same episode replaces the previous output.
    session.start(
        collection_id="col",
        episode_id="ep",
        names=["observation.robot.joint_state"],
        items=[],
        fps=100.0,
        max_duration_s=None,
        metadata={},
    )
    with pytest.raises(RuntimeError, match="already running"):
        session.start(
            collection_id="col2",
            episode_id="ep2",
            names=[],
            items=[],
            fps=30.0,
            max_duration_s=None,
            metadata={},
        )
    session.stop()


def test_session_stops_on_max_duration(tmp_path: Path) -> None:
    session = _CollectionSession(_session_samples, str(tmp_path))
    session.start(
        collection_id="col",
        episode_id="ep",
        names=["n"],
        items=[],
        fps=100.0,
        max_duration_s=0.05,
        metadata={},
    )
    deadline = time.time() + 2.0
    while session.is_running and time.time() < deadline:
        time.sleep(0.02)
    status = session.status()
    assert status["running"] is False
    assert status["stop_reason"] == "max_duration"


def test_session_records_sampling_errors(tmp_path: Path) -> None:
    def broken(names, last_ts):
        return {"n": "not-a-dict"}

    session = _CollectionSession(broken, str(tmp_path))
    session.start(
        collection_id="col",
        episode_id="ep",
        names=["n"],
        items=[],
        fps=100.0,
        max_duration_s=None,
        metadata={},
    )
    deadline = time.time() + 2.0
    while session.is_running and time.time() < deadline:
        time.sleep(0.02)
    status = session.status()
    assert status["stop_reason"] == "error"
    assert "must be a dict" in status["last_error"]


def test_session_skips_duplicate_timestamps(tmp_path: Path) -> None:
    fixed_ts = time.time()

    def fixed(names, last_ts):
        return {name: {"timestamp": fixed_ts, "value": 1} for name in names}

    session = _CollectionSession(fixed, str(tmp_path))
    session.start(
        collection_id="col",
        episode_id="ep",
        names=["n"],
        items=[],
        fps=200.0,
        max_duration_s=None,
        metadata={},
    )
    time.sleep(0.1)
    status = session.stop()
    assert status["counts"]["n"] == 1


# ---------------------------------------------------------------------------
# CollectionService tool surface
# ---------------------------------------------------------------------------

def test_bind_registers_all_collection_tools(service) -> None:
    svc, _subs, _registry = service
    svc.bind()
    tools = svc.bus.list_tools()
    for name in (
        "start_collection",
        "stop_collection",
        "get_collection_status",
        "delete_collection",
    ):
        assert any(name in tool for tool in tools), (name, tools)
    svc.unbind()


@pytest.mark.parametrize(
    "overrides, message",
    [
        ({"names": []}, "names must be a non-empty list"),
        ({"names": ["a", "a"]}, "Duplicate collection name"),
        ({"names": ["nope"]}, "Unknown collection name"),
        ({"collection_id": " "}, "collection_id is required"),
        ({"episode_id": ""}, "episode_id is required"),
        ({"task_prompt": ""}, "task_prompt is required"),
        ({"task_description": " "}, "task_description is required"),
    ],
)
def test_start_collection_rejects_invalid_requests(service, overrides, message) -> None:
    svc, _subs, _registry = service
    result = svc.start_collection(**_start_args(**overrides))
    assert result["success"] is False
    assert message in result["message"]


def test_start_collection_records_live_samples(service) -> None:
    svc, subs, registry = service
    _publish(
        subs["observation.robot.joint_state"],
        "observation.robot.joint_state",
        {"joint_positions": [0.5, 0.25]},
        time.time(),
    )

    result = svc.start_collection(**_start_args())
    assert result["success"] is True, result["message"]
    resource = result["result"]["collection_resource"]
    assert resource["resource_id"]
    assert registry.resolve(resource["resource_id"]).metadata["kind"] == "collection"

    status = svc.get_collection_status()["result"]
    assert status["running"] is True
    assert status["task_prompt"] == "pick"

    time.sleep(0.15)
    stopped = svc.stop_collection()["result"]
    assert stopped["collection_id"] == "col-1"
    assert stopped["per_name_counts"]["observation.robot.joint_state"] >= 1

    final = svc.get_collection_status()["result"]
    assert final["running"] is False


def test_start_collection_reports_session_errors(service) -> None:
    svc, _subs, _registry = service
    ok = svc.start_collection(**_start_args())
    assert ok["success"] is True
    conflicted = svc.start_collection(
        **_start_args(collection_id="col-2", episode_id="ep-2")
    )
    assert conflicted["success"] is False
    assert "already running" in conflicted["message"]
    svc.stop_collection()


def test_start_collection_rejects_bad_frame_rate(service) -> None:
    svc, _subs, _registry = service
    result = svc.start_collection(**_start_args(frame_rate=0))
    assert result["success"] is False
    assert "frame_rate" in result["message"]


def test_stop_collection_without_session_returns_empty_resource(service) -> None:
    svc, _subs, _registry = service
    result = svc.stop_collection()
    assert result["success"] is True
    assert result["result"]["collection_resource"] == {}


def test_get_collection_status_before_any_session(service) -> None:
    svc, _subs, _registry = service
    result = svc.get_collection_status()
    assert result["result"] == {"running": False}


def test_delete_collection_validations(service, tmp_path: Path) -> None:
    svc, subs, registry = service
    assert svc.delete_collection("")["success"] is False
    missing = svc.delete_collection("resource-does-not-exist")
    assert missing["success"] is False

    other_file = tmp_path / "model.bin"
    other_file.write_bytes(b"x")
    record = registry.register_path(
        str(other_file), resource_type="file", domain="model", name="model"
    )
    not_collection = svc.delete_collection(record["resource_id"])
    assert not_collection["success"] is False
    assert "not a collection" in not_collection["message"]


def test_delete_collection_blocks_running_and_deletes_stopped(service) -> None:
    svc, subs, _registry = service
    _publish(
        subs["observation.robot.joint_state"],
        "observation.robot.joint_state",
        {"joint_positions": [0.1]},
        time.time(),
    )
    started = svc.start_collection(**_start_args())
    rid = started["result"]["collection_resource"]["resource_id"]

    blocked = svc.delete_collection(rid)
    assert blocked["success"] is False
    assert "currently being collected" in blocked["message"]

    svc.stop_collection()
    output_dir = svc._session.status()["output_dir"]
    assert os.path.exists(output_dir)
    deleted = svc.delete_collection(rid)
    assert deleted["success"] is True
    assert deleted["result"]["deleted"] is True
    assert not os.path.exists(output_dir)


def test_unbind_stops_running_session(service) -> None:
    svc, _subs, _registry = service
    svc.bind()
    svc.start_collection(**_start_args())
    assert svc._session.is_running
    svc.unbind()
    assert not svc._session.is_running


# ---------------------------------------------------------------------------
# Sampling and value encoding
# ---------------------------------------------------------------------------

def test_sample_encodes_observation_and_action_values(service) -> None:
    svc, subs, _registry = service
    now = time.time()
    _publish(
        subs["observation.robot.joint_state"],
        "observation.robot.joint_state",
        {"joint_positions": [1, 2], "joint_velocities": [0, 0]},
        now,
    )
    _publish(
        subs["action.robot.joint_position"],
        "action.robot.joint_position",
        {"joint_positions": [3, 4]},
        now,
    )

    samples = svc._sample(
        ["observation.robot.joint_state", "action.robot.joint_position"], {}
    )
    assert samples["observation.robot.joint_state"]["value"] == {
        "joint_positions": [1.0, 2.0],
        "joint_velocities": [0.0, 0.0],
    }
    assert samples["action.robot.joint_position"]["value"] == {
        "joint_positions": [3.0, 4.0]
    }


def test_sample_skips_seen_invalid_and_unrequested(service) -> None:
    svc, subs, _registry = service
    now = time.time()
    _publish(
        subs["observation.robot.joint_state"],
        "observation.robot.joint_state",
        {"joint_positions": [1]},
        now,
    )
    # Invalid action payload (missing joint_positions) must be skipped quietly.
    _publish(
        subs["action.robot.joint_position"],
        "action.robot.joint_position",
        {"wrong": 1},
        now,
    )

    unrequested = svc._sample(["action.robot.joint_position"], {})
    assert unrequested == {}

    seen = svc._sample(
        ["observation.robot.joint_state"],
        {"observation.robot.joint_state": now},
    )
    assert seen == {}

    fresh = svc._sample(["observation.robot.joint_state"], {})
    assert set(fresh) == {"observation.robot.joint_state"}


def test_collection_item_descriptors_expose_protocol_metadata(service) -> None:
    svc, _subs, _registry = service
    observation_item = svc._item_by_name["observation.robot.joint_state"]
    assert observation_item["category"] == "observation"
    assert observation_item["component_name"] == "robot"
    assert observation_item["type"] == "joint_state"
    assert observation_item["description"] == "robot joints"
    assert observation_item["frame_rate"] == 30.0

    action_item = svc._item_by_name["action.robot.joint_position"]
    assert action_item["category"] == "action"
    assert action_item["type"] == "joint_position"
    assert action_item["name"] == "joint_position"
