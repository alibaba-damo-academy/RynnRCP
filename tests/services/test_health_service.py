"""Tests for HealthService protocol reporting."""

from __future__ import annotations

import time
from typing import Any

import pytest

from rynnrcp.config.runner_config import RunnerHealthSpec, RunnerInputSpec
from rynnrcp.protocol.methods import GET_HEALTH
from rynnrcp.runtime.tool_bus import ToolBus
from rynnrcp.services.base_service import BaseService
from rynnrcp.services.health_service import HealthService
from rynnrcp.utils.payload import json_observation_payload, pack_channel_message


class _FakeSubscriber:
    def __init__(self) -> None:
        self.raw: bytes | None = None

    def read_latest(self) -> bytes | None:
        return self.raw


def _input_spec(name: str = "observation.robot.joint_state") -> RunnerInputSpec:
    return RunnerInputSpec(
        name=name,
        runner_name="robot",
        protocol="module",
        adapter="",
        channel=name,
        msg_size=4096,
        object_name=name,
    )


def _health_spec(name: str = "health.robot") -> RunnerHealthSpec:
    return RunnerHealthSpec(
        name=name,
        runner_name="robot",
        protocol="module",
        channel=name,
        msg_size=4096,
        object_name=name,
        info={"component_name": "robot_component"},
    )


@pytest.fixture
def make_service(monkeypatch: pytest.MonkeyPatch):
    """Build a HealthService with channel subscription stubbed out."""

    def factory(
        inputs: list[RunnerInputSpec],
        healths: list[RunnerHealthSpec] = (),
        stale_after_s: float = 2.0,
    ) -> tuple[HealthService, dict[str, _FakeSubscriber]]:
        subscribed: list[str] = []
        monkeypatch.setattr(
            BaseService,
            "subscribe_channel",
            lambda self, channel, msg_size, transport="shm": subscribed.append(channel),
        )
        service = HealthService(
            ToolBus(), inputs=inputs, healths=healths, stale_after_s=stale_after_s
        )
        subs: dict[str, _FakeSubscriber] = {}
        for channel in subscribed:
            subs[channel] = _FakeSubscriber()
        service._subscribers = dict(subs)
        return service, subs

    return factory


def _publish(sub: _FakeSubscriber, object_name: str, value: Any, timestamp: float) -> None:
    payload = json_observation_payload(value, object_name, timestamp)
    sub.raw = pack_channel_message(timestamp, payload)


def test_bind_registers_and_unbind_removes_get_health(make_service) -> None:
    service, _subs = make_service([_input_spec()])
    service.bind()
    assert GET_HEALTH.name in service.bus.list_tools()
    service.unbind()
    assert GET_HEALTH.name not in service.bus.list_tools()


def test_reports_no_data_warning_for_silent_inputs(make_service) -> None:
    service, _subs = make_service([_input_spec()])
    result = service.get_health()
    assert result["success"] is True
    messages = result["result"]["msg"]
    assert len(messages) == 1
    assert messages[0]["code"] == "observation.no_data"
    assert messages[0]["source"] == "robot"


def test_fresh_observation_produces_no_messages(make_service) -> None:
    spec = _input_spec()
    service, subs = make_service([spec])
    _publish(subs[spec.channel], spec.object_name, {"joint_positions": [0.0]}, time.time())
    assert service.get_health()["result"]["msg"] == []


def test_stale_observation_produces_stale_warning(make_service) -> None:
    spec = _input_spec()
    service, subs = make_service([spec], stale_after_s=0.5)
    _publish(
        subs[spec.channel],
        spec.object_name,
        {"joint_positions": [0.0]},
        time.time() - 10.0,
    )
    messages = service.get_health()["result"]["msg"]
    assert [m["code"] for m in messages] == ["observation.stale"]


def test_uses_rcp_observation_name_from_spec_info(make_service) -> None:
    spec = RunnerInputSpec(
        name="observation.front.image",
        runner_name="front",
        protocol="port",
        adapter="",
        channel="observation.front.image",
        msg_size=16384,
        object_name="observation.front.image",
        info={"rcp_observation_name": "front_camera"},
    )
    service, _subs = make_service([spec])
    messages = service.get_health()["result"]["msg"]
    assert "front_camera" in messages[0]["message"]


def test_health_source_issues_are_normalized(make_service) -> None:
    spec = _health_spec()
    service, subs = make_service([], healths=[spec])
    now = time.time()
    _publish(
        subs[spec.channel],
        spec.object_name,
        {
            "errors": [{"code": "motor.fault", "message": "overheat", "level": "fatal"}],
            "warnings": ["low battery"],
            "msg": [{"message": "booted"}],
        },
        now,
    )

    messages = service.get_health()["result"]["msg"]
    by_code = {m["code"]: m for m in messages}
    assert by_code["motor.fault"]["level"] == "fatal"
    assert by_code["motor.fault"]["source"] == "robot_component"
    assert by_code["health.issue"]["level"] in {"warning", "info"}
    levels = sorted(m["level"] for m in messages)
    assert levels == ["fatal", "info", "warning"]


def test_health_source_invalid_payload_becomes_warning(make_service) -> None:
    spec = _health_spec()
    service, subs = make_service([], healths=[spec])
    sub = subs[spec.channel]
    sub.raw = pack_channel_message(time.time(), b"\xff\xfe not json")

    messages = service.get_health()["result"]["msg"]
    assert [m["code"] for m in messages] == ["health.invalid_payload"]
    assert messages[0]["level"] == "warning"


def test_health_source_non_mapping_payload_is_error(make_service) -> None:
    spec = _health_spec()
    service, subs = make_service([], healths=[spec])
    _publish(subs[spec.channel], spec.object_name, [1, 2, 3], time.time())

    messages = service.get_health()["result"]["msg"]
    assert messages[0]["code"] == "health.invalid_payload"
    assert messages[0]["level"] == "error"


def test_health_source_non_list_issue_groups_are_flagged(make_service) -> None:
    spec = _health_spec()
    service, subs = make_service([], healths=[spec])
    _publish(
        subs[spec.channel],
        spec.object_name,
        {"errors": "broken"},
        time.time(),
    )

    messages = service.get_health()["result"]["msg"]
    assert messages[0]["code"] == "health.invalid_payload"
    assert "must be a list" in messages[0]["message"]


def test_health_source_none_payload_produces_nothing(make_service) -> None:
    spec = _health_spec()
    service, subs = make_service([], healths=[spec])
    _publish(subs[spec.channel], spec.object_name, None, time.time())
    assert service.get_health()["result"]["msg"] == []
