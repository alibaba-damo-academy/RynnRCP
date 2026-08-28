"""Tests for the protocol-level client wrapper and manifest resolution."""

from __future__ import annotations

from typing import Any

import pytest

from rynnrcp.interface.codec import InterfaceError, InterfaceResponse
from rynnrcp.interface.discovery import Endpoint
from rynnrcp.interface.protocol_client import (
    RcpProtocolClient,
    ServerManifest,
    ServerSelectionError,
    _dedupe_manifests_by_instance,
    _endpoint_priority,
    _instance_key,
    connect_to_server,
    discover_manifests,
    resolve_server_manifest,
)


def _manifest_payload(robot_id: str = "so101") -> dict[str, Any]:
    return {
        "robot_id": robot_id,
        "robot_name": "SO101",
        "embodiment_type": "single_arm",
        "components": [{"name": "robot"}],
        "observations": [{"name": "observation.robot.joint_state"}],
        "actions": [{"name": "action.robot.joint_position"}],
        "capabilities": {"observations": True, "actions": False},
    }


class _FakeConnection:
    def __init__(self, responses: dict[str, InterfaceResponse] | None = None) -> None:
        self.requests: list[tuple[str, Any, int | None]] = []
        self.subscriptions: list[tuple[str, Any, dict, int | None]] = []
        self.responses = responses or {}
        self.closed = False

    def request(self, method: str, payload: Any = None, *, timeout_ms=None):
        self.requests.append((method, payload, timeout_ms))
        return self.responses.get(
            method, InterfaceResponse(request_id="1", payload={"echo": method})
        )

    def subscribe(self, method: str, payload: Any = None, *, metadata=None, timeout_ms=None):
        self.subscriptions.append((method, payload, dict(metadata or {}), timeout_ms))
        return iter(())

    def close(self) -> None:
        self.closed = True


class _FakeInterface:
    def __init__(self, endpoints, manifests: dict[str, dict[str, Any]]) -> None:
        self.endpoints = list(endpoints)
        self.manifests = manifests
        self.connections: list[_FakeConnection] = []

    def discover(self, timeout_s: float = 1.5):
        return list(self.endpoints)

    def connect(self, endpoint: Endpoint) -> _FakeConnection:
        payload = self.manifests.get(endpoint.endpoint_id)
        if payload is None:
            raise ConnectionError("unreachable")
        connection = _FakeConnection(
            {"get_manifest": InterfaceResponse(request_id="1", payload=payload)}
        )
        self.connections.append(connection)
        return connection


# ---------------------------------------------------------------------------
# ServerManifest
# ---------------------------------------------------------------------------

def test_server_manifest_from_payload_and_capabilities() -> None:
    manifest = ServerManifest.from_payload(_manifest_payload())
    assert manifest.robot_id == "so101"
    assert manifest.can_provide_state is True
    assert manifest.can_accept_action is False
    assert manifest.endpoint is None
    assert manifest.model_refs == {}


# ---------------------------------------------------------------------------
# RcpProtocolClient request wiring
# ---------------------------------------------------------------------------

@pytest.fixture
def client() -> tuple[RcpProtocolClient, _FakeConnection]:
    connection = _FakeConnection()
    return RcpProtocolClient(connection), connection


def test_simple_requests_use_expected_methods(client) -> None:
    protocol, connection = client
    protocol.ping()
    protocol.list_tools()
    protocol.list_observations()
    protocol.list_actions()
    protocol.list_policies()
    protocol.get_health()
    protocol.stop_collection()
    protocol.get_collection_status()
    methods = [method for method, _payload, _timeout in connection.requests]
    assert methods == [
        "ping",
        "list_tools",
        "list_observations",
        "list_actions",
        "list_policies",
        "get_health",
        "stop_collection",
        "get_collection_status",
    ]
    protocol.close()
    assert connection.closed is True


def test_get_manifest_parses_payload(client) -> None:
    protocol, connection = client
    connection.responses["get_manifest"] = InterfaceResponse(
        request_id="1", payload=_manifest_payload()
    )
    manifest = protocol.get_manifest()
    assert manifest.robot_name == "SO101"

    connection.responses["get_manifest"] = InterfaceResponse(
        request_id="1", status=500, message="boom", payload=None
    )
    with pytest.raises(InterfaceError, match="get_manifest failed"):
        protocol.get_manifest()


def test_observation_and_action_payloads(client) -> None:
    protocol, connection = client
    protocol.get_observations(["a", "b"], sync=True)
    protocol.run_action_chunk("action.x", [{"joint_positions": [1]}], frame_rate=30)
    protocol.stop_action("estop")
    protocol.stop_action()

    payloads = {method: payload for method, payload, _ in connection.requests}
    assert payloads["get_observations"] == {"names": ["a", "b"], "sync": True}
    assert payloads["run_action_chunk"] == {
        "name": "action.x",
        "frames": [{"joint_positions": [1]}],
        "frame_rate": 30.0,
    }
    stop_calls = [p for m, p, _ in connection.requests if m == "stop_action"]
    assert stop_calls == [{"reason": "estop"}, {}]


def test_subscription_methods_pass_metadata(client) -> None:
    protocol, connection = client
    protocol.subscribe_observations(["a"], stream_hz=15.0)
    protocol.run_action_chunk_async("action.x", [], frame_rate=10)
    protocol.subscribe_health(stream_hz=1.0)

    methods = [entry[0] for entry in connection.subscriptions]
    assert methods == ["get_observations", "run_action_chunk", "get_health"]
    assert connection.subscriptions[0][2] == {"stream_hz": 15.0}
    assert connection.subscriptions[1][2] == {"stream_once": True}


def test_policy_payload_shapes(client) -> None:
    protocol, connection = client
    protocol.start_policy("p1")
    protocol.start_policy("p1", runtime_inputs={"prompt": "x"})
    protocol.update_policy_inputs(runtime_inputs={"prompt": "y"})
    protocol.update_policy_inputs(policy_id="p1", runtime_inputs={})
    protocol.stop_policy()
    protocol.stop_policy("p1", reason="done")

    payloads = [payload for _method, payload, _timeout in connection.requests]
    assert payloads == [
        {"policy_id": "p1"},
        {"policy_id": "p1", "runtime_inputs": {"prompt": "x"}},
        {"runtime_inputs": {"prompt": "y"}},
        {"runtime_inputs": {}, "policy_id": "p1"},
        {},
        {"policy_id": "p1", "reason": "done"},
    ]


def test_resource_payload_shapes(client) -> None:
    protocol, connection = client
    protocol.get_resource_info("r1")
    protocol.list_resources(domain="data", kind="collection", cursor="2", limit=10)
    protocol.list_resources()
    protocol.list_resource_entries("r1", recursive=True, cursor="0", limit=5)
    protocol.read_resource("r1", offset=8, limit=16)
    protocol.delete_resource("r1")
    protocol.snapshot_resource("r1")
    protocol.prepare_resource_archive(resource_id="r1")
    protocol.prepare_resource_archive(resource_ids=["a", "b"], format="zip")
    protocol.delete_collection("r1")

    payloads = {**{m: p for m, p, _ in connection.requests}}
    assert payloads["get_resource_info"] == {"resource_id": "r1"}
    assert payloads["list_resource_entries"] == {
        "resource_id": "r1",
        "recursive": True,
        "cursor": "0",
        "limit": 5,
    }
    assert payloads["read_resource"] == {"resource_id": "r1", "offset": 8, "limit": 16}
    assert payloads["delete_collection"] == {"resource_id": "r1"}
    archive_calls = [p for m, p, _ in connection.requests if m == "prepare_resource_archive"]
    assert archive_calls == [
        {"format": "zip", "resource_id": "r1"},
        {"format": "zip", "resource_ids": ["a", "b"]},
    ]
    list_calls = [p for m, p, _ in connection.requests if m == "list_resources"]
    assert list_calls == [
        {"domain": "data", "kind": "collection", "cursor": "2", "limit": 10},
        {},
    ]


def test_start_collection_payload(client) -> None:
    protocol, connection = client
    protocol.start_collection(
        ["obs"],
        collection_id="c",
        episode_id="e",
        task_prompt="p",
        task_description="d",
        frame_rate=25,
        max_duration=60,
        metadata={"k": "v"},
    )
    _method, payload, _timeout = connection.requests[0]
    assert payload == {
        "names": ["obs"],
        "collection_id": "c",
        "episode_id": "e",
        "task_prompt": "p",
        "task_description": "d",
        "frame_rate": 25.0,
        "max_duration": 60.0,
        "metadata": {"k": "v"},
    }


# ---------------------------------------------------------------------------
# Discovery / resolution helpers
# ---------------------------------------------------------------------------

def _endpoint(endpoint_id: str, *, source: str = "mdns", address: str = "10.0.0.1:8080",
              metadata: dict | None = None) -> Endpoint:
    return Endpoint(
        endpoint_id=endpoint_id,
        transport="grpc",
        address=address,
        metadata=metadata or {},
        source=source,
    )


def test_discover_manifests_skips_unreachable_and_dedupes() -> None:
    same_instance = {"server_instance_id": "inst-1"}
    interface = _FakeInterface(
        [
            _endpoint("a", source="mdns", metadata=same_instance),
            _endpoint("b", source="static", address="127.0.0.1:8080", metadata=same_instance),
            _endpoint("dead"),
        ],
        manifests={"a": _manifest_payload(), "b": _manifest_payload()},
    )
    manifests = discover_manifests(interface=interface)
    assert len(manifests) == 1
    # The static endpoint wins over mdns for the same server instance.
    assert manifests[0].endpoint.source == "static"
    assert all(connection.closed for connection in interface.connections)

    undeduped = discover_manifests(interface=interface, dedupe_instances=False)
    assert len(undeduped) == 2


def test_resolve_server_manifest_matches_robot_id() -> None:
    interface = _FakeInterface(
        [_endpoint("a")], manifests={"a": _manifest_payload("so101")}
    )
    manifest = resolve_server_manifest(robot_id="so101", interface=interface)
    assert manifest.robot_id == "so101"

    with pytest.raises(ServerSelectionError, match="was not found"):
        resolve_server_manifest(robot_id="other", interface=interface)
    with pytest.raises(ServerSelectionError, match="robot_id is required"):
        resolve_server_manifest(robot_id=" ", interface=interface)


def test_resolve_server_manifest_filters_embodiment_type() -> None:
    interface = _FakeInterface(
        [_endpoint("a")], manifests={"a": _manifest_payload("so101")}
    )
    with pytest.raises(ServerSelectionError, match="was not found"):
        resolve_server_manifest(
            robot_id="so101", embodiment_type="humanoid", interface=interface
        )


def test_resolve_server_manifest_rejects_duplicate_instances() -> None:
    interface = _FakeInterface(
        [
            _endpoint("a", address="10.0.0.1:8080"),
            _endpoint("b", address="10.0.0.2:8080"),
        ],
        manifests={"a": _manifest_payload("so101"), "b": _manifest_payload("so101")},
    )
    with pytest.raises(ServerSelectionError, match="duplicate robot_id"):
        resolve_server_manifest(robot_id="so101", interface=interface)


def test_connect_to_server_returns_protocol_client() -> None:
    interface = _FakeInterface(
        [_endpoint("a")], manifests={"a": _manifest_payload("so101")}
    )
    protocol = connect_to_server(robot_id="so101", interface=interface)
    assert isinstance(protocol, RcpProtocolClient)


def test_endpoint_priority_and_instance_key() -> None:
    assert _endpoint_priority(None) == 99
    assert _endpoint_priority(_endpoint("a", source="static")) == 0
    assert _endpoint_priority(_endpoint("a", source="local_registry")) == 1
    assert _endpoint_priority(_endpoint("a", source="mdns")) == 2
    assert _endpoint_priority(_endpoint("a", source="other")) == 50

    manifest = ServerManifest.from_payload(_manifest_payload())
    assert _instance_key(manifest) == "so101"

    with_endpoint = ServerManifest.from_payload(
        _manifest_payload(), endpoint=_endpoint("a", metadata={"server_instance_id": "i9"})
    )
    assert _instance_key(with_endpoint) == "so101:grpc:instance:i9"

    without_instance = ServerManifest.from_payload(
        _manifest_payload(), endpoint=_endpoint("a")
    )
    assert _instance_key(without_instance) == "so101:grpc:10.0.0.1:8080"


def test_dedupe_keeps_distinct_instances() -> None:
    first = ServerManifest.from_payload(
        _manifest_payload("r1"), endpoint=_endpoint("a", address="1.1.1.1:1")
    )
    second = ServerManifest.from_payload(
        _manifest_payload("r2"), endpoint=_endpoint("b", address="2.2.2.2:2")
    )
    assert len(_dedupe_manifests_by_instance([first, second])) == 2
