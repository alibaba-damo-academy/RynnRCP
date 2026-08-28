from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from rynnrcp.interface.client import ClientInterface
from rynnrcp.interface.discovery import REGISTRY_ENV
from rynnrcp.interface.protocol_client import RcpProtocolClient
from rynnrcp.config.runtime_config import RuntimeConfig
from rynnrcp.runtime.tool_bus import ToolBus
from rynnrcp.server import RynnRCPServer


pytest.importorskip("grpc")

MOCK_INTEGRATION_PATH = str(Path(__file__).with_name("mock_runtime_integration.yaml"))


class FakeRuntime:
    def __init__(self, config: RuntimeConfig):
        self._config = config.runtime_context
        self.bus = ToolBus()
        self.started = False
        self.stopped = False
        self.bus.add_tool("get_manifest", self.get_manifest)
        self.bus.add_tool("get_observations", self.get_observations)
        self.bus.add_tool("get_health", self.get_health)
        self.bus.add_tool("run_action_chunk", self.run_action_chunk)

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def tool_list(self) -> dict[str, dict[str, Any]]:
        return self.bus.list_tools()

    def get_manifest(self) -> dict[str, Any]:
        return ToolBus.make_result(
            True,
            result={
                "robot_id": self._config["manifest"]["robot_id"],
                "robot_name": self._config["manifest"]["robot_name"],
                "embodiment_type": "single_arm",
                "components": [],
                "observations": [{"name": "observation.robot.joint_state", "type": "joint_state"}],
                "actions": [{"name": "action.robot.joint_position", "type": "joint_position"}],
                "capabilities": dict(self._config["manifest"]["capabilities"]),
                "model_refs": {},
                "metadata": {},
            },
        )

    def get_observations(self, names: list[str], sync: bool = False) -> dict[str, Any]:
        return ToolBus.make_result(
            True,
            result={
                "observations": [
                    {
                        "name": key,
                        "timestamp": 1.0,
                        "value": {"joint_positions": [1.0, 2.0, 3.0]},
                    }
                    for key in names
                ]
            },
        )

    def run_action_chunk(self, name: str, frames: list[dict[str, Any]], frame_rate: int = 100) -> dict[str, Any]:
        return ToolBus.make_result(True, result={"name": name, "accepted_frames": len(frames), "frame_rate": frame_rate})

    def get_health(self) -> dict[str, Any]:
        return ToolBus.make_result(True, result={"errors": [], "warnings": []})


def test_rynnrcp_server_exposes_toolbus_over_grpc() -> None:
    created: dict[str, FakeRuntime] = {}
    config = {
        "config_type": "rynnrcp_server_config",
        "version": 1,
            "manifest": {
                "robot_id": "fake_runtime",
                "robot_name": "fake runtime",
                "capabilities": {
                    "observations": True,
                    "actions": True,
                    "health": True,
                    "resources": True,
                    "data_collection": False,
                    "policy_service": False,
                },
            },
            "server": {
                "config_name": "so101",
                "interface": {
                    "host": "127.0.0.1",
                    "port": 0,
                    "local_registry": False,
                    "mdns": False,
                },
            },
            "integration": {"config": MOCK_INTEGRATION_PATH},
            "components": _fake_components(),
        }

    def runtime_factory(config_arg: object) -> FakeRuntime:
        runtime = FakeRuntime(config_arg)  # type: ignore[arg-type]
        created["runtime"] = runtime
        return runtime

    server = RynnRCPServer(
        config=config,
        runtime_factory=runtime_factory,
    )
    server.start()
    try:
        conn = ClientInterface.with_defaults().connect(f"127.0.0.1:{server.bound_port}")
        try:
            manifest = conn.request("get_manifest", {}, timeout_ms=1000)
            observation = conn.request(
                "get_observations",
                {"names": ["observation.robot.joint_state"]},
                timeout_ms=1000,
            )
            action = conn.request(
                "run_action_chunk",
                {
                    "name": "action.robot.joint_position",
                    "frames": [{"joint_positions": [0.1, 0.2]}],
                    "frame_rate": 50,
                },
                timeout_ms=1000,
            )
        finally:
            conn.close()
    finally:
        server.stop()

    runtime = created["runtime"]
    assert runtime.started is True
    assert runtime.stopped is True
    assert manifest.payload["robot_id"] == "fake_runtime"
    assert manifest.payload["robot_name"] == "fake runtime"
    assert manifest.payload["capabilities"]["observations"] is True
    assert manifest.payload["capabilities"]["actions"] is True
    assert observation.payload["observations"][0]["value"] == {"joint_positions": [1.0, 2.0, 3.0]}
    assert action.payload == {"name": "action.robot.joint_position", "accepted_frames": 1, "frame_rate": 50}


def test_rynnrcp_server_streams_observations_over_grpc() -> None:
    created: dict[str, FakeRuntime] = {}
    config = _fake_server_config("stream_runtime")
    config["server"]["interface"]["local_registry"] = False

    def runtime_factory(config_arg: object) -> FakeRuntime:
        runtime = FakeRuntime(config_arg)  # type: ignore[arg-type]
        created["runtime"] = runtime
        return runtime

    server = RynnRCPServer(
        config=config,
        runtime_factory=runtime_factory,
    )
    server.start()
    try:
        conn = ClientInterface.with_defaults().connect(f"127.0.0.1:{server.bound_port}")
        try:
            client = RcpProtocolClient(conn)
            stream = client.subscribe_observations(["observation.robot.joint_state"], stream_hz=30)
            first = next(stream)
            second = next(stream)
            stream.cancel()
        finally:
            conn.close()
    finally:
        server.stop()

    assert first.ok is True
    assert second.ok is True
    assert first.payload["observations"][0]["value"] == {"joint_positions": [1.0, 2.0, 3.0]}
    assert second.payload["observations"][0]["value"] == {"joint_positions": [1.0, 2.0, 3.0]}
    assert second.metadata["server_stream_timing_ms"]["sequence"] == 1


def test_rynnrcp_server_streams_health_over_grpc() -> None:
    config = _fake_server_config("health_stream_runtime")
    config["server"]["interface"]["local_registry"] = False

    server = RynnRCPServer(config=config, runtime_factory=lambda config_arg: FakeRuntime(config_arg))  # type: ignore[arg-type]
    server.start()
    try:
        conn = ClientInterface.with_defaults().connect(f"127.0.0.1:{server.bound_port}")
        try:
            client = RcpProtocolClient(conn)
            stream = client.subscribe_health(stream_hz=30)
            first = next(stream)
            second = next(stream)
            stream.cancel()
        finally:
            conn.close()
    finally:
        server.stop()

    assert first.ok is True
    assert second.ok is True
    assert second.payload == {"errors": [], "warnings": []}
    assert second.metadata["server_stream_timing_ms"]["sequence"] == 1


def test_rynnrcp_server_streams_async_action_response_over_grpc() -> None:
    config = _fake_server_config("action_stream_runtime")
    config["server"]["interface"]["local_registry"] = False

    server = RynnRCPServer(config=config, runtime_factory=lambda config_arg: FakeRuntime(config_arg))  # type: ignore[arg-type]
    server.start()
    try:
        conn = ClientInterface.with_defaults().connect(f"127.0.0.1:{server.bound_port}")
        try:
            client = RcpProtocolClient(conn)
            stream = client.run_action_chunk_async(
                "action.robot.joint_position",
                [{"joint_positions": [0.1, 0.2]}],
                frame_rate=50,
            )
            response = next(stream)
            stream.cancel()
        finally:
            conn.close()
    finally:
        server.stop()

    assert response.ok is True
    assert response.payload == {"name": "action.robot.joint_position", "accepted_frames": 1, "frame_rate": 50.0}
    assert response.metadata["server_stream_timing_ms"]["stream_once"] is True


def test_rynnrcp_server_rejects_duplicate_live_server_id(monkeypatch: pytest.MonkeyPatch, tmp_path) -> None:
    monkeypatch.setenv(REGISTRY_ENV, str(tmp_path / "registry"))
    configs = [_fake_server_config("duplicate_server"), _fake_server_config("duplicate_server")]
    created: list[FakeRuntime] = []

    def runtime_factory(config_arg: object) -> FakeRuntime:
        runtime = FakeRuntime(config_arg)  # type: ignore[arg-type]
        created.append(runtime)
        return runtime

    first = RynnRCPServer(config=configs[0], runtime_factory=runtime_factory)
    first.start()
    try:
        second = RynnRCPServer(config=configs[1], runtime_factory=runtime_factory)
        with pytest.raises(RuntimeError, match="robot_id 'duplicate_server' is already online"):
            second.start()
    finally:
        first.stop()

    assert created[0].started is True
    assert created[1].started is False


def _fake_server_config(server_id: str) -> dict[str, Any]:
    return {
        "config_type": "rynnrcp_server_config",
        "version": 1,
            "manifest": {
                "robot_id": server_id,
                "robot_name": server_id,
                "capabilities": {
                    "observations": True,
                    "actions": True,
                    "health": True,
                    "resources": True,
                    "data_collection": False,
                    "policy_service": False,
                },
            },
            "server": {
                "config_name": "so101",
                "interface": {
                    "host": "127.0.0.1",
                    "port": 0,
                    "local_registry": True,
                    "mdns": False,
                },
            },
            "integration": {"config": MOCK_INTEGRATION_PATH},
            "components": _fake_components(),
        }


def _fake_components() -> dict[str, Any]:
    return {
        "robot": {
            "enabled": True,
            "state_values": [1.0, 2.0, 3.0, 4.0, 5.0, 0.25],
            "sys_path": [str(Path(__file__).parent)],
        },
        "mock_camera": {
            "enabled": False,
            "image_width": 8,
            "image_height": 4,
            "image_fps": 30.0,
            "sys_path": [str(Path(__file__).parent)],
        },
    }
