from __future__ import annotations

import json
import socket
from typing import Any
from urllib.request import urlopen

from rynnrcp.runtime.tool_bus import ToolBus
from rynnrcp.visualization.server import VisualizationServer


class FakeRuntime:
    def __init__(self) -> None:
        self.bus = ToolBus()
        self.action_service = FakeActionService()
        self.services: dict[str, Any] = {"action_service": self.action_service}
        self.manifest_reads = 0
        self.observation_reads = 0
        self.bus.add_tool("get_manifest", self.get_manifest)
        self.bus.add_tool("get_observations", self.get_observations)

    def get_manifest(self) -> dict[str, Any]:
        self.manifest_reads += 1
        return {
            "robot_id": "visual-test",
            "robot_name": "Visual Test",
            "observations": [{"name": "observation.robot.joint_state", "type": "joint_state"}],
            "actions": [],
        }

    def get_observations(self, names: list[str], sync: bool = False) -> dict[str, Any]:
        self.observation_reads += 1
        return ToolBus.make_result(
            True,
            result={
                "observations": [
                    {"name": name, "timestamp": 1.0, "value": {"joint_positions": [1.0, 2.0]}}
                    for name in names
                ]
            },
        )


class FakeActionService:
    def __init__(self) -> None:
        self.enabled = False
        self.latest_actions: dict[str, Any] = {}

    def set_latest_action_capture(self, enabled: bool) -> None:
        self.enabled = bool(enabled)
        if not enabled:
            self.latest_actions.clear()


def test_idle_server_does_not_read_runtime_until_requested() -> None:
    runtime = FakeRuntime()
    server = VisualizationServer(runtime, port=0)
    server.start()
    try:
        assert runtime.manifest_reads == 0
        assert runtime.observation_reads == 0
        assert runtime.action_service.enabled is False

        schema = _get_json(f"{server.url}api/schema")
        assert schema["result"]["robot_id"] == "visual-test"
        assert runtime.manifest_reads == 1
        assert runtime.observation_reads == 0
        assert runtime.action_service.enabled is False

        snapshot = _get_json(
            f"{server.url}api/snapshot?name=observation.robot.joint_state"
        )
        assert snapshot["result"]["observations"][0]["value"]["joint_positions"] == [1.0, 2.0]
        assert runtime.manifest_reads == 1
        assert runtime.observation_reads == 1
        assert runtime.action_service.enabled is True
    finally:
        server.stop()
    assert runtime.action_service.enabled is False


def test_preferred_port_falls_back_to_next_available(caplog) -> None:
    blocker = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    blocker.bind(("127.0.0.1", 0))
    blocker.listen(1)
    preferred_port = int(blocker.getsockname()[1])
    server = VisualizationServer(FakeRuntime(), port=preferred_port)
    try:
        server.start()
        assert server.bound_port != preferred_port
        assert server.bound_port > preferred_port
        assert "is unavailable" in caplog.text
    finally:
        server.stop()
        blocker.close()


def test_dashboard_formats_state_and_action_values_to_three_decimals() -> None:
    server = VisualizationServer(FakeRuntime(), port=0)
    server.start()
    try:
        with urlopen(server.url, timeout=2.0) as response:
            html = response.read().decode("utf-8")

        assert "function fixed(value)" in html
        assert "rounded.toFixed(3)" in html
        assert 'class="vector-cell"' in html
        assert 'class="value-view"' in html
        assert "valueHtml(v)" in html
        assert "valueHtml(v.value)" in html
    finally:
        server.stop()


def _get_json(url: str) -> dict[str, Any]:
    with urlopen(url, timeout=2.0) as response:
        return json.loads(response.read().decode("utf-8"))
