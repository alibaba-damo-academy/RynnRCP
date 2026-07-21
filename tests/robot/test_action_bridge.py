from __future__ import annotations

import importlib.util
from pathlib import Path


def _load_action_bridge():
    path = Path(__file__).parents[2] / "robots" / "sim_robot" / "action_bridge.py"
    spec = importlib.util.spec_from_file_location("sim_action_bridge", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _FakeRcpClient:
    def __init__(self) -> None:
        self.calls = []

    def run_action_chunk(self, **kwargs):
        self.calls.append(kwargs)


def test_dispatches_repeated_and_all_zero_action_frames() -> None:
    bridge = _load_action_bridge()
    client = _FakeRcpClient()
    positions = [0.0] * 6

    assert bridge._dispatch_action_frame(client, "action.robot.joint_position", positions, 30.0)
    assert bridge._dispatch_action_frame(client, "action.robot.joint_position", positions, 30.0)

    assert len(client.calls) == 2
    assert all(call["frames"] == [{"joint_positions": positions}] for call in client.calls)


def test_skips_only_missing_or_empty_action_frames() -> None:
    bridge = _load_action_bridge()
    client = _FakeRcpClient()

    assert not bridge._dispatch_action_frame(client, "action.robot.joint_position", None, 30.0)
    assert not bridge._dispatch_action_frame(client, "action.robot.joint_position", [], 30.0)
    assert client.calls == []
