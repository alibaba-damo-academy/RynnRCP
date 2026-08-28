from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path


def _load_action_bridge():
    if "zmq" not in sys.modules:
        try:
            import zmq  # noqa: F401
        except ModuleNotFoundError:
            sys.modules["zmq"] = types.ModuleType("zmq")
    path = Path(__file__).parents[2] / "robots" / "sim_robot" / "action_bridge.py"
    spec = importlib.util.spec_from_file_location("sim_action_bridge", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _FakeRcpClient:
    def __init__(self, response=None) -> None:
        self.calls = []
        self.response = response

    def run_action_chunk(self, **kwargs):
        self.calls.append(kwargs)
        return self.response


class _FakeResponse:
    def __init__(self, *, ok: bool, status: int, message: str) -> None:
        self.ok = ok
        self.status = status
        self.message = message


class _FakeLogGate:
    def __init__(self) -> None:
        self.failures = []
        self.successes = 0

    def failure(self, message, *args, **kwargs) -> None:
        self.failures.append((message, args, kwargs))

    def success(self) -> None:
        self.successes += 1


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


def test_reports_rcp_rejection(caplog) -> None:
    bridge = _load_action_bridge()
    client = _FakeRcpClient(
        _FakeResponse(ok=False, status=2, message="Expected 8 joint values, got 9")
    )

    with caplog.at_level("ERROR"):
        accepted = bridge._dispatch_action_frame(
            client,
            "action.robot.joint_position",
            [0.0] * 9,
            30.0,
        )

    assert not accepted
    assert "RCP_REJECT" in caplog.text
    assert "Expected 8 joint values, got 9" in caplog.text


def test_receive_stats_expose_zero_frames_and_dimension_changes() -> None:
    bridge = _load_action_bridge()
    stats = {}

    first = bridge._update_action_receive_stats(
        stats,
        "robot",
        [0.0] * 9,
        log_interval=0.0,
    )
    second = bridge._update_action_receive_stats(
        stats,
        "robot",
        [1.0] * 8,
        log_interval=0.0,
    )

    assert first.total == 1
    assert first.dimension == 9
    assert first.all_zero
    assert first.window_zero_count == 1
    assert second.total == 2
    assert second.previous_dimension == 9
    assert second.dimension == 8
    assert not second.all_zero
    assert second.consecutive_zero == 0


def test_receive_stats_detect_changes_between_periodic_log_samples() -> None:
    bridge = _load_action_bridge()
    stats = {}

    bridge._update_action_receive_stats(
        stats,
        "robot",
        [0.0, 0.0],
        log_interval=100.0,
    )
    changed = bridge._update_action_receive_stats(
        stats,
        "robot",
        [0.5, -0.25],
        log_interval=100.0,
    )

    assert changed.window_count == 2
    assert changed.window_changed_count == 1
    assert changed.window_max_delta == 0.5
    assert changed.joint_ranges == (0.5, 0.25)


def test_missing_action_uses_rate_limited_availability_log() -> None:
    bridge = _load_action_bridge()
    poller = bridge.JointCommandPoller.__new__(bridge.JointCommandPoller)
    poller.robot_name = "fr3"
    poller._endpoint = "tcp://127.0.0.1:8081"
    poller._availability_log = _FakeLogGate()

    poller._record_missing_action("request timed out")

    assert len(poller._availability_log.failures) == 1
    _message, args, _kwargs = poller._availability_log.failures[0]
    assert args == ("fr3", "tcp://127.0.0.1:8081", "request timed out")


def test_configures_rotating_log_file_under_robot_root(
    monkeypatch, tmp_path
) -> None:
    bridge = _load_action_bridge()
    monkeypatch.setenv("RYNNRCP_HOME", str(tmp_path))
    monkeypatch.setenv("RYNNRCP_RUN_ID", "shared-sim-run")

    log_path = bridge._configure_action_bridge_logging("franka_r3_sim_v2")
    bridge.logger.info("action bridge file logging test")

    from rynnrcp.utils.logging import _stop_async_file_loggers, clear_log_context

    _stop_async_file_loggers()
    clear_log_context()
    bridge.logger.handlers.clear()
    bridge.logger.propagate = True

    expected = (
        tmp_path
        / "franka_r3_sim_v2"
        / "logs"
        / "action_bridge.log"
    )
    assert log_path == str(expected)
    assert expected.is_file()
    assert "action bridge file logging test" in expected.read_text(encoding="utf-8")
    assert "run_id=shared-sim-run" in expected.read_text(encoding="utf-8")
