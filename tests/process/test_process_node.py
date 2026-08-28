"""Tests for ProcessNode lifecycle handling (run in-process with stubs)."""

from __future__ import annotations

import threading
import types
from typing import Any

import pytest

import rynnrcp.process.process_node as process_node_mod
from rynnrcp.process.process_node import (
    ProcessNode,
    _collect_lifecycle_result,
    _send_lifecycle,
    _wait_for_shutdown,
)


class _Queue:
    def __init__(self) -> None:
        self.events: list[dict] = []

    def put(self, item: dict) -> None:
        self.events.append(item)


class _SetEvent:
    """Multiprocessing-style event that reports 'already set'."""

    def wait(self, timeout: float | None = None) -> bool:
        return True


@pytest.fixture(autouse=True)
def quiet_node(monkeypatch: pytest.MonkeyPatch):
    # Skip real logging reconfiguration and the parent-death watchdog thread.
    monkeypatch.setattr(
        process_node_mod, "_configure_process_logging", lambda *a, **k: None
    )
    monkeypatch.setattr(
        process_node_mod, "_start_parent_watchdog", lambda *a, **k: None
    )


def test_run_executes_components_and_reports_lifecycle() -> None:
    node = ProcessNode("worker", registry_attached=False)
    queue = _Queue()
    stop_event = threading.Event()
    cleanups: list[str] = []

    def component_a(mgr: Any, config: dict) -> threading.Event:
        assert mgr is not None
        return stop_event

    def component_b(mgr: Any, config: dict):
        return lambda: cleanups.append("b")

    node.add_component(component_a)
    node.add_component(component_b)
    node.run(lifecycle_queue=queue, shutdown_event=_SetEvent())

    statuses = [event["status"] for event in queue.events]
    assert statuses == ["starting", "starting", "ready"]
    assert queue.events[0]["detail"] == "component_a"
    assert stop_event.is_set()
    assert cleanups == ["b"]
    assert node.running is False


def test_run_reports_component_failure() -> None:
    node = ProcessNode("worker", registry_attached=False)
    queue = _Queue()

    def broken(mgr: Any, config: dict) -> None:
        raise RuntimeError("setup exploded")

    node.add_component(broken)
    with pytest.raises(RuntimeError, match="setup exploded"):
        node.run(lifecycle_queue=queue, shutdown_event=_SetEvent())

    failed = queue.events[-1]
    assert failed["status"] == "failed"
    assert "setup exploded" in failed["error"]


def test_run_handles_keyboard_interrupt() -> None:
    node = ProcessNode("worker", registry_attached=False)
    queue = _Queue()

    def interrupt(mgr: Any, config: dict) -> None:
        raise KeyboardInterrupt

    node.add_component(interrupt)
    node.run(lifecycle_queue=queue, shutdown_event=_SetEvent())
    assert queue.events[-1]["status"] == "stopped"
    assert queue.events[-1]["detail"] == "KeyboardInterrupt"


def test_run_component_cleanup_error_is_swallowed() -> None:
    node = ProcessNode("worker", registry_attached=False)

    def component(mgr: Any, config: dict):
        def failing_cleanup() -> None:
            raise RuntimeError("cleanup boom")

        return failing_cleanup

    node.add_component(component)
    node.run(shutdown_event=_SetEvent())


def test_collect_lifecycle_result_variants() -> None:
    stop_events: list[threading.Event] = []
    cleanups: list = []

    _collect_lifecycle_result(None, stop_events, cleanups)
    assert (stop_events, cleanups) == ([], [])

    event = threading.Event()
    _collect_lifecycle_result(event, stop_events, cleanups)
    assert stop_events == [event]

    fn = lambda: None
    _collect_lifecycle_result(fn, stop_events, cleanups)
    assert cleanups == [fn]

    stoppable = types.SimpleNamespace(stop=lambda: None)
    _collect_lifecycle_result(stoppable, stop_events, cleanups)
    assert cleanups[-1] == stoppable.stop

    nested_event = threading.Event()
    nested_fn = lambda: None
    _collect_lifecycle_result((nested_event, nested_fn), stop_events, cleanups)
    assert nested_event in stop_events
    assert nested_fn in cleanups

    # Objects without stop() are ignored.
    _collect_lifecycle_result(object(), stop_events, cleanups)


def test_send_lifecycle_tolerates_broken_queue() -> None:
    class Broken:
        def put(self, item: dict) -> None:
            raise RuntimeError("queue closed")

    _send_lifecycle(Broken(), "n", "ready")
    _send_lifecycle(None, "n", "ready")


def test_wait_for_shutdown_paths() -> None:
    # Signal event already set → returns immediately without shutdown_event.
    signal_event = threading.Event()
    signal_event.set()
    _wait_for_shutdown(signal_event, None)

    # Shutdown event fires.
    signal_event = threading.Event()
    _wait_for_shutdown(signal_event, _SetEvent())

    # Broken shutdown event falls back to the signal event with a timeout.
    class Broken:
        def wait(self, timeout: float | None = None) -> bool:
            raise OSError("handle closed")

    signal_event = threading.Event()
    _wait_for_shutdown(signal_event, Broken())
