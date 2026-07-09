import os
import sys
import time
import multiprocessing as mp

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rynnrcp.native import WaitResult, create_notifier, open_notifier
import rynnrcp.native.notifier as notifier_module


def _wait_for_named_notifier(name: str, queue) -> None:
    opener = open_notifier(name)
    try:
        queue.put(("opened", None))
        queue.put(("wait", opener.wait(timeout_ms=1000).name))
    finally:
        opener.close()


def test_named_notifier_open_notify_wait():
    name = f"test_notifier_{os.getpid()}"
    creator = create_notifier(name)
    opener = open_notifier(name)
    try:
        assert opener.wait(timeout_ms=0) == WaitResult.TIMEOUT
        assert creator.notify()
        assert opener.wait(timeout_ms=500) == WaitResult.SIGNALED
    finally:
        opener.close()
        creator.close()
        creator.unlink()


def test_named_notifier_timeout():
    name = f"test_notifier_timeout_{os.getpid()}"
    notifier = create_notifier(name)
    try:
        start = time.monotonic()
        assert notifier.wait(timeout_ms=30) == WaitResult.TIMEOUT
        assert (time.monotonic() - start) >= 0.02
    finally:
        notifier.close()
        notifier.unlink()


def test_posix_fallback_poll_interval_defaults_to_five_ms(monkeypatch):
    monkeypatch.delenv("RYNNRCP_NOTIFIER_FALLBACK_POLL_MS", raising=False)

    assert notifier_module._resolve_fallback_poll_interval_s() == 0.005


def test_posix_fallback_poll_interval_is_configurable(monkeypatch):
    monkeypatch.setenv("RYNNRCP_NOTIFIER_FALLBACK_POLL_MS", "12")

    assert notifier_module._resolve_fallback_poll_interval_s() == 0.012


def test_posix_waiter_thread_handles_missing_sem_timedwait():
    if os.name == "nt":
        return
    name = f"test_notifier_waiter_thread_{os.getpid()}"
    notifier = create_notifier(name)
    try:
        notifier._sem_timedwait = None
        assert notifier.wait(timeout_ms=0) == WaitResult.TIMEOUT
        assert notifier.notify()
        assert notifier.wait(timeout_ms=500) == WaitResult.SIGNALED
        assert notifier.wait(timeout_ms=1) == WaitResult.TIMEOUT
    finally:
        notifier.close()
        notifier.unlink()


def test_named_notifier_cross_process_notify_wait():
    name = f"test_notifier_cross_process_{os.getpid()}"
    creator = create_notifier(name)
    ctx = mp.get_context("fork" if "fork" in mp.get_all_start_methods() else "spawn")
    queue = ctx.Queue()
    proc = ctx.Process(target=_wait_for_named_notifier, args=(name, queue))
    proc.start()
    try:
        assert queue.get(timeout=2) == ("opened", None)
        time.sleep(0.05)
        assert creator.notify()
        assert queue.get(timeout=2) == ("wait", "SIGNALED")
        proc.join(timeout=2)
        assert proc.exitcode == 0
    finally:
        if proc.is_alive():
            proc.terminate()
            proc.join(timeout=1)
        creator.close()
        creator.unlink()
