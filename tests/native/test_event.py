# Copyright 2026 RynnRCP Authors. All rights reserved.
# Tests for rynnrcp.native.event

import sys
import os
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rynnrcp.native.event import Event, WaitResult


def test_create_close():
    ev = Event()
    assert ev.is_valid()
    ev.close()
    assert not ev.is_valid()
    print("[PASS] test_create_close")


def test_signal_wait():
    ev = Event()
    assert ev.is_valid()

    ok = ev.signal()
    assert ok

    result = ev.wait(timeout_ms=1000)
    assert result == WaitResult.SIGNALED

    ev.close()
    print("[PASS] test_signal_wait")


def test_timeout():
    ev = Event()
    assert ev.is_valid()

    start = time.monotonic()
    result = ev.wait(timeout_ms=50)
    elapsed_ms = (time.monotonic() - start) * 1000

    assert result == WaitResult.TIMEOUT
    assert elapsed_ms >= 30, f"Timeout too fast: {elapsed_ms:.1f}ms"

    ev.close()
    print(f"[PASS] test_timeout (waited {elapsed_ms:.0f}ms)")


def test_nonblocking_poll():
    ev = Event()
    assert ev.is_valid()

    # No signal -> timeout
    result = ev.wait(timeout_ms=0)
    assert result == WaitResult.TIMEOUT

    # Signal then poll -> signaled
    ev.signal()
    result = ev.wait(timeout_ms=0)
    assert result == WaitResult.SIGNALED

    ev.close()
    print("[PASS] test_nonblocking_poll")


def test_cross_thread_signal():
    ev = Event()
    received = [False]

    def waiter():
        result = ev.wait(timeout_ms=5000)
        if result == WaitResult.SIGNALED:
            received[0] = True

    t = threading.Thread(target=waiter)
    t.start()

    # Give waiter time to enter wait
    time.sleep(0.05)
    ok = ev.signal()
    assert ok

    t.join(timeout=5)
    assert received[0], "Waiter did not receive signal"

    ev.close()
    print("[PASS] test_cross_thread_signal")


def test_multiple_signals():
    ev = Event()
    for _ in range(5):
        ev.signal()

    result = ev.wait(timeout_ms=100)
    assert result == WaitResult.SIGNALED

    ev.close()
    print("[PASS] test_multiple_signals")


def test_closed_event():
    ev = Event()
    ev.close()

    assert not ev.signal()
    result = ev.wait(timeout_ms=0)
    assert result == WaitResult.ERROR
    print("[PASS] test_closed_event")


def test_auto_reset():
    """After wait consumes the signal, subsequent wait should timeout."""
    ev = Event()
    ev.signal()

    result = ev.wait(timeout_ms=100)
    assert result == WaitResult.SIGNALED

    # Signal was consumed, next wait should timeout
    result = ev.wait(timeout_ms=50)
    assert result == WaitResult.TIMEOUT

    ev.close()
    print("[PASS] test_auto_reset")


if __name__ == "__main__":
    print("=== event tests ===")
    test_create_close()
    test_signal_wait()
    test_timeout()
    test_nonblocking_poll()
    test_cross_thread_signal()
    test_multiple_signals()
    test_closed_event()
    test_auto_reset()
    print("=== All event tests passed ===")
