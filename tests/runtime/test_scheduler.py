# Copyright 2026 RynnRCP Authors. All rights reserved.
# Tests for rynnrcp.runtime.scheduler

import sys
import os
import threading
import time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rynnrcp.runtime.scheduler import Scheduler, Component


def test_add_component():
    s = Scheduler()
    counter = [0]
    comp = Component("test_comp", period_ms=10, callback=lambda: None)
    s.add_component(comp)

    assert s.component_count == 1
    assert "test_comp" in s.list_components()

    # Duplicate name should fail
    try:
        s.add_component(Component("test_comp", period_ms=10, callback=lambda: None))
        assert False, "Should have raised"
    except ValueError:
        pass

    print("[PASS] test_add_component")


def test_remove_component():
    s = Scheduler()
    s.add_component(Component("rem", period_ms=10, callback=lambda: None))
    assert s.component_count == 1
    s.remove_component("rem")
    assert s.component_count == 0
    print("[PASS] test_remove_component")


def test_spin_once():
    s = Scheduler()
    counters = {"a": 0, "b": 0}

    s.add_component(Component("a", period_ms=10, callback=lambda: counters.__setitem__("a", counters["a"] + 1)))
    s.add_component(Component("b", period_ms=10, callback=lambda: counters.__setitem__("b", counters["b"] + 1)))

    s.spin_once()
    assert counters["a"] == 1
    assert counters["b"] == 1

    s.spin_once()
    assert counters["a"] == 2
    assert counters["b"] == 2

    print("[PASS] test_spin_once")


def test_spin_once_priority():
    """Higher priority components should execute first in spin_once."""
    s = Scheduler()
    order = []

    s.add_component(Component("low", period_ms=10, priority=0,
                               callback=lambda: order.append("low")))
    s.add_component(Component("high", period_ms=10, priority=10,
                               callback=lambda: order.append("high")))
    s.add_component(Component("mid", period_ms=10, priority=5,
                               callback=lambda: order.append("mid")))

    s.spin_once()
    assert order == ["high", "mid", "low"], f"Got {order}"
    print("[PASS] test_spin_once_priority")


def test_start_stop():
    s = Scheduler()
    counter = [0]
    lock = threading.Lock()

    def increment():
        with lock:
            counter[0] += 1

    s.add_component(Component("ticker", period_ms=10, callback=increment))
    assert not s.is_running

    s.start()
    assert s.is_running
    time.sleep(0.15)  # ~15 ticks at 10ms
    s.stop()
    assert not s.is_running

    with lock:
        final = counter[0]
    # Should have run approximately 15 times (allow tolerance)
    assert final >= 8, f"Only ran {final} times"
    assert final <= 30, f"Ran too many times: {final}"
    print(f"[PASS] test_start_stop (ran {final} times in ~150ms)")


def test_multiple_components():
    s = Scheduler()
    counters = {"fast": 0, "slow": 0}
    lock = threading.Lock()

    def inc(name):
        with lock:
            counters[name] += 1

    s.add_component(Component("fast", period_ms=10, callback=lambda: inc("fast")))
    s.add_component(Component("slow", period_ms=50, callback=lambda: inc("slow")))

    s.start()
    time.sleep(0.2)
    s.stop()

    with lock:
        fast_count = counters["fast"]
        slow_count = counters["slow"]

    # fast (~10ms) should run ~20 times, slow (~50ms) should run ~4 times
    assert fast_count >= 10, f"fast only ran {fast_count}"
    assert slow_count >= 2, f"slow only ran {slow_count}"
    assert fast_count > slow_count, f"fast={fast_count} should be > slow={slow_count}"
    print(f"[PASS] test_multiple_components (fast={fast_count}, slow={slow_count})")


def test_periodic_execution_runs_repeatedly():
    s = Scheduler()
    counter = [0]
    comp = Component("periodic_test", period_ms=20, callback=lambda: counter.__setitem__(0, counter[0] + 1))
    s.add_component(comp)

    s.start()
    time.sleep(0.25)  # ~12 runs at 20ms
    s.stop()

    assert counter[0] >= 5, f"Only {counter[0]} runs"
    print(f"[PASS] test_periodic_execution_runs_repeatedly (runs={counter[0]})")


def test_slight_overrun_does_not_skip_full_period():
    s = Scheduler()
    counter = [0]

    def slight_overrun():
        counter[0] += 1
        time.sleep(0.021)

    comp = Component("slight_overrun", period_ms=20, callback=slight_overrun)
    s.add_component(comp)

    s.start()
    time.sleep(0.16)
    s.stop()

    assert counter[0] >= 4, f"Only ran {counter[0]} times"
    print(f"[PASS] test_slight_overrun_does_not_skip_full_period (runs={counter[0]})")


def test_enable_disable():
    s = Scheduler()
    counter = [0]
    lock = threading.Lock()

    comp = Component("toggle", period_ms=10, callback=lambda: lock.__enter__() or counter.__setitem__(0, counter[0] + 1) or lock.__exit__(None, None, None))

    # Simpler approach
    counter2 = [0]
    def inc():
        counter2[0] += 1

    comp2 = Component("toggle2", period_ms=10, callback=inc)
    s.add_component(comp2)

    s.start()
    time.sleep(0.08)

    comp2.enabled = False
    count_at_disable = counter2[0]
    time.sleep(0.08)
    count_after_disable = counter2[0]

    # Should not have changed much (maybe 0-1 more due to race)
    assert count_after_disable - count_at_disable <= 1, \
        f"Ran {count_after_disable - count_at_disable} times while disabled"

    comp2.enabled = True
    time.sleep(0.08)
    s.stop()

    # Should have resumed
    assert counter2[0] > count_after_disable
    print(f"[PASS] test_enable_disable (disabled at {count_at_disable}, "
          f"after disable {count_after_disable}, final {counter2[0]})")


def test_add_while_running():
    s = Scheduler()
    s.add_component(Component("initial", period_ms=10, callback=lambda: None))
    s.start()

    counter = [0]
    def inc():
        counter[0] += 1

    s.add_component(Component("dynamic", period_ms=10, callback=inc))
    time.sleep(0.1)
    s.stop()

    assert counter[0] >= 3, f"Dynamic component only ran {counter[0]} times"
    print(f"[PASS] test_add_while_running (dynamic ran {counter[0]} times)")


def test_callback_error_handling(caplog):
    s = Scheduler()

    def bad_callback():
        raise RuntimeError("intentional error")

    s.add_component(Component("bad", period_ms=10, callback=bad_callback))
    s.start()
    time.sleep(0.05)
    s.stop()
    # Should not crash the scheduler
    assert "intentional error" in caplog.text
    assert not s.is_running
    print("[PASS] test_callback_error_handling")


def test_stop_returns_when_callback_is_blocked():
    s = Scheduler()
    release = threading.Event()
    entered = threading.Event()

    def blocked():
        entered.set()
        release.wait(timeout=1.0)

    s.add_component(Component("blocked", period_ms=10, callback=blocked))
    s.start()
    assert entered.wait(timeout=0.2)

    start = time.monotonic()
    s.stop()
    elapsed = time.monotonic() - start
    release.set()

    assert elapsed < 0.5
    assert not s.is_running


if __name__ == "__main__":
    print("=== scheduler tests ===")
    test_add_component()
    test_remove_component()
    test_spin_once()
    test_spin_once_priority()
    test_start_stop()
    test_multiple_components()
    test_periodic_execution_runs_repeatedly()
    test_slight_overrun_does_not_skip_full_period()
    test_enable_disable()
    test_add_while_running()
    test_callback_error_handling()
    print("=== All scheduler tests passed ===")
