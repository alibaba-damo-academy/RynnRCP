# Copyright 2026 RynnRCP Authors. All rights reserved.
# Tests for rynnrcp.ipc.transport

import sys
import os
import threading
import time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rynnrcp.ipc.transport import (
    TransportLevel, IntraProcessTransport, ShmTransport, _subscriber_notifier_name,
)
import rynnrcp.ipc.transport as transport_module
from rynnrcp.native import NotifierUnavailable
from rynnrcp.native.shm import shm_unlink


def shm_cleanup(name):
    shm_unlink(f"rc_transport_{name}")


# ===========================================================================
# IntraProcessTransport tests
# ===========================================================================

def test_intra_basic_pubsub():
    t = IntraProcessTransport("test_intra", buffer_size=16)
    received = []
    t.subscribe(lambda data: received.append(data))

    t.publish(b"msg1")
    t.publish(b"msg2")

    assert len(received) == 2
    assert received[0] == b"msg1"
    assert received[1] == b"msg2"
    t.close()
    print("[PASS] test_intra_basic_pubsub")


def test_intra_poll():
    t = IntraProcessTransport("test_intra_poll", buffer_size=16)

    t.publish(b"data_a")
    t.publish(b"data_b")

    assert t.pending_count == 2
    msg = t.poll(timeout_ms=100)
    assert msg == b"data_a"
    msg = t.poll(timeout_ms=100)
    assert msg == b"data_b"
    msg = t.poll(timeout_ms=50)
    assert msg is None  # no more data
    t.close()
    print("[PASS] test_intra_poll")


def test_intra_latest():
    t = IntraProcessTransport("test_intra_latest", buffer_size=8)
    for i in range(5):
        t.publish(f"v{i}".encode())
    assert t.latest() == b"v4"
    t.close()
    print("[PASS] test_intra_latest")


def test_intra_cross_thread():
    t = IntraProcessTransport("test_intra_xt", buffer_size=64)
    received = []

    def subscriber():
        for _ in range(10):
            msg = t.poll(timeout_ms=2000)
            if msg is not None:
                received.append(msg)

    sub_thread = threading.Thread(target=subscriber)
    sub_thread.start()

    time.sleep(0.02)
    for i in range(10):
        t.publish(f"x{i}".encode())
        time.sleep(0.005)

    sub_thread.join(timeout=5)
    assert len(received) == 10, f"Got {len(received)}"
    t.close()
    print("[PASS] test_intra_cross_thread")


def test_intra_multiple_subscribers():
    t = IntraProcessTransport("test_intra_multi")
    r1, r2 = [], []
    t.subscribe(lambda d: r1.append(d))
    t.subscribe(lambda d: r2.append(d))

    t.publish(b"broadcast")
    assert len(r1) == 1 and r1[0] == b"broadcast"
    assert len(r2) == 1 and r2[0] == b"broadcast"
    t.close()
    print("[PASS] test_intra_multiple_subscribers")


def test_intra_level():
    t = IntraProcessTransport("test_level")
    assert t.level == TransportLevel.L1_INTRA_PROCESS
    t.close()
    print("[PASS] test_intra_level")


# ===========================================================================
# ShmTransport tests
# ===========================================================================

def test_shm_basic_pubsub():
    name = "test_shm_ps"
    shm_cleanup(name)
    t = ShmTransport(name, msg_size=32, slot_count=8, create=True)
    received = []
    t.subscribe(lambda d: received.append(d))

    t.publish(b"shm_msg_1" + b'\x00' * 23)
    assert len(received) == 1
    assert received[0][:9] == b"shm_msg_1"
    t.close()
    t.unlink()
    print("[PASS] test_shm_basic_pubsub")


def test_shm_poll():
    name = "test_shm_poll"
    shm_cleanup(name)
    t = ShmTransport(name, msg_size=16, slot_count=8, create=True)

    for i in range(5):
        t.publish(i.to_bytes(8, "little") + b'\x00' * 8)

    # Poll sequentially
    for i in range(5):
        data = t.poll(timeout_ms=100)
        assert data is not None
        val = int.from_bytes(data[:8], "little")
        assert val == i, f"Expected {i}, got {val}"

    # No more data
    assert t.poll(timeout_ms=50) is None
    t.close()
    t.unlink()
    print("[PASS] test_shm_poll")


def test_shm_read_latest():
    name = "test_shm_latest"
    shm_cleanup(name)
    t = ShmTransport(name, msg_size=8, slot_count=4, create=True)

    for i in range(10):
        t.publish(i.to_bytes(8, "little"))

    data = t.read_latest()
    assert data is not None
    val = int.from_bytes(data, "little")
    assert val == 9
    t.close()
    t.unlink()
    print("[PASS] test_shm_read_latest")


def test_shm_payload_may_end_with_zero_bytes():
    name = "test_shm_zero_tail"
    shm_cleanup(name)
    t = ShmTransport(name, msg_size=32, slot_count=4, create=True)

    payload = b"binary\x00\x00"
    t.publish(payload)
    assert t.poll(timeout_ms=100) == payload

    t.close()
    t.unlink()
    print("[PASS] test_shm_payload_may_end_with_zero_bytes")


def test_shm_publish_parts_roundtrip():
    name = "test_shm_parts"
    shm_cleanup(name)
    t = ShmTransport(name, msg_size=32, slot_count=4, create=True)

    t.publish_parts((b"ab", memoryview(b"cd"), bytearray(b"ef")))
    assert t.poll(timeout_ms=100) == b"abcdef"

    t.close()
    t.unlink()
    print("[PASS] test_shm_publish_parts_roundtrip")


def test_shm_notifier_wakes_opened_subscriber():
    name = "test_shm_notify"
    shm_cleanup(name)
    pub = ShmTransport(name, msg_size=32, slot_count=8, create=True)
    sub = ShmTransport(name, msg_size=32, slot_count=8, create=False)

    received = []

    def reader():
        received.append(sub.poll(timeout_ms=1000))

    rt = threading.Thread(target=reader)
    rt.start()
    time.sleep(0.02)
    pub.publish(b"wake_by_notifier")
    rt.join(timeout=2)

    assert received == [b"wake_by_notifier"]
    sub.close()
    pub.close()
    pub.unlink()
    print("[PASS] test_shm_notifier_wakes_opened_subscriber")


def test_shm_notifier_wait_uses_caller_timeout():
    class FakeNotifier:
        def __init__(self):
            self.waits = []

        def wait(self, timeout_ms=-1):
            self.waits.append(timeout_ms)
            return transport_module.WaitResult.TIMEOUT

        def drain(self):
            pass

    notifier = FakeNotifier()
    transport = ShmTransport.__new__(ShmTransport)
    transport._name = "test_shm_wait_timeout"
    transport._notify_mode = "hybrid"
    transport._subscriber_notifier = None
    transport._channel_notifier = notifier
    transport._poll_interval = 0.001

    transport._wait_for_cross_process_signal(time.monotonic() + 0.25)

    assert notifier.waits
    assert notifier.waits[0] > 100
    print("[PASS] test_shm_notifier_wait_uses_caller_timeout")


def test_shm_prunes_stale_subscriber_notifier():
    name = "test_shm_stale_notify"
    shm_cleanup(name)

    class Entry:
        channel_name = name
        subscriber_id = "stale-sub"
        pid = 12345
        notifier_name = "missing-notifier"

    class Registry:
        def __init__(self):
            self.entries = [Entry()]
            self.unregistered = []

        def list_subscribers(self, channel_name):
            assert channel_name == name
            return list(self.entries)

        def unregister_subscriber(self, channel_name, subscriber_id, pid):
            self.unregistered.append((channel_name, subscriber_id, pid))
            self.entries = []

    registry = Registry()
    original_open_notifier = transport_module.open_notifier
    transport_module.open_notifier = lambda _name: (_ for _ in ()).throw(
        NotifierUnavailable("missing for test")
    )
    transport = ShmTransport.__new__(ShmTransport)
    transport._name = name
    transport._registry = registry
    transport._publisher_notifiers = {}
    transport._publisher_notifier_failures = set()
    try:
        transport._refresh_publisher_notifiers()
        transport._refresh_publisher_notifiers()
        assert registry.unregistered == [(name, "stale-sub", 12345)]
        assert transport._publisher_notifiers == {}
    finally:
        transport_module.open_notifier = original_open_notifier
    print("[PASS] test_shm_prunes_stale_subscriber_notifier")


def test_shm_notifier_refresh_uses_subscriber_version():
    name = "test_shm_notify_version"

    class Registry:
        def __init__(self):
            self.version = 1
            self.list_calls = 0

        def subscriber_version(self):
            return self.version

        def list_subscribers(self, channel_name):
            assert channel_name == name
            self.list_calls += 1
            return []

    registry = Registry()
    transport = ShmTransport.__new__(ShmTransport)
    transport._name = name
    transport._registry = registry
    transport._notify_mode = "hybrid"
    transport._channel_notifier = None
    transport._publisher_notifiers = {}
    transport._publisher_notifier_failures = set()
    transport._publisher_notifier_refresh_at = time.monotonic() + 100.0
    transport._publisher_notifier_registry_version = None

    transport._notify_waiters()
    transport._notify_waiters()
    assert registry.list_calls == 1

    registry.version = 2
    transport._notify_waiters()
    assert registry.list_calls == 2
    print("[PASS] test_shm_notifier_refresh_uses_subscriber_version")


def test_shm_subscriber_notifier_name_fits_registry_field():
    channel_name = "/sensor/so101_follower/images/wrist"
    subscriber_id = "57305-7a090e0a63e3"
    name = _subscriber_notifier_name(channel_name, subscriber_id)

    assert name.startswith("rc_sub_")
    assert len(name.encode("utf-8")) < 56
    assert channel_name not in name
    print("[PASS] test_shm_subscriber_notifier_name_fits_registry_field")


def test_shm_cross_thread():
    name = "test_shm_xt"
    shm_cleanup(name)
    pub = ShmTransport(name, msg_size=16, slot_count=32, create=True)
    # Simulate separate "reader" using same SHM
    sub = ShmTransport(name, msg_size=16, slot_count=32, create=False)

    num_msgs = 20
    received = []

    def reader():
        for _ in range(num_msgs):
            data = sub.poll(timeout_ms=2000)
            if data is not None:
                received.append(int.from_bytes(data[:8], "little"))

    rt = threading.Thread(target=reader)
    rt.start()

    time.sleep(0.02)
    for i in range(num_msgs):
        pub.publish(i.to_bytes(8, "little") + b'\x00' * 8)
        time.sleep(0.005)

    rt.join(timeout=10)
    assert len(received) == num_msgs, f"Got {len(received)} of {num_msgs}"
    sub.close()
    pub.close()
    pub.unlink()
    print("[PASS] test_shm_cross_thread")


def test_shm_level():
    name = "test_shm_lv"
    shm_cleanup(name)
    t = ShmTransport(name, msg_size=8, slot_count=4, create=True)
    assert t.level == TransportLevel.L2_SHM
    t.close()
    t.unlink()
    print("[PASS] test_shm_level")


if __name__ == "__main__":
    print("=== transport tests ===")
    # IntraProcess
    test_intra_basic_pubsub()
    test_intra_poll()
    test_intra_latest()
    test_intra_cross_thread()
    test_intra_multiple_subscribers()
    test_intra_level()
    # SHM
    test_shm_basic_pubsub()
    test_shm_poll()
    test_shm_read_latest()
    test_shm_payload_may_end_with_zero_bytes()
    test_shm_notifier_wakes_opened_subscriber()
    test_shm_notifier_wait_uses_caller_timeout()
    test_shm_notifier_refresh_uses_subscriber_version()
    test_shm_prunes_stale_subscriber_notifier()
    test_shm_subscriber_notifier_name_fits_registry_field()
    test_shm_cross_thread()
    test_shm_level()
    print("=== All transport tests passed ===")
