# Copyright 2026 RynnRCP Authors. All rights reserved.
# Tests for rynnrcp.ipc.channel

import sys
import os
import threading
import time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rynnrcp.ipc.channel import ChannelManager, Publisher, Subscriber
from rynnrcp.ipc.transport import TransportLevel


def setup():
    ChannelManager.reset()


def test_create_publisher_subscriber():
    setup()
    mgr = ChannelManager.instance()

    pub = mgr.create_publisher("sensor_a", msg_size=32)
    assert isinstance(pub, Publisher)
    assert pub.channel_name == "sensor_a"

    received = []
    sub = mgr.create_subscriber("sensor_a", msg_size=32,
                                 callback=lambda d: received.append(d))
    assert isinstance(sub, Subscriber)
    assert sub.channel_name == "sensor_a"

    pub.publish(b"hello" + b'\x00' * 27)
    assert len(received) == 1
    assert received[0][:5] == b"hello"

    setup()
    print("[PASS] test_create_publisher_subscriber")


def test_multiple_subscribers():
    setup()
    mgr = ChannelManager.instance()

    pub = mgr.create_publisher("multi_ch", msg_size=16)
    r1, r2, r3 = [], [], []
    mgr.create_subscriber("multi_ch", msg_size=16, callback=lambda d: r1.append(d))
    mgr.create_subscriber("multi_ch", msg_size=16, callback=lambda d: r2.append(d))
    mgr.create_subscriber("multi_ch", msg_size=16, callback=lambda d: r3.append(d))

    pub.publish(b"broadcast_data!!")  # exactly 16 bytes
    assert len(r1) == 1 and len(r2) == 1 and len(r3) == 1
    assert r1[0] == r2[0] == r3[0] == b"broadcast_data!!"

    setup()
    print("[PASS] test_multiple_subscribers")


def test_multiple_polling_subscribers_receive_same_message():
    setup()
    mgr = ChannelManager.instance()

    pub = mgr.create_publisher("multi_poll", msg_size=16)
    sub1 = mgr.create_subscriber("multi_poll", msg_size=16)
    sub2 = mgr.create_subscriber("multi_poll", msg_size=16)

    pub.publish(b"poll_broadcast!")
    assert sub1.poll(timeout_ms=200) == b"poll_broadcast!"
    assert sub2.poll(timeout_ms=200) == b"poll_broadcast!"

    setup()
    print("[PASS] test_multiple_polling_subscribers_receive_same_message")


def test_spin_once():
    setup()
    mgr = ChannelManager.instance()

    pub = mgr.create_publisher("spin_ch", msg_size=8)
    received = []
    sub = mgr.create_subscriber("spin_ch", msg_size=8)

    # Publish in another thread
    def publisher():
        time.sleep(0.02)
        pub.publish(b"spindata")

    t = threading.Thread(target=publisher)
    t.start()

    # spin_once should eventually get the message
    got = sub.spin_once(timeout_ms=2000)
    t.join()

    assert got, "spin_once should have received data"
    setup()
    print("[PASS] test_spin_once")


def test_channel_info():
    setup()
    mgr = ChannelManager.instance()
    mgr.create_publisher("info_ch", msg_size=64)

    info = mgr.get_channel_info("info_ch")
    assert info is not None
    assert info["name"] == "info_ch"
    assert info["msg_size"] == 64
    assert "INTRA" in info["transport_level"]

    assert mgr.get_channel_info("nonexistent") is None

    setup()
    print("[PASS] test_channel_info")


def test_list_channels():
    setup()
    mgr = ChannelManager.instance()
    mgr.create_publisher("ch_a", msg_size=8)
    mgr.create_publisher("ch_b", msg_size=16)
    mgr.create_publisher("ch_c", msg_size=32)

    channels = mgr.list_channels()
    assert set(channels) == {"ch_a", "ch_b", "ch_c"}

    setup()
    print("[PASS] test_list_channels")


def test_msg_size_mismatch():
    setup()
    mgr = ChannelManager.instance()
    mgr.create_publisher("mismatch", msg_size=32)

    try:
        mgr.create_subscriber("mismatch", msg_size=64)
        assert False, "Should have raised ValueError"
    except ValueError:
        pass

    setup()
    print("[PASS] test_msg_size_mismatch")


def test_transport_mismatch():
    setup()
    mgr = ChannelManager.instance()
    mgr.create_publisher("transport_mismatch", msg_size=32,
                         transport=TransportLevel.L1_INTRA_PROCESS)

    try:
        mgr.create_subscriber("transport_mismatch", msg_size=32,
                              transport=TransportLevel.L2_SHM)
        assert False, "Should have raised ValueError"
    except ValueError:
        pass

    setup()
    print("[PASS] test_transport_mismatch")


def test_forced_transport_shm():
    setup()
    mgr = ChannelManager.instance()
    pub = mgr.create_publisher("shm_ch", msg_size=16,
                                transport=TransportLevel.L2_SHM)
    received = []
    sub = mgr.create_subscriber("shm_ch", msg_size=16,
                                 callback=lambda d: received.append(d))

    pub.publish(b"shm_channel_msg!")
    assert len(received) == 1
    assert received[0] == b"shm_channel_msg!"

    setup()
    print("[PASS] test_forced_transport_shm")


def test_singleton():
    setup()
    m1 = ChannelManager.instance()
    m2 = ChannelManager.instance()
    assert m1 is m2
    setup()
    print("[PASS] test_singleton")


def test_close_all():
    setup()
    mgr = ChannelManager.instance()
    mgr.create_publisher("close_a", msg_size=8)
    mgr.create_publisher("close_b", msg_size=8)
    assert len(mgr.list_channels()) == 2

    mgr.close_all()
    assert len(mgr.list_channels()) == 0
    setup()
    print("[PASS] test_close_all")


def test_cross_thread_pubsub():
    setup()
    mgr = ChannelManager.instance()
    pub = mgr.create_publisher("xt_ch", msg_size=16)

    results = []

    def reader():
        sub = mgr.create_subscriber("xt_ch", msg_size=16)
        for _ in range(5):
            if sub.spin_once(timeout_ms=2000):
                results.append(True)

    rt = threading.Thread(target=reader)
    rt.start()

    time.sleep(0.03)
    for i in range(5):
        pub.publish(f"xt_{i:010d}!!".encode())
        time.sleep(0.01)

    rt.join(timeout=15)
    assert len(results) == 5, f"Got {len(results)}"
    setup()
    print("[PASS] test_cross_thread_pubsub")


if __name__ == "__main__":
    print("=== channel tests ===")
    test_create_publisher_subscriber()
    test_multiple_subscribers()
    test_multiple_polling_subscribers_receive_same_message()
    test_spin_once()
    test_channel_info()
    test_list_channels()
    test_msg_size_mismatch()
    test_transport_mismatch()
    test_forced_transport_shm()
    test_singleton()
    test_close_all()
    test_cross_thread_pubsub()
    print("=== All channel tests passed ===")
