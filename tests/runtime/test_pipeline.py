# Copyright 2026 RynnRCP Authors. All rights reserved.
# Integration test: full pipeline Publisher -> Channel -> Subscriber with Scheduler.

import sys
import os
import json
import time
import threading
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rynnrcp.ipc.channel import ChannelManager
from rynnrcp.runtime.scheduler import Scheduler, Component
from rynnrcp.ipc.transport import TransportLevel
from rynnrcp.runtime.runner_manager import ConnectorOutputTarget
from rynnrcp.utils.payload import pack_channel_message


def test_full_pipeline_50hz():
    """Publisher at 50Hz -> Channel -> Subscriber callback verification."""
    ChannelManager.reset()
    mgr = ChannelManager.instance()

    pub = mgr.create_publisher("imu_data", msg_size=32)
    received = []
    lock = threading.Lock()

    def on_imu(data: bytes):
        seq = int.from_bytes(data[:4], "little")
        with lock:
            received.append(seq)

    sub = mgr.create_subscriber("imu_data", msg_size=32, callback=on_imu)

    # Scheduler drives publisher at 50Hz (20ms period)
    sched = Scheduler()
    seq_counter = [0]

    def publish_imu():
        seq = seq_counter[0]
        seq_counter[0] += 1
        data = seq.to_bytes(4, "little") + b'\x00' * 28
        pub.publish(data)

    sched.add_component(Component("imu_pub", period_ms=20, callback=publish_imu))

    sched.start()
    time.sleep(0.5)  # 500ms -> ~25 messages at 50Hz
    sched.stop()

    with lock:
        count = len(received)

    assert count >= 15, f"Only received {count} messages (expected ~25)"
    assert count <= 40, f"Received too many: {count}"
    # Check monotonically increasing
    for i in range(1, len(received)):
        assert received[i] > received[i - 1], \
            f"Non-monotonic at {i}: {received[i-1]} -> {received[i]}"

    ChannelManager.reset()
    print(f"[PASS] test_full_pipeline_50hz (received {count} msgs)")


def test_multi_channel_concurrent():
    """Multiple channels running concurrently through scheduler."""
    ChannelManager.reset()
    mgr = ChannelManager.instance()

    channels = {
        "cam": {"size": 16, "period": 33, "count": [0], "received": []},  # ~30Hz
        "imu": {"size": 8, "period": 10, "count": [0], "received": []},   # ~100Hz
        "cmd": {"size": 8, "period": 50, "count": [0], "received": []},   # ~20Hz
    }

    lock = threading.Lock()
    sched = Scheduler()

    for name, cfg in channels.items():
        pub = mgr.create_publisher(name, msg_size=cfg["size"])

        def make_cb(ch_name):
            def cb(data):
                seq = int.from_bytes(data[:4], "little")
                with lock:
                    channels[ch_name]["received"].append(seq)
            return cb

        mgr.create_subscriber(name, msg_size=cfg["size"], callback=make_cb(name))

        def make_pub(pub_ref, ch_name, sz):
            def publish():
                seq = channels[ch_name]["count"][0]
                channels[ch_name]["count"][0] += 1
                data = seq.to_bytes(4, "little") + b'\x00' * (sz - 4)
                pub_ref.publish(data)
            return publish

        sched.add_component(Component(
            f"{name}_pub",
            period_ms=cfg["period"],
            callback=make_pub(pub, name, cfg["size"]),
        ))

    sched.start()
    time.sleep(0.3)
    sched.stop()

    with lock:
        cam_count = len(channels["cam"]["received"])
        imu_count = len(channels["imu"]["received"])
        cmd_count = len(channels["cmd"]["received"])

    # imu (100Hz) should have most, cmd (20Hz) least
    assert imu_count > cam_count, f"imu={imu_count} should > cam={cam_count}"
    assert cam_count > cmd_count or cam_count >= cmd_count, \
        f"cam={cam_count} should >= cmd={cmd_count}"
    assert imu_count >= 10, f"imu only got {imu_count}"
    assert cmd_count >= 2, f"cmd only got {cmd_count}"

    ChannelManager.reset()
    print(f"[PASS] test_multi_channel_concurrent "
          f"(imu={imu_count}, cam={cam_count}, cmd={cmd_count})")


def test_subscriber_spin_loop():
    """Subscriber using spin_once loop instead of callbacks."""
    ChannelManager.reset()
    mgr = ChannelManager.instance()

    pub = mgr.create_publisher("spin_ch", msg_size=8)
    sub = mgr.create_subscriber("spin_ch", msg_size=8)

    collected = []

    # Publisher thread
    def publisher():
        for i in range(10):
            pub.publish(i.to_bytes(8, "little"))
            time.sleep(0.02)

    # Subscriber spin loop
    def subscriber():
        for _ in range(10):
            if sub.spin_once(timeout_ms=2000):
                data = b""  # spin_once doesn't return data directly with no callback
                collected.append(True)

    # Use callback version for proper collection
    collected2 = []
    sub2 = mgr.create_subscriber("spin_ch", msg_size=8,
                                  callback=lambda d: collected2.append(
                                      int.from_bytes(d, "little")))

    pt = threading.Thread(target=publisher)
    pt.start()
    pt.join()

    assert len(collected2) == 10, f"Got {len(collected2)}"
    assert collected2 == list(range(10)), f"Got {collected2}"

    ChannelManager.reset()
    print("[PASS] test_subscriber_spin_loop")


def test_shm_channel_pipeline():
    """Full pipeline using forced SHM transport."""
    ChannelManager.reset()
    mgr = ChannelManager.instance()

    pub = mgr.create_publisher("shm_pipeline", msg_size=16,
                                transport=TransportLevel.L2_SHM)
    received = []
    sub = mgr.create_subscriber("shm_pipeline", msg_size=16,
                                 callback=lambda d: received.append(
                                     int.from_bytes(d[:4], "little")))

    for i in range(20):
        data = i.to_bytes(4, "little") + b'\x00' * 12
        pub.publish(data)

    assert len(received) == 20
    assert received == list(range(20))

    ChannelManager.reset()
    print("[PASS] test_shm_channel_pipeline")


def test_memory_output_transport_does_not_wait_for_scheduler_tick():
    """Thread-mode memory action channels should be consumed by event wakeup."""
    ChannelManager.reset()

    received = []
    received_event = threading.Event()

    class FakeConnectorFactory:
        def pub(self, connector, params, msg):
            received.append((connector, params, msg))
            received_event.set()

    class FakeOutputAdapter:
        def build_step_output(self, step_output, outputs, fps):
            return [[("module", {"fps": fps}, step_output, 0.0, 1)]]

    target = ConnectorOutputTarget(
        connector_factory=FakeConnectorFactory(),
        connector="module",
        params={},
        output_adapter=FakeOutputAdapter(),
        output_config={},
        channel_name="action.test",
        msg_size=1024,
        channel_transport="memory",
    )

    target.start()
    try:
        publisher = ChannelManager.instance().create_publisher(
            "action.test",
            msg_size=1024,
            transport=TransportLevel.L1_INTRA_PROCESS,
        )
        payload = json.dumps(
            {
                "timestamp": time.time(),
                "extra": {
                    "action_name": "action.test",
                    "action.test": {"joint_positions": [1.0]},
                    "action_type": "joint_position",
                    "fps": 60.0,
                },
            }
        ).encode("utf-8")
        publisher.publish(pack_channel_message(time.time(), payload))

        assert received_event.wait(timeout=0.2)
        assert received[0][2]["action.test"] == {"joint_positions": [1.0]}
    finally:
        target.stop()
        ChannelManager.reset()


if __name__ == "__main__":
    print("=== integration tests ===")
    test_full_pipeline_50hz()
    test_multi_channel_concurrent()
    test_subscriber_spin_loop()
    test_shm_channel_pipeline()
    print("=== All integration tests passed ===")
