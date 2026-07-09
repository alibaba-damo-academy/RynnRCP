"""Tests for multi-process support: SharedChannelRegistry, ChannelManager
auto-routing, ShmTransport cross-process poll, ProcessNode, NodeLauncher,
and GIL independence verification."""

import multiprocessing
import os
import struct
import sys
import threading
import time
import unittest

# ---------------------------------------------------------------------------
# Ensure the package root is on sys.path
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)


# ---------------------------------------------------------------------------
# 1. SharedChannelRegistry
# ---------------------------------------------------------------------------
class TestSharedChannelRegistry(unittest.TestCase):
    """Test SHM-based channel registry CRUD operations."""

    def setUp(self):
        from rynnrcp.ipc.channel_registry import SharedChannelRegistry
        self.reg = SharedChannelRegistry(create=True)

    def tearDown(self):
        self.reg.close()

    def test_register_and_lookup(self):
        self.reg.register("imu", 256, os.getpid(), "rc_transport_imu")
        entry = self.reg.lookup("imu")
        self.assertIsNotNone(entry)
        self.assertEqual(entry.name, "imu")
        self.assertEqual(entry.msg_size, 256)
        self.assertEqual(entry.owner_pid, os.getpid())
        self.assertEqual(entry.shm_name, "rc_transport_imu")

    def test_lookup_missing_returns_none(self):
        self.assertIsNone(self.reg.lookup("nonexistent"))

    def test_register_overwrite(self):
        self.reg.register("cam", 1024, os.getpid(), "rc_transport_cam")
        self.reg.register("cam", 2048, os.getpid() + 1, "rc_transport_cam2")
        entry = self.reg.lookup("cam")
        self.assertEqual(entry.msg_size, 2048)
        self.assertEqual(entry.owner_pid, os.getpid() + 1)

    def test_unregister(self):
        self.reg.register("lidar", 4096, os.getpid(), "rc_transport_lidar")
        self.assertTrue(self.reg.unregister("lidar", os.getpid()))
        self.assertIsNone(self.reg.lookup("lidar"))

    def test_unregister_wrong_pid(self):
        self.reg.register("lidar", 4096, os.getpid(), "rc_transport_lidar")
        self.assertFalse(self.reg.unregister("lidar", os.getpid() + 999))
        self.assertIsNotNone(self.reg.lookup("lidar"))

    def test_list_all(self):
        self.reg.register("a", 10, 1, "sa")
        self.reg.register("b", 20, 2, "sb")
        self.reg.register("c", 30, 3, "sc")
        entries = self.reg.list_all()
        names = {e.name for e in entries}
        self.assertEqual(names, {"a", "b", "c"})

    def test_open_existing(self):
        """A second registry instance (create=False) can read entries."""
        from rynnrcp.ipc.channel_registry import SharedChannelRegistry
        self.reg.register("test_ch", 512, os.getpid(), "rc_transport_test_ch")
        reg2 = SharedChannelRegistry(create=False)
        try:
            entry = reg2.lookup("test_ch")
            self.assertIsNotNone(entry)
            self.assertEqual(entry.msg_size, 512)
        finally:
            from rynnrcp.native.shm import shm_close
            shm_close(reg2._region)

    def test_concurrent_register(self):
        """Multiple threads registering concurrently should not corrupt data."""
        errors = []

        def _register(idx):
            try:
                self.reg.register(f"ch_{idx}", idx * 10, os.getpid(),
                                  f"rc_transport_ch_{idx}")
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=_register, args=(i,)) for i in range(20)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        self.assertEqual(len(errors), 0)
        entries = self.reg.list_all()
        self.assertEqual(len(entries), 20)


# ---------------------------------------------------------------------------
# 2. ChannelManager auto-routing with registry
# ---------------------------------------------------------------------------
class TestChannelAutoRouting(unittest.TestCase):
    """Test that ChannelManager detects cross-process channels via registry."""

    def setUp(self):
        from rynnrcp.ipc.channel import ChannelManager
        ChannelManager.reset()
        self.mgr = ChannelManager.instance()

    def tearDown(self):
        from rynnrcp.ipc.channel import ChannelManager
        ChannelManager.reset()

    def test_default_is_intra_process(self):
        from rynnrcp.ipc.transport import TransportLevel
        level = self.mgr.auto_detect_transport("any_channel")
        self.assertEqual(level, TransportLevel.L1_INTRA_PROCESS)

    def test_auto_detect_shm_with_registry(self):
        """When registry shows another PID, auto_detect returns L2_SHM."""
        from rynnrcp.ipc.transport import TransportLevel
        from rynnrcp.ipc.channel_registry import SharedChannelRegistry
        reg = SharedChannelRegistry(create=True)
        try:
            # Register a channel owned by a different PID
            other_pid = os.getpid() + 9999
            reg.register("remote_ch", 256, other_pid, "rc_transport_remote_ch")
            self.mgr.attach_registry(reg)

            level = self.mgr.auto_detect_transport("remote_ch")
            self.assertEqual(level, TransportLevel.L2_SHM)

            # Own PID -> still intra
            reg.register("local_ch", 256, os.getpid(), "rc_transport_local_ch")
            level2 = self.mgr.auto_detect_transport("local_ch")
            self.assertEqual(level2, TransportLevel.L1_INTRA_PROCESS)
        finally:
            reg.close()

    def test_no_registry_always_intra(self):
        from rynnrcp.ipc.transport import TransportLevel
        # No attach_registry call
        level = self.mgr.auto_detect_transport("whatever")
        self.assertEqual(level, TransportLevel.L1_INTRA_PROCESS)


# ---------------------------------------------------------------------------
# 3. ShmTransport cross-process poll
# ---------------------------------------------------------------------------
class TestShmTransportCrossProcessPoll(unittest.TestCase):
    """Verify that ShmTransport.poll() works without intra-process Event."""

    def test_poll_finds_data_written_directly_to_ring(self):
        """Simulate cross-process: write to RingBuffer directly, poll
        without the publisher calling Event.signal()."""
        from rynnrcp.ipc.transport import ShmTransport

        shm_name = "xproc_poll_test"
        msg_size = 64

        # "Publisher" side: create the SHM transport and write data without
        # sharing the in-process Event with the subscriber below.
        pub = ShmTransport(shm_name, msg_size=msg_size, slot_count=16,
                           create=True, poll_interval_us=50)
        payload = b"hello_cross_process"
        pub.publish(payload)

        try:
            # "Subscriber" side: open existing ring buffer via ShmTransport
            sub = ShmTransport(shm_name, msg_size=msg_size, slot_count=16,
                               create=False, poll_interval_us=50)
            data = sub.poll(timeout_ms=500)
            self.assertIsNotNone(data)
            self.assertTrue(data.startswith(b"hello_cross_process"))
            sub.close()
        finally:
            pub.close()
            pub.unlink()

    def test_poll_timeout_no_data(self):
        from rynnrcp.ipc.transport import ShmTransport

        shm_name = "xproc_timeout_test"
        t = ShmTransport(shm_name, msg_size=32, slot_count=4, create=True,
                         poll_interval_us=100)
        try:
            start = time.monotonic()
            data = t.poll(timeout_ms=50)
            elapsed = time.monotonic() - start
            self.assertIsNone(data)
            self.assertGreaterEqual(elapsed, 0.04)  # at least ~40ms
        finally:
            t.close()
            t.unlink()


# ---------------------------------------------------------------------------
# 4. ProcessNode lifecycle
# ---------------------------------------------------------------------------
def _dummy_setup(mgr, config):
    """A trivial component: publishes one message and returns a stop event."""
    stop = threading.Event()
    pub = mgr.create_publisher("test_ch", msg_size=64)
    pub.publish(b"init" + b"\x00" * 60)
    return stop


def _blocking_setup(_mgr, _config):
    time.sleep(30.0)


class TestProcessNodeLifecycle(unittest.TestCase):
    """Test ProcessNode start / terminate cycle."""

    def test_node_start_and_terminate(self):
        from rynnrcp.ipc.channel_registry import SharedChannelRegistry
        from rynnrcp.process.process_node import ProcessNode

        # Create registry in this process
        reg = SharedChannelRegistry(create=True)
        try:
            node = ProcessNode("test_node", config={})
            node.add_component(_dummy_setup)

            p = multiprocessing.Process(target=node.run, name="test_node",
                                        daemon=False)
            p.start()
            time.sleep(0.5)  # Let it initialise
            self.assertTrue(p.is_alive())

            p.terminate()
            p.join(timeout=3.0)
            self.assertFalse(p.is_alive())
        finally:
            reg.close()

    def test_node_terminate_interrupts_setup_before_ready(self):
        from rynnrcp.ipc.channel_registry import SharedChannelRegistry
        from rynnrcp.process.process_node import ProcessNode

        reg = SharedChannelRegistry(create=True)
        try:
            node = ProcessNode("blocking_node", config={})
            node.add_component(_blocking_setup)

            p = multiprocessing.Process(target=node.run, name="blocking_node", daemon=False)
            p.start()
            time.sleep(0.5)
            self.assertTrue(p.is_alive())

            p.terminate()
            p.join(timeout=2.0)
            self.assertFalse(p.is_alive())
        finally:
            if "p" in locals() and p.is_alive():
                p.kill()
                p.join(timeout=1.0)
            reg.close()


# ---------------------------------------------------------------------------
# 5. NodeLauncher multi-process
# ---------------------------------------------------------------------------
def _publisher_setup(mgr, config):
    """Publish 10 messages to 'mp_test' channel at 10 Hz."""
    import time as _time
    stop = threading.Event()
    pub = mgr.create_publisher("mp_test", msg_size=64,
                               transport=__import__("rynnrcp.ipc.transport",
                               fromlist=["TransportLevel"]).TransportLevel.L2_SHM)

    def _loop():
        for i in range(10):
            if stop.is_set():
                break
            msg = i.to_bytes(8, "little") + b"\x00" * 56
            pub.publish(msg)
            _time.sleep(0.05)

    t = threading.Thread(target=_loop, daemon=True)
    t.start()
    return stop


def _failing_setup(_mgr, _config):
    raise RuntimeError("setup boom")


class TestNodeLauncherMultiProcess(unittest.TestCase):
    """Launch two process nodes and verify SHM channel works."""

    def test_launch_and_shutdown(self):
        from rynnrcp.process.node_launcher import NodeLauncher

        launcher = NodeLauncher()
        launcher.add_node("pub_node", _publisher_setup)
        launcher.add_node("idle_node", _dummy_setup)

        launcher.start()
        try:
            time.sleep(0.3)
            health = launcher.health_check()
            # Both should be alive
            self.assertTrue(health.get("pub_node", False))
            self.assertTrue(health.get("idle_node", False))
        finally:
            launcher.shutdown(timeout=3.0)

        # After shutdown, none alive
        self.assertFalse(launcher.is_alive("pub_node"))
        self.assertFalse(launcher.is_alive("idle_node"))

    def test_start_reports_failed_node(self):
        from rynnrcp.process.node_launcher import NodeLauncher

        launcher = NodeLauncher(startup_timeout_s=2.0)
        launcher.add_node("bad_node", _failing_setup)

        with self.assertRaisesRegex(RuntimeError, "bad_node"):
            launcher.start()


# ---------------------------------------------------------------------------
# 6. GIL independence verification
# ---------------------------------------------------------------------------
def _cpu_burn(result_dict, key, iterations=50_000_000):
    """Burn CPU with pure Python arithmetic (truly GIL-bound).

    Uses a fixed iteration count rather than wall-clock duration to avoid
    releasing the GIL via time.monotonic() calls.
    """
    x = 1
    for _ in range(iterations):
        x = (x * 7 + 3) % 1000003
    result_dict[key] = x


class TestGILIndependence(unittest.TestCase):
    """Verify that multi-process achieves true parallelism vs threads."""

    def test_multiprocess_faster_than_threads(self):
        """Two CPU-bound tasks in processes should be faster than
        in threads (where GIL serialises them).

        Uses a fixed iteration count so the hot loop never releases the
        GIL (no time.monotonic / sum / range builtins in the hot path).
        """
        iters = 50_000_000

        # -- Multi-threaded (GIL-bound) --
        thread_results = {}
        t1 = threading.Thread(target=_cpu_burn,
                              args=(thread_results, "a", iters))
        t2 = threading.Thread(target=_cpu_burn,
                              args=(thread_results, "b", iters))
        ts = time.monotonic()
        t1.start(); t2.start()
        t1.join(); t2.join()
        thread_wall = time.monotonic() - ts

        # -- Multi-process (independent GIL) --
        mgr_mp = multiprocessing.Manager()
        proc_results = mgr_mp.dict()
        p1 = multiprocessing.Process(target=_cpu_burn,
                                     args=(proc_results, "a", iters))
        p2 = multiprocessing.Process(target=_cpu_burn,
                                     args=(proc_results, "b", iters))
        ps = time.monotonic()
        p1.start(); p2.start()
        p1.join(); p2.join()
        proc_wall = time.monotonic() - ps
        mgr_mp.shutdown()

        # Threads: ~2x single-task time (GIL serialises pure Python)
        # Processes: ~1x single-task time + spawn overhead
        print(f"\n  Thread wall: {thread_wall:.3f}s, Process wall: {proc_wall:.3f}s")
        self.assertLess(proc_wall, thread_wall * 0.95,
                        "Multi-process should be faster than multi-thread "
                        "for CPU-bound work (GIL avoidance)")


class TestProcessTask(unittest.TestCase):
    def test_run_python_function_task(self):
        from rynnrcp.process import run_python_function_task

        result = run_python_function_task(
            "operator:add",
            args=[2, 3],
            timeout_s=2,
        )

        self.assertEqual(result, 5)

    def test_run_python_function_task_error_contains_traceback(self):
        from rynnrcp.process import ProcessTaskError, run_python_function_task

        with self.assertRaises(ProcessTaskError) as cm:
            run_python_function_task(
                "operator:missing_function",
                timeout_s=2,
            )

        self.assertIn("Traceback", str(cm.exception))

if __name__ == "__main__":
    multiprocessing.set_start_method("spawn", force=True)
    unittest.main()
