"""Tests for rynnrcp.connectors (base, factory, module, port)."""

import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rynnrcp.connectors.base import BaseConnector
from rynnrcp.connectors.module_connector import ModuleConnector
from rynnrcp.connectors.port_connector import PortConnector
from rynnrcp.connectors.connector_factory import ConnectorFactory, _check_availability
from rynnrcp.utils import camel_to_snake


# ── Mock classes for testing ─────────────────────────────────────────

class _MockSensor:
    """Mock sensor with start/read/stop lifecycle."""

    def __init__(self, device_id=0):
        self.device_id = device_id
        self.started = False
        self.stopped = False
        self._counter = 0

    def start(self):
        self.started = True

    def read(self):
        self._counter += 1
        return (True, 640, 480, "rgb8", b"\x00" * 10)

    def stop(self):
        self.stopped = True


class _MockActuator:
    """Mock actuator for module connector tests."""

    def __init__(self, name="arm"):
        self.name = name
        self.last_positions = None
        self._started = False
        self._destroyed = False
        self._homed = False

    def start(self):
        self._started = True

    def set_position(self, positions: list, speed: float = 1.0):
        self.last_positions = positions
        return True

    def read_state(self):
        return [0.1, 0.2, 0.3]

    def destroy(self):
        self._destroyed = True

    def go_home(self):
        self._homed = True


# Register mock classes so ModuleConnector can import them
# We'll use the current module path
_THIS_MODULE = f"{__name__}"


# ── BaseConnector ABC test ───────────────────────────────────────────

class TestBaseConnector(unittest.TestCase):

    def test_cannot_instantiate_abc(self):
        with self.assertRaises(TypeError):
            BaseConnector()

    def test_concrete_subclass(self):
        class Dummy(BaseConnector):
            def pub(self, params, msg): pass
            def sub(self, params, callback): pass
            def stop(self): pass

        d = Dummy()
        self.assertIsInstance(d, BaseConnector)


# ── ModuleConnector tests ────────────────────────────────────────────

class TestModuleConnector(unittest.TestCase):

    def test_sub_mode(self):
        mc = ModuleConnector()
        results = []

        # Point module_name to our mock
        params = {
            "module_name": f"{_THIS_MODULE}._MockActuator",
            "init_args": {"name": "test_arm"},
            "method_name": "read_state",
            "interval": 0.0,
        }
        mc.sub(params, lambda r: results.append(r))
        time.sleep(0.1)
        mc.stop()

        self.assertGreater(len(results), 0)
        self.assertEqual(results[0], [0.1, 0.2, 0.3])

    def test_pub_mode(self):
        mc = ModuleConnector()

        params = {
            "module_name": f"{_THIS_MODULE}._MockActuator",
            "init_args": {"name": "pub_arm"},
            "method_name": "set_position",
            "dynamic_arg": [{"from_key": "action", "arg": "positions"}],
            "static_args": [{"speed": 0.5}],
        }
        msg = {"action": [1.0, 2.0, 3.0]}
        mc.pub(params, msg)

        inst = mc.get_instance(params)
        self.assertEqual(inst.last_positions, [1.0, 2.0, 3.0])
        mc.stop()

    def test_instance_reuse(self):
        mc = ModuleConnector()
        params = {
            "module_name": f"{_THIS_MODULE}._MockActuator",
            "init_args": {"name": "reuse"},
            "method_name": "read_state",
        }
        inst1 = mc.get_instance(params)
        inst2 = mc.get_instance(params)
        self.assertIs(inst1, inst2)
        mc.stop()

    def test_start_and_destroy_call(self):
        mc = ModuleConnector()
        params = {
            "module_name": f"{_THIS_MODULE}._MockActuator",
            "init_args": {"name": "lifecycle"},
            "method_name": "read_state",
            "start_call": [{"method_name": "start"}],
            "destroy_call": [{"method_name": "destroy"}],
        }
        inst = mc.get_instance(params)
        self.assertTrue(inst._started)

        mc.stop()
        self.assertTrue(inst._destroyed)

    def test_shared_instance_preserves_destroy_call_after_pub_params(self):
        mc = ModuleConnector()
        input_params = {
            "module_name": f"{_THIS_MODULE}._MockActuator",
            "init_args": {"name": "shared_lifecycle"},
            "method_name": "read_state",
            "start_call": [{"method_name": "start"}],
            "destroy_call": [
                {"method_name": "go_home"},
                {"method_name": "destroy"},
            ],
        }
        output_params = {
            "module_name": f"{_THIS_MODULE}._MockActuator",
            "init_args": {"name": "shared_lifecycle"},
            "method_name": "set_position",
            "dynamic_arg": [{"from_key": "action", "arg": "positions"}],
            "static_args": [],
        }
        inst = mc.get_instance(input_params)
        mc.pub(output_params, {"action": [1.0, 2.0, 3.0]})

        mc.stop()

        self.assertTrue(inst._homed)
        self.assertTrue(inst._destroyed)

    def test_missing_module_name(self):
        mc = ModuleConnector()
        with self.assertRaises(ValueError):
            mc.pub({"method_name": "foo"}, {})


# ── PortConnector tests ──────────────────────────────────────────────

class TestPortConnector(unittest.TestCase):

    def test_camel_to_snake(self):
        self.assertEqual(camel_to_snake("USBCamera"), "usb_camera")
        self.assertEqual(camel_to_snake("DepthCamera"), "depth_camera")
        self.assertEqual(camel_to_snake("Lidar"), "lidar")

    def test_sub_with_mock_sensor(self):
        """Test PortConnector sub by manually injecting a mock."""
        pc = PortConnector()
        results = []

        # Directly test the lifecycle without class resolution
        inst = _MockSensor(device_id=42)
        key = "MockSensor::42::"

        inst.start()
        pc._instances[key] = inst
        pc._running[key] = True

        def loop():
            while pc._running.get(key, False):
                ret = inst.read()
                results.append(ret)
                time.sleep(0.01)

        th = threading.Thread(target=loop, daemon=True)
        th.start()
        pc._threads[key] = th

        time.sleep(0.1)
        pc.stop()

        self.assertTrue(inst.started)
        self.assertTrue(inst.stopped)
        self.assertGreater(len(results), 0)
        self.assertEqual(results[0][0], True)  # ok flag

    def test_pub_not_supported(self):
        pc = PortConnector()
        with self.assertRaises(NotImplementedError):
            pc.pub({}, None)


# ── ConnectorFactory tests ───────────────────────────────────────────

class TestConnectorFactory(unittest.TestCase):

    def test_module_connector_via_factory(self):
        factory = ConnectorFactory(connector_names={"module"})
        self.assertIn("module", factory.available)
        conn = factory.get_connector("module")
        self.assertIsInstance(conn, ModuleConnector)

    def test_unknown_connector_raises(self):
        factory = ConnectorFactory(connector_names={"module"})
        with self.assertRaises(ValueError):
            factory.get_connector("grpc")

    def test_ros2_lcm_availability_check(self):
        # These just check the flag - they won't crash
        r = _check_availability("ros2")
        l = _check_availability("lcm")
        self.assertIsInstance(r, bool)
        self.assertIsInstance(l, bool)

    def test_declared_unused_connector_not_instantiated(self):
        factory = ConnectorFactory(
            connector_names={"module"},
            connectors_config={
                "unused": {"class": "missing.module.Connector"},
            },
        )
        self.assertEqual(factory.available, ["module"])

    def test_factories_are_config_isolated(self):
        f1 = ConnectorFactory(connector_names={"module"})
        f2 = ConnectorFactory(connector_names={"port"})
        self.assertIsNot(f1, f2)
        self.assertEqual(f1.available, ["module"])
        self.assertEqual(f2.available, ["port"])


# ── Run ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=== rynnrcp.connectors tests ===")
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    for tc in (TestBaseConnector, TestModuleConnector, TestPortConnector,
               TestConnectorFactory):
        suite.addTests(loader.loadTestsFromTestCase(tc))
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
