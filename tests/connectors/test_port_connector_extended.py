"""Extended tests for PortConnector class resolution and scheduling."""

from __future__ import annotations

import sys
import time
import types
from typing import Any

import pytest

from rynnrcp.connectors.port_connector import PortConnector


@pytest.fixture
def driver_module(monkeypatch: pytest.MonkeyPatch) -> types.ModuleType:
    """Install a fake driver module importable as ``fake_driver_mod``."""
    module = types.ModuleType("fake_driver_mod")

    class FakeSensor:
        instances: list["FakeSensor"] = []

        def __init__(self, device_id: str = "", fps: float = 0.0) -> None:
            self.device_id = device_id
            self.fps = fps
            self.started = False
            self.stopped = False
            self.reads = 0
            FakeSensor.instances.append(self)

        def start(self) -> None:
            self.started = True

        def read(self) -> tuple:
            self.reads += 1
            return (True, self.reads)

        def stop(self) -> None:
            self.stopped = True

    module.FakeSensor = FakeSensor
    module.not_a_class = object()
    monkeypatch.setitem(sys.modules, "fake_driver_mod", module)
    return module


def test_resolve_class_by_dotted_path(driver_module) -> None:
    connector = PortConnector()
    mod_name, cls = connector._resolve_class("fake_driver_mod.FakeSensor")
    assert mod_name == "fake_driver_mod"
    assert cls is driver_module.FakeSensor
    # Second lookup hits the cache.
    assert connector._resolve_class("fake_driver_mod.FakeSensor")[1] is cls


def test_resolve_class_rejects_non_class(driver_module) -> None:
    connector = PortConnector()
    with pytest.raises(TypeError, match="is not a class"):
        connector._resolve_class("fake_driver_mod.not_a_class")


def test_resolve_class_unknown_module_raises() -> None:
    connector = PortConnector()
    with pytest.raises(ValueError, match="Cannot resolve port_type"):
        connector._resolve_class("missing_module.MissingClass")
    with pytest.raises(ValueError, match="Cannot resolve port_type"):
        connector._resolve_class("TotallyUnknownDriver")


def test_sub_requires_port_type() -> None:
    connector = PortConnector()
    with pytest.raises(ValueError, match="port_type is required"):
        connector.sub({}, lambda msg: None)


def test_sub_verifies_lifecycle_interface(driver_module, monkeypatch) -> None:
    class NoStop:
        def __init__(self) -> None:
            pass

        def start(self) -> None:
            pass

        def read(self) -> None:
            pass

    driver_module.NoStop = NoStop
    connector = PortConnector()
    with pytest.raises(TypeError, match="no callable stop"):
        connector.sub({"port_type": "fake_driver_mod.NoStop"}, lambda msg: None)


def test_sub_thread_polling_delivers_messages(driver_module) -> None:
    connector = PortConnector()
    received: list = []
    connector.sub(
        {
            "port_type": "fake_driver_mod.FakeSensor",
            "init_args": {"device_id": "cam0", "fps": 200.0},
            "object_name": "obs",
        },
        received.append,
    )
    sensor = driver_module.FakeSensor.instances[-1]
    assert sensor.started is True

    deadline = time.time() + 2.0
    while len(received) < 2 and time.time() < deadline:
        time.sleep(0.01)
    connector.stop()

    assert len(received) >= 2
    assert received[0] == (True, 1)
    assert sensor.stopped is True
    assert connector._threads == {}
    assert connector._instances == {}


def test_sub_scheduler_mode_registers_component(driver_module) -> None:
    class FakeScheduler:
        def __init__(self) -> None:
            self.components: dict[str, Any] = {}
            self.removed: list[str] = []

        def add_component(self, component: Any) -> None:
            self.components[component.name] = component

        def remove_component(self, name: str) -> None:
            self.removed.append(name)

    scheduler = FakeScheduler()
    connector = PortConnector()
    received: list = []
    connector.sub(
        {
            "port_type": "fake_driver_mod.FakeSensor",
            "init_args": {"device_id": "cam1"},
            "object_name": "obs",
            "interval": 0.02,
            "_scheduler": scheduler,
            "_component_name": "camera:front",
        },
        received.append,
    )

    assert "camera:front" in scheduler.components
    component = scheduler.components["camera:front"]
    assert component.period_ms == pytest.approx(20.0)

    # Manually tick the registered component like the scheduler would.
    component.callback()
    assert received == [(True, 1)]

    connector.stop()
    assert scheduler.removed == ["camera:front"]
    # After stop the tick becomes a no-op.
    component.callback()
    assert received == [(True, 1)]


def test_read_errors_are_rate_limited(driver_module, monkeypatch, caplog) -> None:
    connector = PortConnector()
    exc = RuntimeError("read failed")
    connector._log_loop_error("k", "msg", exc)
    connector._log_loop_error("k", "msg", exc)
    assert list(connector._last_error_log_at) == ["k"]

    # After the rate-limit window a new log timestamp is recorded.
    connector._last_error_log_at["k"] -= 3.0
    before = connector._last_error_log_at["k"]
    connector._log_loop_error("k", "msg", exc)
    assert connector._last_error_log_at["k"] > before


def test_pub_is_not_supported() -> None:
    with pytest.raises(NotImplementedError):
        PortConnector().pub({}, None)
