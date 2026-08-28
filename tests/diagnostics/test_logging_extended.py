"""Extended tests for rynnrcp.utils.logging (tags, shm sink, server config)."""

from __future__ import annotations

import logging
from pathlib import Path

import pytest

from rynnrcp.utils.logging import (
    ShmLogHandler,
    configure_logging,
    configure_server_logging,
    log_tag,
)


@pytest.fixture(autouse=True)
def restore_root_logger():
    root = logging.getLogger()
    previous_handlers = list(root.handlers)
    previous_level = root.level
    yield
    root.handlers.clear()
    for handler in previous_handlers:
        root.addHandler(handler)
    root.setLevel(previous_level)


def test_log_tag_maps_known_namespaces() -> None:
    assert log_tag("rynnrcp.interface.server") == "INTERFACE"
    assert log_tag("rynnrcp.runtime") == "RUNTIME"
    assert log_tag("rynnrcp.services.action_service") == "SERVICE"
    assert log_tag("rynnrcp.connectors.port_connector") == "CONNECTOR"
    assert log_tag("rynnrcp.ipc.channel") == "IPC"
    assert log_tag("rynnkit.cameras.usb_camera") == "CAMERA"
    assert log_tag("rynnrcp.app.teleop") == "APP"
    assert log_tag("rynnrcp_app_rynnbot.mqtt_client") == "APP"
    assert log_tag("rynnrcp_robot_sim.zmq_clients") == "ROBOT"
    assert log_tag("rynnrcp.action_bridge") == "BRIDGE"
    assert log_tag("rynnrcp") == "SERVER"
    assert log_tag("rynnrcp.other") == "SERVER"
    assert log_tag("third_party.lib") == "GENERAL"


def test_configure_logging_rejects_unknown_sink() -> None:
    with pytest.raises(ValueError, match="Unknown log sink"):
        configure_logging(sinks=["carrier-pigeon"])


def test_configure_logging_named_logger_disables_propagation() -> None:
    logger = configure_logging(logger_name="rynnrcp.test.naming", sinks=["stderr"])
    assert logger.propagate is False
    assert logger.handlers


def test_configure_logging_shm_sink_writes_ring_buffer() -> None:
    logger = configure_logging(
        sinks=["shm"],
        shm_channel="rynnrcp_log_test",
        shm_level=logging.INFO,
        logger_name="rynnrcp.test.shm",
    )
    shm_handlers = [h for h in logger.handlers if isinstance(h, ShmLogHandler)]
    assert len(shm_handlers) == 1
    handler = shm_handlers[0]
    try:
        logger.info("hello shm")
        assert handler._ring_buffer is not None

        from rynnrcp.ipc.ring_buffer import RingBuffer

        reader = RingBuffer(
            "rynnrcp_log_test", slot_size=2048, slot_count=256, create=False
        )
        try:
            index = reader.latest_index()
            assert index >= 0
            message = reader.read(index)
            assert b"hello shm" in bytes(message)
        finally:
            reader.close()
    finally:
        handler.close()
        assert handler._ring_buffer is None
        logger.handlers.clear()


def test_configure_server_logging_writes_into_session_dir(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setenv("RYNNRCP_HOME", str(tmp_path))
    config = {
        "config_type": "rynnrcp_server_config",
        "manifest": {"robot_id": "test_robot"},
    }
    monkeypatch.setenv("RYNNRCP_RUN_ID", "integration-run")
    logger = configure_server_logging(config, session_id="sess1")
    try:
        logger.info("server line")
        log_file = tmp_path / "test_robot" / "logs" / "sess1" / "server.log"
        assert log_file.exists()
        from rynnrcp.utils.logging import get_log_context

        assert get_log_context()["run_id"] == "integration-run"
    finally:
        from rynnrcp.utils.logging import _stop_async_file_loggers, clear_log_context

        _stop_async_file_loggers()
        clear_log_context()
        logger.handlers.clear()
