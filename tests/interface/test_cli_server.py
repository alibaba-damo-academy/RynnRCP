"""Tests for the rynnrcp-server CLI entry point."""

from __future__ import annotations

from typing import Any

import pytest

import rynnrcp.cli_server as cli_server


class _FakeServer:
    instances: list["_FakeServer"] = []

    def __init__(self, config: str, *, log_session_id: str | None = None) -> None:
        self.config = config
        self.log_session_id = log_session_id
        self.server_instance_id = "instance-1"
        self.bound_port = 8080
        self.visualization_urls = [
            "http://127.0.0.1:9000",
            "http://192.168.0.2:9000",
        ]
        self.started = False
        self.stopped = False
        self.start_error: Exception | None = None
        self.wait_error: Exception | None = None
        _FakeServer.instances.append(self)

    def start(self) -> None:
        if self.start_error is not None:
            raise self.start_error
        self.started = True

    def wait_for_termination(self) -> None:
        if self.wait_error is not None:
            raise self.wait_error

    def stop(self) -> None:
        self.stopped = True


@pytest.fixture(autouse=True)
def patched_cli(monkeypatch: pytest.MonkeyPatch):
    _FakeServer.instances = []
    monkeypatch.setattr(cli_server, "RynnRCPServer", _FakeServer)
    monkeypatch.setattr(cli_server, "configure_server_logging", lambda *a, **k: None)
    monkeypatch.setattr(cli_server, "new_log_session_id", lambda: "session-1")
    monkeypatch.setattr(cli_server, "find_lan_ips", lambda: ["192.168.0.2"])
    return None


def test_main_requires_config_argument() -> None:
    with pytest.raises(SystemExit):
        cli_server.main([])


def test_main_runs_server_and_prints_endpoints(capsys: pytest.CaptureFixture) -> None:
    exit_code = cli_server.main(["--config", "server.yaml"])
    assert exit_code == 0

    server = _FakeServer.instances[0]
    assert server.config == "server.yaml"
    assert server.log_session_id == "session-1"
    assert server.started and server.stopped

    out = capsys.readouterr().out
    assert "RynnRCP server is running." in out
    assert "127.0.0.1:8080" in out
    assert "192.168.0.2:8080" in out
    assert "Debug UI Local: http://127.0.0.1:9000" in out
    assert "Debug UI LAN:   http://192.168.0.2:9000" in out


def test_main_returns_2_when_start_fails(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture
) -> None:
    def broken_server(config: str, **kwargs: Any) -> _FakeServer:
        server = _FakeServer(config, **kwargs)
        server.start_error = RuntimeError("port busy")
        return server

    monkeypatch.setattr(cli_server, "RynnRCPServer", broken_server)
    exit_code = cli_server.main(["--config", "server.yaml"])
    assert exit_code == 2
    assert "port busy" in capsys.readouterr().err


def test_main_returns_130_on_keyboard_interrupt(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture
) -> None:
    def interrupted_server(config: str, **kwargs: Any) -> _FakeServer:
        server = _FakeServer(config, **kwargs)
        server.wait_error = KeyboardInterrupt()
        return server

    monkeypatch.setattr(cli_server, "RynnRCPServer", interrupted_server)
    exit_code = cli_server.main(["--config", "server.yaml"])
    assert exit_code == 130
    assert _FakeServer.instances[0].stopped is True
    assert "Stopping RynnRCP server." in capsys.readouterr().out
