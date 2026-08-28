"""Tests for mDNS publisher/discovery helpers with fake zeroconf objects."""

from __future__ import annotations

import socket
import types
from typing import Any

import pytest

import rynnrcp.interface.mdns as mdns
from rynnrcp.interface.mdns import (
    MDNS_SERVICE_TYPE,
    MdnsPublisher,
    _decode,
    _MdnsListener,
    _service_type,
    find_lan_ips,
)


def test_service_type_normalization() -> None:
    assert _service_type("_x._tcp.local") == "_x._tcp.local."
    assert _service_type("_x._tcp.local.") == "_x._tcp.local."
    assert _service_type("") == MDNS_SERVICE_TYPE


def test_decode_handles_bytes_and_values() -> None:
    assert _decode(b"abc") == "abc"
    assert _decode(b"\xff") == "\ufffd"
    assert _decode(7) == "7"


def test_find_lan_ips_excludes_loopback(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(mdns, "_probe_default_route_ip", lambda: "10.0.0.5")
    monkeypatch.setattr(mdns, "_hostname_ips", lambda: ["127.0.0.1", "10.0.0.5", "192.168.1.9"])
    assert find_lan_ips() == ["10.0.0.5", "192.168.1.9"]

    monkeypatch.setattr(mdns, "_probe_default_route_ip", lambda: "")
    monkeypatch.setattr(mdns, "_hostname_ips", lambda: [])
    assert find_lan_ips() == []


class _FakeZeroconf:
    def __init__(self) -> None:
        self.registered: list[Any] = []
        self.unregistered: list[Any] = []
        self.closed = False
        self.services: dict[tuple[str, str], Any] = {}

    def register_service(self, info: Any) -> None:
        self.registered.append(info)

    def unregister_service(self, info: Any) -> None:
        self.unregistered.append(info)

    def close(self) -> None:
        self.closed = True

    def get_service_info(self, service_type: str, name: str, timeout: int = 1000):
        return self.services.get((service_type, name))


class _FakeServiceInfo:
    def __init__(self, service_type, name, *, addresses, port, properties, server):
        self.type = service_type
        self.name = name
        self.addresses = addresses
        self.port = port
        self.properties = properties
        self.server = server


@pytest.fixture
def fake_zeroconf(monkeypatch: pytest.MonkeyPatch) -> _FakeZeroconf:
    instance = _FakeZeroconf()
    monkeypatch.setattr(
        mdns, "_require_zeroconf", lambda: (lambda: instance, _FakeServiceInfo)
    )
    return instance


def test_publisher_registers_and_unregisters(
    fake_zeroconf: _FakeZeroconf, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(mdns, "find_lan_ips", lambda: ["192.168.1.10"])
    publisher = MdnsPublisher(
        endpoint_id="robot-1", port=8080, metadata={"robot": "so101"}
    )
    publisher.start()

    info = fake_zeroconf.registered[0]
    assert info.type == MDNS_SERVICE_TYPE
    assert info.port == 8080
    assert info.properties["server_id"] == "robot-1"
    assert info.properties["grpc_port"] == "8080"
    assert info.properties["robot"] == "so101"
    assert info.addresses == [socket.inet_aton("192.168.1.10")]

    publisher.stop()
    assert fake_zeroconf.unregistered == [info]
    assert fake_zeroconf.closed is True
    # A second stop is a harmless no-op.
    publisher.stop()


def test_publisher_falls_back_to_loopback_address(
    fake_zeroconf: _FakeZeroconf, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(mdns, "find_lan_ips", lambda: [])
    publisher = MdnsPublisher(endpoint_id="robot-1", port=8080)
    publisher.start()
    assert fake_zeroconf.registered[0].addresses == [socket.inet_aton("127.0.0.1")]
    publisher.stop()


def _service(properties: dict[bytes, bytes], addresses=None, port: int = 8080):
    return types.SimpleNamespace(
        properties=properties,
        addresses=addresses if addresses is not None else [socket.inet_aton("10.0.0.9")],
        port=port,
        server="host.local.",
    )


def test_listener_tracks_add_update_and_remove() -> None:
    zeroconf = _FakeZeroconf()
    listener = _MdnsListener()
    name = f"robot-1.{MDNS_SERVICE_TYPE}"
    zeroconf.services[(MDNS_SERVICE_TYPE, name)] = _service(
        {b"server_id": b"robot-1", b"transport": b"grpc"}
    )

    listener.add_service(zeroconf, MDNS_SERVICE_TYPE, name)
    endpoints = listener.endpoints()
    assert len(endpoints) == 1
    endpoint = endpoints[0]
    assert endpoint.endpoint_id == "robot-1"
    assert endpoint.address == "10.0.0.9:8080"
    assert endpoint.source == "mdns"

    zeroconf.services[(MDNS_SERVICE_TYPE, name)] = _service(
        {b"server_id": b"robot-1"}, port=9090
    )
    listener.update_service(zeroconf, MDNS_SERVICE_TYPE, name)
    assert listener.endpoints()[0].address == "10.0.0.9:9090"

    listener.remove_service(zeroconf, MDNS_SERVICE_TYPE, name)
    assert listener.endpoints() == []


def test_listener_ignores_unresolvable_services() -> None:
    zeroconf = _FakeZeroconf()
    listener = _MdnsListener()
    listener.add_service(zeroconf, MDNS_SERVICE_TYPE, "ghost." + MDNS_SERVICE_TYPE)
    assert listener.endpoints() == []


def test_listener_falls_back_to_server_hostname() -> None:
    zeroconf = _FakeZeroconf()
    listener = _MdnsListener()
    name = f"noaddr.{MDNS_SERVICE_TYPE}"
    zeroconf.services[(MDNS_SERVICE_TYPE, name)] = _service({}, addresses=[], port=8081)
    listener.add_service(zeroconf, MDNS_SERVICE_TYPE, name)
    endpoint = listener.endpoints()[0]
    assert endpoint.address == "host.local:8081"
    # Without server_id the endpoint id falls back to the service short name.
    assert endpoint.endpoint_id == "noaddr"
