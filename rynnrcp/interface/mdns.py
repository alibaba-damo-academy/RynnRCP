"""mDNS discovery for RynnRCP Interface."""

from __future__ import annotations

import socket
import threading
import time
from typing import Any, Mapping

from rynnrcp.interface.discovery import Endpoint
from rynnrcp.utils import safe_name


MDNS_SERVICE_TYPE = "_rynn-rcp._tcp.local."


class MdnsPublisher:
    """Publish one gRPC endpoint via mDNS/Zeroconf."""

    def __init__(
        self,
        *,
        endpoint_id: str,
        port: int,
        metadata: Mapping[str, Any] | None = None,
        service_type: str = MDNS_SERVICE_TYPE,
    ):
        self.endpoint_id = endpoint_id
        self.port = int(port)
        self.metadata = dict(metadata or {})
        self.service_type = _service_type(service_type)
        self._zeroconf = None
        self._info = None

    def start(self) -> None:
        Zeroconf, ServiceInfo = _require_zeroconf()
        properties = {
            "protocol_version": "1",
            "transport": "grpc",
            "server_id": self.endpoint_id,
            "grpc_port": str(self.port),
            **{str(k): str(v) for k, v in self.metadata.items()},
        }
        self._zeroconf = Zeroconf()
        self._info = ServiceInfo(
            self.service_type,
            f"{safe_name(self.endpoint_id)}.{self.service_type}",
            addresses=[socket.inet_aton(ip) for ip in find_lan_ips()] or [socket.inet_aton("127.0.0.1")],
            port=self.port,
            properties=properties,
            server=f"{safe_name(socket.gethostname()) or 'rynnrcp'}.local.",
        )
        self._zeroconf.register_service(self._info)

    def stop(self) -> None:
        if self._zeroconf is None or self._info is None:
            return
        self._zeroconf.unregister_service(self._info)
        self._zeroconf.close()
        self._zeroconf = None
        self._info = None


class MdnsDiscovery:
    """Find Interface gRPC endpoints on the local network."""

    def __init__(self, service_type: str = MDNS_SERVICE_TYPE):
        self.service_type = _service_type(service_type)

    def discover(self, timeout_s: float | None = 1.5) -> list[Endpoint]:
        Zeroconf, _ServiceInfo = _require_zeroconf()
        from zeroconf import ServiceBrowser

        listener = _MdnsListener()
        zeroconf = Zeroconf()
        browser = ServiceBrowser(zeroconf, self.service_type, listener)
        try:
            time.sleep(float(timeout_s if timeout_s is not None else 1.5))
            return listener.endpoints()
        finally:
            browser.cancel()
            zeroconf.close()


class _MdnsListener:
    def __init__(self):
        self._lock = threading.Lock()
        self._endpoints: dict[str, Endpoint] = {}

    def add_service(self, zeroconf: Any, service_type: str, name: str) -> None:
        self._read_service(zeroconf, service_type, name)

    def update_service(self, zeroconf: Any, service_type: str, name: str) -> None:
        self._read_service(zeroconf, service_type, name)

    def remove_service(self, zeroconf: Any, service_type: str, name: str) -> None:
        with self._lock:
            self._endpoints.pop(name, None)

    def endpoints(self) -> list[Endpoint]:
        with self._lock:
            return list(self._endpoints.values())

    def _read_service(self, zeroconf: Any, service_type: str, name: str) -> None:
        info = zeroconf.get_service_info(service_type, name, timeout=1000)
        if info is None:
            return
        properties = {_decode(k): _decode(v) for k, v in dict(info.properties or {}).items()}
        addresses = [socket.inet_ntoa(address) for address in info.addresses]
        host = addresses[0] if addresses else info.server.rstrip(".")
        with self._lock:
            self._endpoints[name] = Endpoint(
                endpoint_id=properties.get("server_id") or name.split(".", 1)[0],
                transport="grpc",
                address=f"{host}:{int(info.port)}",
                metadata=properties,
                source="mdns",
            )


def find_lan_ips() -> list[str]:
    """Return likely LAN IPv4 addresses for the current host."""
    ips = {_probe_default_route_ip(), *_hostname_ips()}
    return sorted(ip for ip in ips if ip and not ip.startswith("127."))


def _probe_default_route_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            return str(sock.getsockname()[0])
    except OSError:
        return ""


def _hostname_ips() -> list[str]:
    try:
        return [item[4][0] for item in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)]
    except OSError:
        return []


def _decode(value: Any) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def _service_type(value: str) -> str:
    value = str(value or MDNS_SERVICE_TYPE).strip()
    return value if value.endswith(".") else f"{value}."


def _require_zeroconf() -> tuple[Any, Any]:
    try:
        from zeroconf import ServiceInfo, Zeroconf
    except ModuleNotFoundError as exc:
        raise RuntimeError("mDNS discovery requires zeroconf. Install with: pip install zeroconf") from exc
    return Zeroconf, ServiceInfo
