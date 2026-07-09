"""Client-facing Interface API."""

from __future__ import annotations

from typing import Iterable, Protocol

from rynnrcp.interface.discovery import Endpoint, LocalRegistry, StaticDiscovery
from rynnrcp.interface.grpc_transport import GrpcConnection, connect_grpc
from rynnrcp.interface.mdns import MdnsDiscovery


class DiscoveryProvider(Protocol):
    def discover(self, timeout_s: float | None = None) -> list[Endpoint]:
        ...


class ClientInterface:
    """Discover and connect to Interface servers."""

    def __init__(self, discovery_providers: Iterable[DiscoveryProvider] | None = None):
        self._providers = list(discovery_providers or [])

    @classmethod
    def with_defaults(
        cls,
        *,
        static_addresses: Iterable[str] | None = None,
        local_registry: bool = True,
        mdns: bool = False,
    ) -> "ClientInterface":
        providers: list[DiscoveryProvider] = []
        if static_addresses:
            providers.append(StaticDiscovery(static_addresses))
        if local_registry:
            providers.append(LocalRegistry())
        if mdns:
            providers.append(MdnsDiscovery())
        return cls(providers)

    def discover(self, timeout_s: float | None = 1.5) -> list[Endpoint]:
        endpoints: list[Endpoint] = []
        seen: set[tuple[str, str]] = set()
        for provider in self._providers:
            for endpoint in provider.discover(timeout_s=timeout_s):
                key = (endpoint.transport, endpoint.address)
                if key in seen:
                    continue
                seen.add(key)
                endpoints.append(endpoint)
        return endpoints

    def connect(self, endpoint: Endpoint | str) -> GrpcConnection:
        return connect_grpc(endpoint)
