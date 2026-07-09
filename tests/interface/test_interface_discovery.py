from __future__ import annotations

from rynnrcp.interface.discovery import Endpoint, LocalRegistry, StaticDiscovery


def test_static_discovery_normalizes_addresses() -> None:
    endpoints = StaticDiscovery(["127.0.0.1:50051"]).discover()

    assert endpoints == [
        Endpoint(
            endpoint_id="127.0.0.1:50051",
            transport="grpc",
            address="127.0.0.1:50051",
            metadata={},
            source="static",
        )
    ]


def test_local_registry_registers_and_discovers_endpoint(tmp_path) -> None:
    registry = LocalRegistry(tmp_path)
    endpoint = Endpoint(
        endpoint_id="test server",
        transport="grpc",
        address="127.0.0.1:50051",
        metadata={"config_name": "bench"},
    )

    registry.register(endpoint)
    discovered = registry.discover()
    registry.unregister(endpoint.endpoint_id)

    assert len(discovered) == 1
    assert discovered[0].endpoint_id == "test server"
    assert discovered[0].transport == "grpc"
    assert discovered[0].address == "127.0.0.1:50051"
    assert discovered[0].metadata == {"config_name": "bench"}
    assert discovered[0].source == "local_registry"
    assert registry.discover() == []
