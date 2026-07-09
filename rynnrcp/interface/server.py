"""Server-facing Interface API."""

from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from typing import Any

from rynnrcp.interface.discovery import Endpoint, LocalRegistry
from rynnrcp.interface.grpc_transport import GrpcServer, RequestHandler
from rynnrcp.interface.mdns import MdnsPublisher


@dataclass
class ServerInterface:
    """Run a gRPC Interface server with optional local/mDNS discovery."""

    handler: RequestHandler
    server_id: str
    server_instance_id: str | None = None
    config_name: str = "unknown"
    display_name: str = ""
    host: str = "0.0.0.0"
    port: int = 50051
    max_workers: int = 16
    register_local: bool = True
    publish_mdns: bool = False
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self._grpc = GrpcServer(
            self.handler,
            host=self.host,
            port=self.port,
            max_workers=self.max_workers,
        )
        if not self.server_instance_id:
            self.server_instance_id = uuid.uuid4().hex
        if not self.display_name:
            self.display_name = self.server_id
        self._registry = LocalRegistry()
        self._publisher: MdnsPublisher | None = None
        self._registered_endpoint: Endpoint | None = None

    @property
    def bound_port(self) -> int:
        return self._grpc.port

    @property
    def local_endpoint(self) -> Endpoint:
        return Endpoint(
            endpoint_id=self.server_id,
            transport="grpc",
            address=f"127.0.0.1:{self.bound_port}",
            metadata=self._endpoint_metadata(),
            source="local_registry",
        )

    def start(self) -> None:
        self._grpc.start()
        if self.register_local:
            self._registered_endpoint = self.local_endpoint
            self._registry.register(self._registered_endpoint)
        if self.publish_mdns:
            self._publisher = MdnsPublisher(
                endpoint_id=self.server_id,
                port=self.bound_port,
                metadata=self._endpoint_metadata(),
            )
            self._publisher.start()

    def stop(self) -> None:
        if self._publisher is not None:
            self._publisher.stop()
            self._publisher = None
        if self._registered_endpoint is not None:
            self._registry.unregister(self._registered_endpoint.endpoint_id)
            self._registered_endpoint = None
        self._grpc.stop()

    def wait_for_termination(self) -> None:
        self._grpc.wait_for_termination()

    def _endpoint_metadata(self) -> dict[str, Any]:
        metadata = {
            "server_id": self.server_id,
            "server_instance_id": self.server_instance_id,
            "config_name": self.config_name,
            "display_name": self.display_name,
            "protocol_version": 1,
            "transport": "grpc",
            "grpc_port": self.bound_port,
        }
        metadata.update(self.metadata)
        return metadata
