"""RynnRCP Interface communication SDK."""

from rynnrcp.interface.client import ClientInterface
from rynnrcp.interface.codec import (
    STATUS_BAD_REQUEST,
    STATUS_ERROR,
    STATUS_NOT_FOUND,
    STATUS_OK,
    STATUS_TIMEOUT,
    InterfaceError,
    InterfaceRequest,
    InterfaceResponse,
    MethodNotFoundError,
)
from rynnrcp.interface.discovery import Endpoint, LocalRegistry, StaticDiscovery
from rynnrcp.interface.grpc_transport import GrpcConnection, GrpcServer
from rynnrcp.interface.method_dispatcher import InterfaceMethodDispatcher
from rynnrcp.interface.mdns import MdnsDiscovery
from rynnrcp.interface.protocol_client import (
    RcpProtocolClient,
    ServerManifest,
    ServerSelectionError,
    connect_to_server,
    discover_manifests,
    resolve_server_manifest,
)
from rynnrcp.interface.server import ServerInterface

__all__ = [
    "ClientInterface",
    "Endpoint",
    "GrpcConnection",
    "GrpcServer",
    "InterfaceError",
    "InterfaceMethodDispatcher",
    "InterfaceRequest",
    "InterfaceResponse",
    "LocalRegistry",
    "MdnsDiscovery",
    "MethodNotFoundError",
    "RcpProtocolClient",
    "ServerInterface",
    "ServerManifest",
    "ServerSelectionError",
    "StaticDiscovery",
    "STATUS_BAD_REQUEST",
    "STATUS_ERROR",
    "STATUS_NOT_FOUND",
    "STATUS_OK",
    "STATUS_TIMEOUT",
    "connect_to_server",
    "discover_manifests",
    "resolve_server_manifest",
]
