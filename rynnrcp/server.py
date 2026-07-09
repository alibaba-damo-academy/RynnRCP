"""RynnRCP server entry point.

The server is the long-lived robot access process. It owns the runtime graph
and exposes the configured robot capabilities through the Interface protocol.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any
import uuid

from rynnrcp.config.runtime_config import RuntimeConfig
from rynnrcp.interface.client import ClientInterface
from rynnrcp.interface.method_dispatcher import InterfaceMethodDispatcher
from rynnrcp.interface.server import ServerInterface
from rynnrcp.interface.protocol_client import ServerManifest, discover_manifests


RuntimeFactory = Callable[[RuntimeConfig], Any]
DUPLICATE_CHECK_TIMEOUT_S = 0.5
DUPLICATE_CHECK_REQUEST_TIMEOUT_MS = 500


@dataclass
class RynnRCPServer:
    """Start a robot runtime and expose it as one discoverable RynnRCP server."""

    config: str | Mapping[str, Any]
    runtime_factory: RuntimeFactory | None = None
    log_session_id: str | None = None

    def __post_init__(self) -> None:
        self._runtime: Any | None = None
        self._interface_server: ServerInterface | None = None
        self._server_instance_id = uuid.uuid4().hex

    @property
    def runtime(self) -> Any | None:
        return self._runtime

    @property
    def interface_server(self) -> ServerInterface | None:
        return self._interface_server

    @property
    def server_instance_id(self) -> str:
        return self._server_instance_id

    @property
    def bound_port(self) -> int:
        return self._interface_server.bound_port if self._interface_server is not None else 0

    def start(self) -> None:
        if self._runtime is not None or self._interface_server is not None:
            raise RuntimeError("RynnRCPServer is already started")

        runtime = self._create_runtime()
        server_cfg = _runtime_server_config(runtime)
        interface_cfg = _interface_config(server_cfg)
        _reject_duplicate_robot_id(
            str(server_cfg["id"]),
            local_registry=bool(interface_cfg.get("local_registry", True)),
            mdns=bool(interface_cfg.get("mdns", True)),
        )
        runtime.start()
        try:
            metadata = _server_metadata(runtime, self.config, server_cfg)
            handler = InterfaceMethodDispatcher(
                runtime.bus,
                server_id=str(server_cfg["id"]),
                server_instance_id=self._server_instance_id,
                config_name=str(server_cfg["config_name"]),
                display_name=str(server_cfg.get("display_name") or server_cfg["id"]),
                capabilities=dict(server_cfg.get("capabilities") or {}),
                metadata=metadata,
            )
            interface_server = _create_interface_server(
                handler=handler,
                server_cfg=server_cfg,
                interface_cfg=interface_cfg,
                instance_id=self._server_instance_id,
                metadata=metadata,
            )
            interface_server.start()
        except Exception:
            runtime.stop()
            raise

        self._runtime = runtime
        self._interface_server = interface_server

    def stop(self) -> None:
        interface_server = self._interface_server
        runtime = self._runtime
        self._interface_server = None
        self._runtime = None

        if interface_server is not None:
            interface_server.stop()
        if runtime is not None:
            runtime.stop()

    def wait_for_termination(self) -> None:
        if self._interface_server is None:
            raise RuntimeError("RynnRCPServer is not started")
        self._interface_server.wait_for_termination()

    def _create_runtime(self) -> Any:
        if isinstance(self.config, Mapping):
            runtime_config = RuntimeConfig.from_mapping(self.config, log_session_id=self.log_session_id)
        else:
            runtime_config = RuntimeConfig.load(str(self.config), log_session_id=self.log_session_id)

        if self.runtime_factory is not None:
            return self.runtime_factory(runtime_config)

        from rynnrcp.runtime import Runtime

        return Runtime(runtime_config)

def _create_interface_server(
    *,
    handler: InterfaceMethodDispatcher,
    server_cfg: Mapping[str, Any],
    interface_cfg: Mapping[str, Any],
    instance_id: str,
    metadata: Mapping[str, Any],
) -> ServerInterface:
    return ServerInterface(
        handler,
        server_id=str(server_cfg["id"]),
        server_instance_id=instance_id,
        config_name=str(server_cfg["config_name"]),
        display_name=str(server_cfg.get("display_name") or server_cfg["id"]),
        host=str(interface_cfg.get("host") or "0.0.0.0"),
        port=int(interface_cfg.get("port") or 0),
        max_workers=int(interface_cfg.get("workers") or 16),
        register_local=bool(interface_cfg.get("local_registry", True)),
        publish_mdns=bool(interface_cfg.get("mdns", True)),
        metadata=dict(metadata),
    )


def _interface_config(server_cfg: Mapping[str, Any]) -> dict[str, Any]:
    value = server_cfg.get("interface")
    return dict(value) if isinstance(value, Mapping) else {}


def _reject_duplicate_robot_id(
    robot_id: str,
    *,
    local_registry: bool,
    mdns: bool,
) -> None:
    if not local_registry and not mdns:
        return
    client = ClientInterface.with_defaults(local_registry=local_registry, mdns=mdns)
    duplicates = [
        item
        for item in discover_manifests(
            interface=client,
            timeout_s=DUPLICATE_CHECK_TIMEOUT_S,
            request_timeout_ms=DUPLICATE_CHECK_REQUEST_TIMEOUT_MS,
            dedupe_instances=True,
        )
        if item.robot_id == robot_id
    ]
    if not duplicates:
        return
    addresses = ", ".join(_manifest_address(item) for item in duplicates)
    raise RuntimeError(
        f"robot_id {robot_id!r} is already online at {addresses}; "
        "use a unique robot_id for each running robot server"
    )


def _manifest_address(manifest: ServerManifest) -> str:
    if manifest.endpoint is None:
        return manifest.robot_id
    return f"{manifest.endpoint.address} ({manifest.endpoint.source})"


def _runtime_server_config(runtime: Any) -> Mapping[str, Any]:
    config = getattr(runtime, "_config", None)
    if not isinstance(config, Mapping):
        raise RuntimeError("runtime config is not available")
    manifest = config.get("manifest")
    if not isinstance(manifest, Mapping):
        raise RuntimeError("manifest config is required")
    server = config.get("server") or {}
    if not isinstance(server, Mapping):
        raise RuntimeError("server config must be a mapping")
    result = dict(server)
    result["id"] = str(manifest["robot_id"])
    result["display_name"] = str(manifest["robot_name"])
    result["capabilities"] = dict(manifest.get("capabilities") or {})
    if "config_name" not in result:
        result["config_name"] = result["id"]
    return result


def _server_metadata(
    runtime: Any,
    config_source: str | Mapping[str, Any],
    server_cfg: Mapping[str, Any],
) -> dict[str, Any]:
    config = getattr(runtime, "_config", None)
    metadata = dict(server_cfg.get("metadata") or {})
    metadata["runtime_interface"] = True
    if isinstance(config, Mapping):
        manifest = config.get("manifest")
        if isinstance(manifest, Mapping):
            metadata["robot_id"] = str(manifest.get("robot_id") or "")
    if isinstance(config_source, str):
        metadata["config_path"] = str(Path(config_source))
    return metadata
