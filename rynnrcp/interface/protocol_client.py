"""Protocol-level client helpers built on top of ClientInterface."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Iterator, Mapping

from rynnrcp.interface.client import ClientInterface
from rynnrcp.interface.codec import InterfaceError, InterfaceResponse
from rynnrcp.interface.discovery import Endpoint
from rynnrcp.interface.grpc_transport import GrpcConnection
from rynnrcp.protocol.methods import (
    DELETE_COLLECTION,
    DELETE_RESOURCE,
    GET_COLLECTION_STATUS,
    GET_HEALTH,
    GET_MANIFEST,
    GET_OBSERVATIONS,
    GET_RESOURCE_INFO,
    LIST_ACTIONS,
    LIST_OBSERVATIONS,
    LIST_POLICIES,
    LIST_RESOURCES,
    LIST_RESOURCE_ENTRIES,
    PREPARE_RESOURCE_ARCHIVE,
    READ_RESOURCE,
    RUN_ACTION_CHUNK,
    SNAPSHOT_RESOURCE,
    START_COLLECTION,
    START_POLICY,
    STOP_ACTION,
    STOP_COLLECTION,
    STOP_POLICY,
    UPDATE_POLICY_INPUTS,
)


@dataclass(frozen=True)
class ServerManifest:
    """Normalized manifest returned by one RCP Interface server."""

    robot_id: str
    robot_name: str
    embodiment_type: str
    components: list[dict[str, Any]]
    observations: list[dict[str, Any]]
    actions: list[dict[str, Any]]
    capabilities: dict[str, Any]
    model_refs: dict[str, Any]
    metadata: dict[str, Any]
    endpoint: Endpoint | None = None

    @classmethod
    def from_payload(
        cls,
        payload: dict[str, Any],
        *,
        endpoint: Endpoint | None = None,
    ) -> "ServerManifest":
        return cls(
            robot_id=str(payload["robot_id"]),
            robot_name=str(payload["robot_name"]),
            embodiment_type=str(payload["embodiment_type"]),
            components=list(payload["components"]),
            observations=list(payload["observations"]),
            actions=list(payload["actions"]),
            capabilities=dict(payload.get("capabilities") or {}),
            model_refs=dict(payload.get("model_refs") or {}),
            metadata=dict(payload.get("metadata") or {}),
            endpoint=endpoint,
        )

    @property
    def can_provide_state(self) -> bool:
        return bool(self.capabilities.get("observations"))

    @property
    def can_accept_action(self) -> bool:
        return bool(self.capabilities.get("actions"))


class ServerSelectionError(RuntimeError):
    """Raised when a configured server cannot be resolved unambiguously."""


class RcpProtocolClient:
    """Small semantic wrapper over the raw Interface request API."""

    def __init__(self, connection: GrpcConnection):
        self._connection = connection

    @classmethod
    def connect(cls, endpoint: Endpoint | str, *, interface: ClientInterface | None = None) -> "RcpProtocolClient":
        client = interface or ClientInterface.with_defaults(local_registry=False)
        return cls(client.connect(endpoint))

    def close(self) -> None:
        self._connection.close()

    def ping(self, payload: Any = None, *, timeout_ms: int | None = 1000) -> InterfaceResponse:
        return self._connection.request("ping", payload, timeout_ms=timeout_ms)

    def request(
        self,
        method: str,
        payload: Any = None,
        *,
        timeout_ms: int | None = 1000,
    ) -> InterfaceResponse:
        return self._connection.request(method, payload, timeout_ms=timeout_ms)

    def get_manifest(self, *, timeout_ms: int | None = 1000) -> ServerManifest:
        response = self._connection.request(GET_MANIFEST.name, {}, timeout_ms=timeout_ms)
        if not response.ok or not isinstance(response.payload, dict):
            raise InterfaceError(f"{GET_MANIFEST.name} failed: {response.message}")
        return ServerManifest.from_payload(response.payload)

    def list_tools(self, *, timeout_ms: int | None = 1000) -> InterfaceResponse:
        return self._connection.request("list_tools", {}, timeout_ms=timeout_ms)

    def list_observations(self, *, timeout_ms: int | None = 1000) -> InterfaceResponse:
        return self._connection.request(LIST_OBSERVATIONS.name, {}, timeout_ms=timeout_ms)

    def get_observations(
        self,
        names: list[str],
        *,
        sync: bool = False,
        timeout_ms: int | None = 1000,
    ) -> InterfaceResponse:
        return self._connection.request(
            GET_OBSERVATIONS.name,
            {"names": list(names), "sync": bool(sync)},
            timeout_ms=timeout_ms,
        )

    def subscribe_observations(
        self,
        names: list[str],
        *,
        sync: bool = False,
        stream_hz: float = 30.0,
        timeout_ms: int | None = None,
    ) -> Iterator[InterfaceResponse]:
        return self._connection.subscribe(
            GET_OBSERVATIONS.name,
            {"names": list(names), "sync": bool(sync)},
            metadata={"stream_hz": float(stream_hz)},
            timeout_ms=timeout_ms,
        )

    def list_actions(self, *, timeout_ms: int | None = 1000) -> InterfaceResponse:
        return self._connection.request(LIST_ACTIONS.name, {}, timeout_ms=timeout_ms)

    def run_action_chunk(
        self,
        name: str,
        frames: list[dict[str, Any]],
        *,
        frame_rate: float,
        timeout_ms: int | None = 1000,
    ) -> InterfaceResponse:
        return self._connection.request(
            RUN_ACTION_CHUNK.name,
            {
                "name": str(name),
                "frames": list(frames),
                "frame_rate": float(frame_rate),
            },
            timeout_ms=timeout_ms,
        )

    def run_action_chunk_async(
        self,
        name: str,
        frames: list[dict[str, Any]],
        *,
        frame_rate: float,
        timeout_ms: int | None = None,
    ) -> Iterator[InterfaceResponse]:
        return self._connection.subscribe(
            RUN_ACTION_CHUNK.name,
            {
                "name": str(name),
                "frames": list(frames),
                "frame_rate": float(frame_rate),
            },
            metadata={"stream_once": True},
            timeout_ms=timeout_ms,
        )

    def stop_action(self, reason: str | None = None, *, timeout_ms: int | None = 1000) -> InterfaceResponse:
        payload = {} if reason is None else {"reason": str(reason)}
        return self._connection.request(STOP_ACTION.name, payload, timeout_ms=timeout_ms)

    def list_policies(self, *, timeout_ms: int | None = 1000) -> InterfaceResponse:
        return self._connection.request(LIST_POLICIES.name, {}, timeout_ms=timeout_ms)

    def start_policy(
        self,
        policy_id: str,
        *,
        runtime_inputs: Mapping[str, Any] | None = None,
        timeout_ms: int | None = 1000,
    ) -> InterfaceResponse:
        payload: dict[str, Any] = {"policy_id": str(policy_id)}
        if runtime_inputs is not None:
            payload["runtime_inputs"] = dict(runtime_inputs)
        return self._connection.request(START_POLICY.name, payload, timeout_ms=timeout_ms)

    def update_policy_inputs(
        self,
        *,
        policy_id: str | None = None,
        runtime_inputs: Mapping[str, Any],
        timeout_ms: int | None = 1000,
    ) -> InterfaceResponse:
        payload: dict[str, Any] = {"runtime_inputs": dict(runtime_inputs)}
        if policy_id is not None:
            payload["policy_id"] = str(policy_id)
        return self._connection.request(UPDATE_POLICY_INPUTS.name, payload, timeout_ms=timeout_ms)

    def stop_policy(
        self,
        policy_id: str | None = None,
        *,
        reason: str | None = None,
        timeout_ms: int | None = 1000,
    ) -> InterfaceResponse:
        payload: dict[str, Any] = {}
        if policy_id is not None:
            payload["policy_id"] = str(policy_id)
        if reason is not None:
            payload["reason"] = str(reason)
        return self._connection.request(STOP_POLICY.name, payload, timeout_ms=timeout_ms)

    def get_health(self, *, timeout_ms: int | None = 1000) -> InterfaceResponse:
        return self._connection.request(GET_HEALTH.name, {}, timeout_ms=timeout_ms)

    def subscribe_health(
        self,
        *,
        stream_hz: float = 2.0,
        timeout_ms: int | None = None,
    ) -> Iterator[InterfaceResponse]:
        return self._connection.subscribe(
            GET_HEALTH.name,
            {},
            metadata={"stream_hz": float(stream_hz)},
            timeout_ms=timeout_ms,
        )

    def get_resource_info(self, resource_id: str, *, timeout_ms: int | None = 1000) -> InterfaceResponse:
        return self._connection.request(GET_RESOURCE_INFO.name, {"resource_id": str(resource_id)}, timeout_ms=timeout_ms)

    def list_resources(
        self,
        *,
        domain: str | None = None,
        kind: str | None = None,
        cursor: str | None = None,
        limit: int | None = None,
        timeout_ms: int | None = 1000,
    ) -> InterfaceResponse:
        payload: dict[str, Any] = {}
        if domain is not None:
            payload["domain"] = str(domain)
        if kind is not None:
            payload["kind"] = str(kind)
        if cursor is not None:
            payload["cursor"] = str(cursor)
        if limit is not None:
            payload["limit"] = int(limit)
        return self._connection.request(LIST_RESOURCES.name, payload, timeout_ms=timeout_ms)

    def list_resource_entries(
        self,
        resource_id: str,
        *,
        recursive: bool = False,
        cursor: str | None = None,
        limit: int | None = None,
        timeout_ms: int | None = 1000,
    ) -> InterfaceResponse:
        payload: dict[str, Any] = {"resource_id": str(resource_id), "recursive": bool(recursive)}
        if cursor is not None:
            payload["cursor"] = str(cursor)
        if limit is not None:
            payload["limit"] = int(limit)
        return self._connection.request(LIST_RESOURCE_ENTRIES.name, payload, timeout_ms=timeout_ms)

    def read_resource(
        self,
        resource_id: str,
        *,
        offset: int = 0,
        limit: int | None = None,
        timeout_ms: int | None = 1000,
    ) -> InterfaceResponse:
        payload: dict[str, Any] = {"resource_id": str(resource_id), "offset": int(offset)}
        if limit is not None:
            payload["limit"] = int(limit)
        return self._connection.request(READ_RESOURCE.name, payload, timeout_ms=timeout_ms)

    def delete_resource(self, resource_id: str, *, timeout_ms: int | None = 1000) -> InterfaceResponse:
        return self._connection.request(DELETE_RESOURCE.name, {"resource_id": str(resource_id)}, timeout_ms=timeout_ms)

    def snapshot_resource(self, resource_id: str, *, timeout_ms: int | None = 1000) -> InterfaceResponse:
        return self._connection.request(SNAPSHOT_RESOURCE.name, {"resource_id": str(resource_id)}, timeout_ms=timeout_ms)

    def prepare_resource_archive(
        self,
        *,
        resource_id: str | None = None,
        resource_ids: list[str] | None = None,
        format: str = "zip",
        timeout_ms: int | None = 1000,
    ) -> InterfaceResponse:
        payload: dict[str, Any] = {"format": str(format)}
        if resource_id is not None:
            payload["resource_id"] = str(resource_id)
        if resource_ids is not None:
            payload["resource_ids"] = [str(item) for item in resource_ids]
        return self._connection.request(PREPARE_RESOURCE_ARCHIVE.name, payload, timeout_ms=timeout_ms)

    def start_collection(
        self,
        names: list[str],
        *,
        collection_id: str,
        episode_id: str,
        task_prompt: str,
        task_description: str,
        frame_rate: float | None = None,
        max_duration: float | None = None,
        metadata: Mapping[str, Any] | None = None,
        timeout_ms: int | None = 1000,
    ) -> InterfaceResponse:
        payload: dict[str, Any] = {
            "names": list(names),
            "collection_id": str(collection_id),
            "episode_id": str(episode_id),
            "task_prompt": str(task_prompt),
            "task_description": str(task_description),
        }
        if frame_rate is not None:
            payload["frame_rate"] = float(frame_rate)
        if max_duration is not None:
            payload["max_duration"] = float(max_duration)
        if metadata is not None:
            payload["metadata"] = dict(metadata)
        return self._connection.request(START_COLLECTION.name, payload, timeout_ms=timeout_ms)

    def stop_collection(self, *, timeout_ms: int | None = 1000) -> InterfaceResponse:
        return self._connection.request(STOP_COLLECTION.name, {}, timeout_ms=timeout_ms)

    def get_collection_status(self, *, timeout_ms: int | None = 1000) -> InterfaceResponse:
        return self._connection.request(GET_COLLECTION_STATUS.name, {}, timeout_ms=timeout_ms)

    def delete_collection(
        self,
        resource_id: str,
        *,
        timeout_ms: int | None = 1000,
    ) -> InterfaceResponse:
        payload = {"resource_id": str(resource_id)}
        return self._connection.request(
            DELETE_COLLECTION.name,
            payload,
            timeout_ms=timeout_ms,
        )


def discover_manifests(
    interface: ClientInterface | None = None,
    *,
    timeout_s: float = 1.5,
    request_timeout_ms: int = 1000,
    dedupe_instances: bool = True,
) -> list[ServerManifest]:
    """Discover endpoints and fetch manifests from reachable servers."""

    client = interface or ClientInterface.with_defaults(mdns=True)
    manifests = [
        manifest
        for endpoint in client.discover(timeout_s=timeout_s)
        if (manifest := _fetch_manifest(client, endpoint, request_timeout_ms)) is not None
    ]
    return _dedupe_manifests_by_instance(manifests) if dedupe_instances else manifests


def resolve_server_manifest(
    *,
    robot_id: str,
    embodiment_type: str | None = None,
    interface: ClientInterface | None = None,
    timeout_s: float = 1.5,
    request_timeout_ms: int = 1000,
) -> ServerManifest:
    """Resolve exactly one configured robot by persistent robot_id."""

    expected_id = str(robot_id).strip()
    if not expected_id:
        raise ServerSelectionError("robot_id is required")

    manifests = discover_manifests(
        interface=interface,
        timeout_s=timeout_s,
        request_timeout_ms=request_timeout_ms,
        dedupe_instances=True,
    )
    matches = [item for item in manifests if item.robot_id == expected_id]
    expected_embodiment = str(embodiment_type).strip() if embodiment_type else ""
    if expected_embodiment:
        matches = [item for item in matches if item.embodiment_type == expected_embodiment]
    if not matches:
        raise ServerSelectionError(f"robot_id {expected_id!r} was not found")

    if len({_instance_key(item) for item in matches}) > 1:
        addresses = ", ".join(sorted(str(item.endpoint.address) for item in matches if item.endpoint))
        raise ServerSelectionError(
            f"duplicate robot_id {expected_id!r} discovered from multiple robot instances: {addresses}"
        )
    return matches[0]


def connect_to_server(
    *,
    robot_id: str,
    embodiment_type: str | None = None,
    interface: ClientInterface | None = None,
    timeout_s: float = 1.5,
    request_timeout_ms: int = 1000,
) -> RcpProtocolClient:
    """Resolve and connect to one configured server."""

    client = interface or ClientInterface.with_defaults(mdns=True)
    manifest = resolve_server_manifest(
        robot_id=robot_id,
        embodiment_type=embodiment_type,
        interface=client,
        timeout_s=timeout_s,
        request_timeout_ms=request_timeout_ms,
    )
    if manifest.endpoint is None:
        raise ServerSelectionError(f"robot_id {robot_id!r} resolved without an endpoint")
    return RcpProtocolClient(client.connect(manifest.endpoint))


def _fetch_manifest(
    client: ClientInterface,
    endpoint: Endpoint,
    request_timeout_ms: int,
) -> ServerManifest | None:
    for candidate in _endpoint_address_candidates(endpoint):
        conn = None
        try:
            conn = client.connect(candidate)
            response = conn.request(GET_MANIFEST.name, {}, timeout_ms=request_timeout_ms)
        except Exception:
            continue
        finally:
            if conn is not None:
                conn.close()
        if response.ok and isinstance(response.payload, dict):
            return ServerManifest.from_payload(response.payload, endpoint=candidate)
    return None


def _endpoint_address_candidates(endpoint: Endpoint) -> list[Endpoint]:
    """Return endpoint variants sorted by routing metric (lower is better)."""
    candidates_with_metrics: list[tuple[Endpoint, int]] = []
    
    # Parse metrics from mdns_addresses metadata
    mdns_addr = str(endpoint.metadata.get("mdns_addresses") or "")
    ip_metrics: dict[str, int] = {}
    for entry in mdns_addr.split(","):
        if ":" in entry:
            parts = entry.rsplit(":", 1)
            if len(parts) == 2:
                ip, metric_str = parts
                try:
                    ip_metrics[ip] = int(metric_str)
                except ValueError:
                    pass
    
    # Add primary endpoint with its metric (or default 999)
    primary_host = endpoint.address.rsplit(":", 1)[0] if ":" in endpoint.address else endpoint.address
    primary_metric = ip_metrics.get(primary_host, 999)
    candidates_with_metrics.append((endpoint, primary_metric))
    
    # Add alternative addresses with their metrics
    for ip, metric in ip_metrics.items():
        if ip != primary_host and ":" in endpoint.address:
            address = f"{ip}:{endpoint.address.rsplit(':', 1)[1]}"
            candidates_with_metrics.append((replace(endpoint, address=address), metric))
    
    # Sort by metric (ascending) and return endpoints
    candidates_with_metrics.sort(key=lambda x: x[1])
    return [ep for ep, _ in candidates_with_metrics]


def _dedupe_manifests_by_instance(manifests: list[ServerManifest]) -> list[ServerManifest]:
    selected: dict[str, ServerManifest] = {}
    for item in manifests:
        key = _instance_key(item)
        current = selected.get(key)
        if current is None or _endpoint_priority(item.endpoint) < _endpoint_priority(current.endpoint):
            selected[key] = item
    return list(selected.values())


def _endpoint_priority(endpoint: Endpoint | None) -> int:
    if endpoint is None:
        return 99
    return {"static": 0, "local_registry": 1, "mdns": 2}.get(endpoint.source, 50)


def _instance_key(manifest: ServerManifest) -> str:
    if manifest.endpoint is None:
        return manifest.robot_id
    server_instance_id = str(manifest.endpoint.metadata.get("server_instance_id") or "").strip()
    if server_instance_id:
        return f"{manifest.robot_id}:{manifest.endpoint.transport}:instance:{server_instance_id}"
    return f"{manifest.robot_id}:{manifest.endpoint.transport}:{manifest.endpoint.address}"
