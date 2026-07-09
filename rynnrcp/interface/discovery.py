"""Endpoint discovery helpers for RynnRCP Interface."""

from __future__ import annotations

import json
import os
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping

from rynnrcp.utils import safe_name


REGISTRY_ENV = "RYNNRCP_INTERFACE_REGISTRY_DIR"


@dataclass(frozen=True)
class Endpoint:
    """A discovered Interface endpoint."""

    endpoint_id: str
    transport: str
    address: str
    metadata: dict[str, Any] = field(default_factory=dict)
    source: str = "static"

    def to_dict(self) -> dict[str, Any]:
        return {
            "endpoint_id": self.endpoint_id,
            "transport": self.transport,
            "address": self.address,
            "metadata": dict(self.metadata),
            "source": self.source,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "Endpoint":
        metadata = data.get("metadata") or {}
        if not isinstance(metadata, dict):
            metadata = {"value": metadata}
        return cls(
            endpoint_id=str(data.get("endpoint_id") or data.get("server_id") or data["address"]),
            transport=str(data.get("transport") or "grpc"),
            address=str(data["address"]),
            metadata=metadata,
            source=str(data.get("source") or "static"),
        )


def default_registry_dir() -> Path:
    return Path(
        os.environ.get(
            REGISTRY_ENV,
            str(Path(tempfile.gettempdir()) / "rynnrcp-interface-registry"),
        )
    )


class StaticDiscovery:
    """Discovery provider backed by explicit endpoint addresses."""

    def __init__(self, endpoints: Iterable[str | Endpoint | Mapping[str, Any]]):
        self._endpoints = [self._normalize(item) for item in endpoints]

    def discover(self, timeout_s: float | None = None) -> list[Endpoint]:
        return list(self._endpoints)

    @staticmethod
    def _normalize(item: str | Endpoint | Mapping[str, Any]) -> Endpoint:
        if isinstance(item, Endpoint):
            return item
        if isinstance(item, str):
            return Endpoint(
                endpoint_id=item,
                transport="grpc",
                address=item,
                metadata={},
                source="static",
            )
        return Endpoint.from_dict({**dict(item), "source": item.get("source", "static")})


class LocalRegistry:
    """Local endpoint registry for same-machine discovery."""

    def __init__(self, registry_dir: str | Path | None = None):
        self.registry_dir = Path(registry_dir) if registry_dir else default_registry_dir()

    def register(self, endpoint: Endpoint, *, ttl_s: float | None = None) -> Path:
        self.registry_dir.mkdir(parents=True, exist_ok=True)
        path = self._endpoint_path(endpoint.endpoint_id)
        payload = endpoint.to_dict()
        payload["registered_at"] = time.time()
        payload["pid"] = os.getpid()
        if ttl_s is not None:
            payload["expires_at"] = time.time() + float(ttl_s)
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        return path

    def unregister(self, endpoint_id: str) -> None:
        try:
            self._endpoint_path(endpoint_id).unlink()
        except FileNotFoundError:
            pass

    def discover(self, timeout_s: float | None = None) -> list[Endpoint]:
        if not self.registry_dir.exists():
            return []
        now = time.time()
        endpoints: list[Endpoint] = []
        for path in sorted(self.registry_dir.glob("*.json")):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                continue
            expires_at = data.get("expires_at")
            if expires_at is not None and float(expires_at) < now:
                continue
            data["source"] = "local_registry"
            try:
                endpoints.append(Endpoint.from_dict(data))
            except (KeyError, TypeError, ValueError):
                continue
        return endpoints

    def _endpoint_path(self, endpoint_id: str) -> Path:
        return self.registry_dir / f"{safe_name(endpoint_id)}.json"
