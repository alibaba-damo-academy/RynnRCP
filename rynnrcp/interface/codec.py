"""MessagePack envelope helpers for RynnRCP Interface."""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Mapping

import msgpack


PROTOCOL_VERSION = 1

STATUS_OK = 0
STATUS_ERROR = 1
STATUS_BAD_REQUEST = 2
STATUS_TIMEOUT = 3
STATUS_NOT_FOUND = 4


class InterfaceError(RuntimeError):
    """Base error raised by Interface client/server helpers."""


class MethodNotFoundError(InterfaceError):
    """Raised by server adapters when a method is unknown."""


@dataclass(frozen=True)
class InterfaceRequest:
    """One request envelope carried by a TransportBackend."""

    method: str
    payload: Any = None
    request_id: str = field(default_factory=lambda: uuid.uuid4().hex)
    metadata: dict[str, Any] = field(default_factory=dict)
    timeout_ms: int | None = None
    version: int = PROTOCOL_VERSION

    def to_dict(self) -> dict[str, Any]:
        data: dict[str, Any] = {
            "version": int(self.version),
            "request_id": str(self.request_id),
            "method": str(self.method),
            "payload": self.payload,
            "metadata": dict(self.metadata),
        }
        if self.timeout_ms is not None:
            data["timeout_ms"] = int(self.timeout_ms)
        return data


@dataclass(frozen=True)
class InterfaceResponse:
    """One response envelope carried by a TransportBackend."""

    request_id: str
    status: int = STATUS_OK
    message: str = "OK"
    payload: Any = None
    payload_refs: list[dict[str, Any]] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)
    version: int = PROTOCOL_VERSION

    @property
    def ok(self) -> bool:
        return int(self.status) == STATUS_OK

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": int(self.version),
            "request_id": str(self.request_id),
            "status": int(self.status),
            "message": str(self.message),
            "payload": self.payload,
            "payload_refs": list(self.payload_refs),
            "metadata": dict(self.metadata),
        }


def pack_message(message: Mapping[str, Any]) -> bytes:
    """Encode one envelope map as MessagePack bytes."""
    return msgpack.packb(dict(message), use_bin_type=True)


def unpack_message(data: bytes | bytearray | memoryview) -> dict[str, Any]:
    """Decode one MessagePack envelope map."""
    value = msgpack.unpackb(bytes(data), raw=False, strict_map_key=False)
    if not isinstance(value, dict):
        raise InterfaceError("Interface envelope must be a MessagePack map")
    return value


def pack_request(request: InterfaceRequest) -> bytes:
    return pack_message(request.to_dict())


def unpack_request(data: bytes | bytearray | memoryview) -> InterfaceRequest:
    envelope = unpack_message(data)
    method = envelope.get("method")
    if not isinstance(method, str) or not method:
        raise InterfaceError("Interface request missing non-empty method")
    request_id = envelope.get("request_id") or uuid.uuid4().hex
    metadata = envelope.get("metadata") or {}
    if not isinstance(metadata, dict):
        raise InterfaceError("Interface request metadata must be a map")
    timeout_ms = envelope.get("timeout_ms")
    if timeout_ms is not None:
        timeout_ms = int(timeout_ms)
    return InterfaceRequest(
        version=int(envelope.get("version", PROTOCOL_VERSION)),
        request_id=str(request_id),
        method=method,
        payload=envelope.get("payload"),
        metadata=metadata,
        timeout_ms=timeout_ms,
    )


def pack_response(response: InterfaceResponse) -> bytes:
    return pack_message(response.to_dict())


def unpack_response(data: bytes | bytearray | memoryview) -> InterfaceResponse:
    envelope = unpack_message(data)
    request_id = envelope.get("request_id")
    if not isinstance(request_id, str) or not request_id:
        raise InterfaceError("Interface response missing non-empty request_id")
    metadata = envelope.get("metadata") or {}
    payload_refs = envelope.get("payload_refs") or []
    if not isinstance(metadata, dict):
        raise InterfaceError("Interface response metadata must be a map")
    if not isinstance(payload_refs, list):
        raise InterfaceError("Interface response payload_refs must be a list")
    return InterfaceResponse(
        version=int(envelope.get("version", PROTOCOL_VERSION)),
        request_id=request_id,
        status=int(envelope.get("status", STATUS_ERROR)),
        message=str(envelope.get("message", "")),
        payload=envelope.get("payload"),
        payload_refs=payload_refs,
        metadata=metadata,
    )


def ok_response(
    request_id: str,
    payload: Any = None,
    *,
    metadata: Mapping[str, Any] | None = None,
) -> InterfaceResponse:
    return InterfaceResponse(
        request_id=request_id,
        status=STATUS_OK,
        message="OK",
        payload=payload,
        metadata=dict(metadata or {}),
    )


def error_response(
    request_id: str,
    message: str,
    *,
    status: int = STATUS_ERROR,
    metadata: Mapping[str, Any] | None = None,
) -> InterfaceResponse:
    return InterfaceResponse(
        request_id=request_id,
        status=status,
        message=message,
        payload=None,
        metadata=dict(metadata or {}),
    )


def now_ms() -> int:
    return int(time.time() * 1000)
