"""gRPC TransportBackend for RynnRCP Interface.

The Python prototype uses gRPC generic handlers with raw MessagePack Interface
envelopes from ``codec.py``. Unary calls use ``Request``; server-streaming
subscriptions use ``Subscribe``.
"""

from __future__ import annotations

import logging
import time
from concurrent import futures
from dataclasses import replace
from typing import Any, Callable, Mapping

from rynnrcp.interface.codec import (
    STATUS_BAD_REQUEST,
    STATUS_ERROR,
    STATUS_NOT_FOUND,
    InterfaceError,
    InterfaceRequest,
    InterfaceResponse,
    MethodNotFoundError,
    error_response,
    ok_response,
    pack_request,
    pack_response,
    unpack_request,
    unpack_response,
)
from rynnrcp.interface.discovery import Endpoint


logger = logging.getLogger(__name__)

SERVICE_NAME = "rynnrcp.interface.v1.RcpInterface"
REQUEST_METHOD = "Request"
SUBSCRIBE_METHOD = "Subscribe"
REQUEST_PATH = f"/{SERVICE_NAME}/{REQUEST_METHOD}"
SUBSCRIBE_PATH = f"/{SERVICE_NAME}/{SUBSCRIBE_METHOD}"

RequestHandler = Callable[[InterfaceRequest], Any]


class GrpcConnection:
    """Client-side gRPC connection for Interface request/response calls."""

    def __init__(
        self,
        address: str,
        *,
        options: IterableOption | None = None,
    ):
        grpc = _require_grpc()
        self.address = address
        self._channel = grpc.insecure_channel(address, options=list(options or _default_options()))
        self._request = self._channel.unary_unary(
            REQUEST_PATH,
            request_serializer=_identity,
            response_deserializer=_identity,
        )
        self._subscribe = self._channel.unary_stream(
            SUBSCRIBE_PATH,
            request_serializer=_identity,
            response_deserializer=_identity,
        )

    def request(
        self,
        method: str,
        payload: Any = None,
        *,
        metadata: Mapping[str, Any] | None = None,
        timeout_ms: int | None = None,
    ) -> InterfaceResponse:
        request = InterfaceRequest(
            method=method,
            payload=payload,
            metadata=dict(metadata or {}),
            timeout_ms=timeout_ms,
        )
        started = time.perf_counter()
        packed = pack_request(request)
        packed_at = time.perf_counter()
        try:
            raw = self._request(
                packed,
                timeout=None if timeout_ms is None else float(timeout_ms) / 1000.0,
            )
            response_started = time.perf_counter()
        except Exception as exc:
            grpc = _require_grpc()
            if isinstance(exc, grpc.RpcError):
                detail = exc.details() if hasattr(exc, "details") else str(exc)
                raise InterfaceError(f"gRPC request failed: {detail}") from exc
            raise
        response = unpack_response(raw)
        finished = time.perf_counter()
        if response.request_id != request.request_id:
            raise InterfaceError(
                f"response request_id mismatch: {response.request_id} != {request.request_id}"
            )
        return _with_metadata(
            response,
            "client_timing_ms",
            {
                "pack_request": _ms(packed_at - started),
                "grpc_call": _ms(response_started - packed_at),
                "unpack_response": _ms(finished - response_started),
                "total": _ms(finished - started),
            },
        )

    def close(self) -> None:
        close = getattr(self._channel, "close", None)
        if close is None:
            return
        result = close()
        wait = getattr(result, "result", None)
        if wait is not None:
            wait(timeout=1.0)

    def subscribe(
        self,
        method: str,
        payload: Any = None,
        *,
        metadata: Mapping[str, Any] | None = None,
        timeout_ms: int | None = None,
    ) -> "GrpcResponseStream":
        request = InterfaceRequest(
            method=method,
            payload=payload,
            metadata=dict(metadata or {}),
            timeout_ms=timeout_ms,
        )
        raw_responses = self._subscribe(
            pack_request(request),
            timeout=None if timeout_ms is None else float(timeout_ms) / 1000.0,
        )
        return GrpcResponseStream(raw_responses, request.request_id)


class GrpcResponseStream:
    """Iterator of Interface responses from a server-streaming gRPC call."""

    def __init__(self, raw_responses: Any, request_id: str) -> None:
        self._raw_responses = raw_responses
        self._request_id = request_id

    def __iter__(self) -> "GrpcResponseStream":
        return self

    def __next__(self) -> InterfaceResponse:
        try:
            response = unpack_response(next(self._raw_responses))
        except Exception as exc:
            grpc = _require_grpc()
            if isinstance(exc, grpc.RpcError):
                detail = exc.details() if hasattr(exc, "details") else str(exc)
                raise InterfaceError(f"gRPC subscribe failed: {detail}") from exc
            raise
        if response.request_id != self._request_id:
            raise InterfaceError(
                f"stream response request_id mismatch: {response.request_id} != {self._request_id}"
            )
        return response

    def cancel(self) -> None:
        cancel = getattr(self._raw_responses, "cancel", None)
        if cancel is not None:
            cancel()


class GrpcServer:
    """Server-side gRPC wrapper for Interface request handlers."""

    def __init__(
        self,
        handler: RequestHandler,
        *,
        host: str = "0.0.0.0",
        port: int = 50051,
        max_workers: int = 16,
        options: IterableOption | None = None,
    ):
        grpc = _require_grpc()
        self._handler = handler
        self.host = host
        self.port = int(port)
        self._server = grpc.server(
            futures.ThreadPoolExecutor(max_workers=max_workers),
            options=list(options or _default_options()),
        )
        method_handler = grpc.unary_unary_rpc_method_handler(
            self._handle_bytes,
            request_deserializer=_identity,
            response_serializer=_identity,
        )
        subscribe_handler = grpc.unary_stream_rpc_method_handler(
            self._handle_subscribe_bytes,
            request_deserializer=_identity,
            response_serializer=_identity,
        )
        generic_handler = grpc.method_handlers_generic_handler(
            SERVICE_NAME,
            {
                REQUEST_METHOD: method_handler,
                SUBSCRIBE_METHOD: subscribe_handler,
            },
        )
        self._server.add_generic_rpc_handlers((generic_handler,))
        self._started_at = 0.0

    @property
    def address(self) -> str:
        return f"{self.host}:{self.port}"

    @property
    def started_at(self) -> float:
        return self._started_at

    def start(self) -> int:
        bound_port = self._server.add_insecure_port(self.address)
        if bound_port <= 0:
            raise InterfaceError(f"failed to bind gRPC Interface server at {self.address}")
        self.port = int(bound_port)
        self._server.start()
        self._started_at = time.time()
        logger.info("RynnRCP Interface gRPC server listening on %s", self.address)
        return self.port

    def stop(self, grace_s: float = 1.0) -> None:
        self._server.stop(grace_s).wait(timeout=max(float(grace_s), 0.1) + 1.0)

    def wait_for_termination(self) -> None:
        self._server.wait_for_termination()

    def _handle_bytes(self, raw_request: bytes, context: Any) -> bytes:
        started = time.perf_counter()
        try:
            request = unpack_request(raw_request)
        except Exception as exc:
            logger.warning("Bad Interface request envelope: %s", exc)
            return pack_response(
                error_response(
                    "",
                    f"bad request: {exc}",
                    status=STATUS_BAD_REQUEST,
                )
            )
        unpacked_at = time.perf_counter()
        response = self._dispatch_request(request, unpacked_at=unpacked_at)
        response = _with_metadata(
            response,
            "server_timing_ms",
            {
                "unpack_request": _ms(unpacked_at - started),
                **dict(response.metadata.get("server_timing_ms") or {}),
            },
        )
        return pack_response(response)

    def _handle_subscribe_bytes(self, raw_request: bytes, context: Any) -> Any:
        try:
            request = unpack_request(raw_request)
        except Exception as exc:
            logger.warning("Bad Interface subscribe request envelope: %s", exc)
            yield pack_response(
                error_response(
                    "",
                    f"bad request: {exc}",
                    status=STATUS_BAD_REQUEST,
                )
            )
            return

        if _stream_once(request.metadata):
            response = self._dispatch_request(request, unpacked_at=time.perf_counter())
            response = _with_metadata(
                response,
                "server_stream_timing_ms",
                {
                    "sequence": 0,
                    "stream_once": True,
                },
            )
            yield pack_response(response)
            return

        hz = _stream_hz(request.metadata)
        interval = 1.0 / hz
        next_tick = time.monotonic()
        sequence = 0
        while context.is_active():
            started = time.perf_counter()
            response = self._dispatch_request(request, unpacked_at=started)
            response = _with_metadata(
                response,
                "server_stream_timing_ms",
                {
                    "sequence": sequence,
                    "stream_hz": hz,
                },
            )
            yield pack_response(response)
            sequence += 1
            if not response.ok:
                return

            next_tick += interval
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()

    def _dispatch_request(self, request: InterfaceRequest, *, unpacked_at: float) -> InterfaceResponse:
        try:
            result = self._handler(request)
            if isinstance(result, InterfaceResponse):
                response = result
            else:
                response = ok_response(request.request_id, result)
        except MethodNotFoundError as exc:
            response = error_response(request.request_id, str(exc), status=STATUS_NOT_FOUND)
        except InterfaceError as exc:
            response = error_response(request.request_id, str(exc), status=STATUS_ERROR)
        except Exception as exc:
            logger.exception(
                "Interface handler failed for method=%s error_type=%s error=%s",
                request.method,
                type(exc).__name__,
                exc,
            )
            response = error_response(request.request_id, str(exc), status=STATUS_ERROR)
        handled_at = time.perf_counter()
        response = _with_metadata(
            response,
            "server_timing_ms",
            {
                "handler": _ms(handled_at - unpacked_at),
            },
        )
        return response


def connect_grpc(endpoint: Endpoint | str, **kwargs: Any) -> GrpcConnection:
    if isinstance(endpoint, Endpoint):
        if endpoint.transport != "grpc":
            raise InterfaceError(f"unsupported endpoint transport: {endpoint.transport}")
        address = endpoint.address
    else:
        address = str(endpoint)
    return GrpcConnection(address, **kwargs)


IterableOption = list[tuple[str, Any]] | tuple[tuple[str, Any], ...]


def _default_options() -> list[tuple[str, Any]]:
    return [
        ("grpc.max_send_message_length", 64 * 1024 * 1024),
        ("grpc.max_receive_message_length", 64 * 1024 * 1024),
    ]


def _identity(data: bytes) -> bytes:
    return data


def _ms(seconds: float) -> float:
    return round(float(seconds) * 1000.0, 3)


def _stream_hz(metadata: Mapping[str, Any]) -> float:
    value = metadata.get("stream_hz", 30.0)
    try:
        hz = float(value)
    except (TypeError, ValueError):
        hz = 30.0
    return max(1.0, min(240.0, hz))


def _stream_once(metadata: Mapping[str, Any]) -> bool:
    return str(metadata.get("stream_once", "")).strip().lower() in {"1", "true", "yes", "on"}


def _with_metadata(response: InterfaceResponse, key: str, value: Mapping[str, Any]) -> InterfaceResponse:
    metadata = dict(response.metadata)
    metadata[key] = dict(value)
    return replace(response, metadata=metadata)


def _require_grpc() -> Any:
    try:
        import grpc
    except ModuleNotFoundError as exc:
        raise RuntimeError("gRPC transport requires grpcio. Install with: pip install grpcio") from exc
    return grpc
