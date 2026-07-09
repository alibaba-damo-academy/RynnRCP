"""Measure RynnRCP Interface gRPC latency with fixed-size payloads.

Run one terminal as server:

    python -m rynnrcp.interface.grpc_latency_bench server --host 0.0.0.0 --port 50071

Run another terminal as client:

    python -m rynnrcp.interface.grpc_latency_bench client --target 127.0.0.1:50071

Or run both in one process:

    python -m rynnrcp.interface.grpc_latency_bench local --port 50071
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time
from dataclasses import dataclass
from typing import Any

from rynnrcp.interface.codec import InterfaceRequest, ok_response, pack_request, unpack_response
from rynnrcp.interface.grpc_transport import (
    REQUEST_PATH,
    GrpcServer,
    _default_options,
    _identity,
    _require_grpc,
)


DEFAULT_SIZES = [0, 1024, 4096, 16384, 65536, 262144, 1048576]


@dataclass
class Sample:
    total_ms: float
    pack_ms: float
    grpc_ms: float
    unpack_ms: float
    server_ms: float


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    server = sub.add_parser("server", help="run benchmark echo server")
    server.add_argument("--host", default="0.0.0.0")
    server.add_argument("--port", type=int, default=50071)
    server.add_argument("--workers", type=int, default=16)

    client = sub.add_parser("client", help="run benchmark client")
    client.add_argument("--target", required=True, help="host:port")
    client.add_argument("--samples", type=int, default=100)
    client.add_argument("--warmup", type=int, default=10)
    client.add_argument("--sizes", default=",".join(str(item) for item in DEFAULT_SIZES))
    client.add_argument("--pattern", choices=("echo", "ack"), default="echo")
    client.add_argument("--timeout-ms", type=int, default=5000)

    local = sub.add_parser("local", help="run benchmark server and client in one process")
    local.add_argument("--host", default="127.0.0.1")
    local.add_argument("--port", type=int, default=0)
    local.add_argument("--workers", type=int, default=16)
    local.add_argument("--samples", type=int, default=100)
    local.add_argument("--warmup", type=int, default=10)
    local.add_argument("--sizes", default=",".join(str(item) for item in DEFAULT_SIZES))
    local.add_argument("--pattern", choices=("echo", "ack"), default="echo")
    local.add_argument("--timeout-ms", type=int, default=5000)

    args = parser.parse_args(argv)
    if args.command == "server":
        return _run_server(args)
    if args.command == "local":
        return _run_local(args)
    return _run_client(args)


def _run_server(args: argparse.Namespace) -> int:
    server = GrpcServer(_handle_request, host=args.host, port=args.port, max_workers=args.workers)
    port = server.start()
    print(f"gRPC latency bench server listening on {args.host}:{port}", flush=True)
    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        print("\nstopping server...", flush=True)
        server.stop(0.2)
    return 0


def _run_local(args: argparse.Namespace) -> int:
    server = GrpcServer(_handle_request, host=args.host, port=args.port, max_workers=args.workers)
    port = server.start()
    print(f"gRPC latency bench local server listening on {args.host}:{port}", flush=True)
    try:
        client_args = argparse.Namespace(
            target=f"{args.host}:{port}",
            samples=args.samples,
            warmup=args.warmup,
            sizes=args.sizes,
            pattern=args.pattern,
            timeout_ms=args.timeout_ms,
        )
        return _run_client(client_args)
    finally:
        server.stop(0.2)


def _handle_request(request: InterfaceRequest) -> dict[str, Any]:
    payload = request.payload or {}
    if not isinstance(payload, dict):
        raise ValueError("payload must be a dict")
    data = payload.get("data", b"")
    if isinstance(data, bytearray):
        data = bytes(data)
    if not isinstance(data, bytes):
        raise ValueError("payload.data must be bytes")
    pattern = str(payload.get("pattern") or "echo")
    if pattern == "ack":
        return {"size_bytes": len(data)}
    if pattern == "echo":
        return {"size_bytes": len(data), "data": data}
    raise ValueError(f"unknown pattern: {pattern}")


def _run_client(args: argparse.Namespace) -> int:
    sizes = _parse_sizes(args.sizes)
    grpc = _require_grpc()
    channel = grpc.insecure_channel(args.target, options=_default_options())
    request_call = channel.unary_unary(
        REQUEST_PATH,
        request_serializer=_identity,
        response_deserializer=_identity,
    )
    print(
        f"target={args.target} pattern={args.pattern} encoding=bytes "
        f"samples={args.samples} warmup={args.warmup}",
        flush=True,
    )
    print(
        "size_bytes  wire_request  wire_response  avg_ms  p50_ms  p95_ms  p99_ms  "
        "min_ms  max_ms  pack_avg  grpc_avg  unpack_avg  server_avg",
        flush=True,
    )
    try:
        for size in sizes:
            row = _measure_size(
                request_call,
                size=size,
                pattern=args.pattern,
                samples=args.samples,
                warmup=args.warmup,
                timeout_ms=args.timeout_ms,
            )
            print(_format_row(row), flush=True)
    finally:
        close = getattr(channel, "close", None)
        if close is not None:
            result = close()
            wait = getattr(result, "result", None)
            if wait is not None:
                wait(timeout=1.0)
    return 0


def _measure_size(
    request_call: Any,
    *,
    size: int,
    pattern: str,
    samples: int,
    warmup: int,
    timeout_ms: int,
) -> dict[str, Any]:
    payload = _payload(size)
    total = max(1, int(samples)) + max(0, int(warmup))
    kept: list[Sample] = []
    wire_request = 0
    wire_response = 0
    for index in range(total):
        request = InterfaceRequest(
            method="bench.echo",
            payload={"pattern": pattern, "data": payload},
            timeout_ms=timeout_ms,
        )
        started = time.perf_counter()
        packed = pack_request(request)
        packed_at = time.perf_counter()
        try:
            raw = request_call(packed, timeout=float(timeout_ms) / 1000.0)
        except Exception as exc:
            grpc = _require_grpc()
            if isinstance(exc, grpc.RpcError):
                code = exc.code() if hasattr(exc, "code") else None
                detail = exc.details() if hasattr(exc, "details") else str(exc)
                raise RuntimeError(
                    "gRPC latency bench request failed. Make sure the target is a "
                    "grpc_latency_bench server, not a normal RynnRCP/SO101 server. "
                    "Start it with: python -m rynnrcp.interface.grpc_latency_bench "
                    "server --host 0.0.0.0 --port 50071. "
                    f"gRPC status={code} details={detail}"
                ) from exc
            raise
        response_started = time.perf_counter()
        response = unpack_response(raw)
        finished = time.perf_counter()
        if not response.ok:
            raise RuntimeError(response.message)
        if index >= warmup:
            server_timing = response.metadata.get("server_timing_ms") or {}
            kept.append(
                Sample(
                    total_ms=_ms(finished - started),
                    pack_ms=_ms(packed_at - started),
                    grpc_ms=_ms(response_started - packed_at),
                    unpack_ms=_ms(finished - response_started),
                    server_ms=float(server_timing.get("pre_pack_total") or 0.0),
                )
            )
            wire_request = len(packed)
            wire_response = len(raw)
    totals = [item.total_ms for item in kept]
    return {
        "size": size,
        "wire_request": wire_request,
        "wire_response": wire_response,
        "avg": statistics.fmean(totals),
        "p50": _percentile(totals, 50),
        "p95": _percentile(totals, 95),
        "p99": _percentile(totals, 99),
        "min": min(totals),
        "max": max(totals),
        "pack_avg": statistics.fmean(item.pack_ms for item in kept),
        "grpc_avg": statistics.fmean(item.grpc_ms for item in kept),
        "unpack_avg": statistics.fmean(item.unpack_ms for item in kept),
        "server_avg": statistics.fmean(item.server_ms for item in kept),
    }


def _payload(size: int) -> bytes:
    if size <= 0:
        return b""
    return os.urandom(int(size))


def _parse_sizes(value: str) -> list[int]:
    sizes: list[int] = []
    for item in str(value).split(","):
        item = item.strip()
        if item:
            sizes.append(int(item))
    if not sizes:
        raise ValueError("--sizes must contain at least one integer")
    return sizes


def _percentile(values: list[float], percentile: int) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * (float(percentile) / 100.0)
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _format_row(row: dict[str, Any]) -> str:
    return (
        f"{int(row['size']):10d}  "
        f"{int(row['wire_request']):12d}  "
        f"{int(row['wire_response']):13d}  "
        f"{row['avg']:7.2f}  "
        f"{row['p50']:6.2f}  "
        f"{row['p95']:6.2f}  "
        f"{row['p99']:6.2f}  "
        f"{row['min']:6.2f}  "
        f"{row['max']:6.2f}  "
        f"{row['pack_avg']:8.3f}  "
        f"{row['grpc_avg']:8.3f}  "
        f"{row['unpack_avg']:10.3f}  "
        f"{row['server_avg']:10.3f}"
    )


def _ms(seconds: float) -> float:
    return float(seconds) * 1000.0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
