#!/usr/bin/env python3
"""Run integration benchmarks against real devices exposed by RynnRCP."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import shutil
import signal
import statistics
import subprocess
import sys
import threading
import time
from dataclasses import asdict, is_dataclass, replace
from datetime import datetime, timezone
from importlib import metadata
from pathlib import Path
from typing import Any


class JsonlRecorder:
    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = path.open("w", encoding="utf-8")
        self._lock = threading.Lock()

    def write(self, value: dict[str, Any]) -> None:
        with self._lock:
            self._stream.write(json.dumps(value, ensure_ascii=False) + "\n")
            self._stream.flush()

    def close(self) -> None:
        with self._lock:
            self._stream.close()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def percentile(values: list[float], level: int) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * level / 100.0
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def summarize(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "avg": statistics.fmean(values),
        "p50": percentile(values, 50),
        "p95": percentile(values, 95),
        "p99": percentile(values, 99),
        "min": min(values),
        "max": max(values),
    }


def safe_json(value: Any, depth: int = 0) -> Any:
    if depth > 8:
        return {"type": type(value).__name__, "truncated": True}
    if value is None or isinstance(value, (bool, int, float)):
        return value
    if isinstance(value, str):
        return value if len(value) <= 2000 else value[:2000] + "…"
    if isinstance(value, (bytes, bytearray, memoryview)):
        return {"type": "bytes", "size_bytes": len(value)}
    if is_dataclass(value):
        return safe_json(asdict(value), depth + 1)
    if isinstance(value, dict):
        return {str(key): safe_json(item, depth + 1) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_json(item, depth + 1) for item in value]
    return {"type": type(value).__name__, "repr": repr(value)[:500]}


def binary_size(value: Any) -> int:
    if isinstance(value, (bytes, bytearray, memoryview)):
        return len(value)
    if isinstance(value, dict):
        return sum(binary_size(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return sum(binary_size(item) for item in value)
    return 0


def csv_values(value: str | None) -> list[str]:
    return [item.strip() for item in str(value or "").split(",") if item.strip()]


def byte_sizes(value: str | None) -> list[int]:
    sizes: list[int] = []
    for item in csv_values(value):
        size = int(item)
        if size < 0:
            raise ValueError("transport payload sizes must be non-negative")
        sizes.append(size)
    if not sizes:
        raise ValueError("transport payload sizes must contain at least one integer")
    return list(dict.fromkeys(sizes))


def bootstrap_repo(repo_value: str | None) -> Path | None:
    if repo_value:
        root = Path(repo_value).expanduser().resolve()
        if not (root / "rynnrcp").is_dir():
            raise RuntimeError(f"RynnRCP repository root is invalid: {root}")
        sys.path.insert(0, str(root))
        return root
    for parent in Path(__file__).resolve().parents:
        if (parent / "rynnrcp").is_dir() and (parent / "pyproject.toml").is_file():
            sys.path.insert(0, str(parent))
            return parent
    return None


def resolve_output(args: argparse.Namespace) -> Path:
    if args.output:
        return Path(args.output).expanduser().resolve()
    if args.config:
        source_name = Path(args.config).expanduser().stem
    else:
        source_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(args.endpoint)).strip("_")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return (Path.cwd() / "benchmark_results" / f"{source_name}_{timestamp}").resolve()


def command_output(command: list[str], cwd: Path) -> str:
    try:
        result = subprocess.run(
            command,
            cwd=str(cwd),
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
        return result.stdout.strip() or result.stderr.strip()
    except Exception as exc:
        return f"unavailable: {exc}"


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def environment_record(repo: Path | None, config: Path | None) -> dict[str, Any]:
    import psutil

    packages: dict[str, str] = {}
    for name in ("rynnrcp", "grpcio", "msgpack", "PyYAML", "numpy", "psutil"):
        try:
            packages[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            packages[name] = "not installed"
    memory = psutil.virtual_memory()
    result: dict[str, Any] = {
        "collected_at": utc_now(),
        "platform": {
            "system": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
            "processor": platform.processor(),
            "logical_cpu_count": psutil.cpu_count(logical=True),
            "physical_cpu_count": psutil.cpu_count(logical=False),
            "memory_total_bytes": memory.total,
        },
        "python": {"version": sys.version, "executable": sys.executable},
        "packages": packages,
    }
    if repo is not None:
        result["repository"] = {
            "path": str(repo),
            "revision": command_output(["git", "rev-parse", "HEAD"], repo),
            "status": command_output(["git", "status", "--short"], repo),
        }
    if config is not None:
        result["server_config"] = {"path": str(config), "sha256": file_sha256(config)}
    return result


class Phase:
    def __init__(self) -> None:
        self._value = "startup"
        self._lock = threading.Lock()

    def set(self, value: str) -> None:
        with self._lock:
            self._value = str(value)

    def get(self) -> str:
        with self._lock:
            return self._value


class ResourceMonitor:
    def __init__(
        self,
        output: Path,
        phase: Phase,
        server_pid: int | None,
        interval_s: float,
    ) -> None:
        self._output = output
        self._phase = phase
        self._server_pid = server_pid
        self._interval_s = max(0.2, float(interval_s))
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.rows: list[dict[str, Any]] = []

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="resource_monitor", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    def _run(self) -> None:
        import psutil

        recorder = JsonlRecorder(self._output)
        client_process = psutil.Process(os.getpid())
        known_processes: dict[int, Any] = {}
        client_process.cpu_percent(None)
        psutil.cpu_percent(None)
        started = time.monotonic()
        next_sample = started + self._interval_s
        try:
            while not self._stop.is_set():
                if self._stop.wait(max(0.0, next_sample - time.monotonic())):
                    break
                try:
                    server_processes: list[Any] = []
                    if self._server_pid is not None:
                        try:
                            root = psutil.Process(self._server_pid)
                            server_processes = [root, *root.children(recursive=True)]
                        except (psutil.NoSuchProcess, psutil.AccessDenied):
                            server_processes = []
                    server_rows: list[dict[str, Any]] = []
                    for process in server_processes:
                        try:
                            tracked = known_processes.get(process.pid)
                            if tracked is None:
                                tracked = process
                                known_processes[process.pid] = tracked
                                tracked.cpu_percent(None)
                            server_rows.append(
                                {
                                    "pid": tracked.pid,
                                    "name": tracked.name(),
                                    "cpu_percent": tracked.cpu_percent(None),
                                    "rss_kib": tracked.memory_info().rss // 1024,
                                }
                            )
                        except (psutil.NoSuchProcess, psutil.AccessDenied):
                            continue
                    memory = psutil.virtual_memory()
                    record = {
                        "collected_at": utc_now(),
                        "elapsed_s": time.monotonic() - started,
                        "phase": self._phase.get(),
                        "ok": True,
                        "system_cpu_percent": psutil.cpu_percent(None),
                        "system_memory_percent": memory.percent,
                        "system_memory_available_kib": memory.available // 1024,
                        "client_cpu_percent": client_process.cpu_percent(None),
                        "client_rss_kib": client_process.memory_info().rss // 1024,
                        "server_process_tree_cpu_percent": sum(
                            float(item["cpu_percent"]) for item in server_rows
                        ),
                        "server_process_tree_rss_kib": sum(int(item["rss_kib"]) for item in server_rows),
                        "server_processes": server_rows,
                    }
                except Exception as exc:
                    record = {
                        "collected_at": utc_now(),
                        "elapsed_s": time.monotonic() - started,
                        "phase": self._phase.get(),
                        "ok": False,
                        "error": str(exc),
                    }
                self.rows.append(record)
                recorder.write(record)
                next_sample += self._interval_s
        finally:
            recorder.close()


class ManagedServer:
    ENDPOINT_PATTERN = re.compile(r"Local gRPC:\s*([^\s]+)")

    def __init__(self, config: Path, repo: Path | None, output: Path) -> None:
        self.config = config
        self.repo = repo
        self.output = output
        self.process: subprocess.Popen[Any] | None = None
        self._log_stream: Any = None

    @property
    def pid(self) -> int | None:
        return self.process.pid if self.process is not None else None

    def start(self, timeout_s: float) -> str:
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self._log_stream = self.output.open("w", encoding="utf-8")
        environment = os.environ.copy()
        environment["PYTHONUNBUFFERED"] = "1"
        if self.repo is not None:
            current = environment.get("PYTHONPATH", "")
            environment["PYTHONPATH"] = str(self.repo) + (os.pathsep + current if current else "")
        command = [sys.executable, "-u", "-m", "rynnrcp.cli_server", "--config", str(self.config)]
        self.process = subprocess.Popen(
            command,
            stdout=self._log_stream,
            stderr=subprocess.STDOUT,
            text=True,
            env=environment,
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError(
                    f"RynnRCP Server exited during startup with code {self.process.returncode}:\n{self.log_tail()}"
                )
            self._log_stream.flush()
            content = self.output.read_text(encoding="utf-8", errors="replace")
            match = self.ENDPOINT_PATTERN.search(content)
            if match:
                return match.group(1)
            time.sleep(0.1)
        raise RuntimeError(f"RynnRCP Server startup timed out:\n{self.log_tail()}")

    def log_tail(self, lines: int = 40) -> str:
        if not self.output.is_file():
            return "server log is unavailable"
        return "\n".join(self.output.read_text(encoding="utf-8", errors="replace").splitlines()[-lines:])

    def stop(self) -> None:
        process = self.process
        if process is None:
            return
        if process.poll() is None:
            try:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=20.0)
            except (subprocess.TimeoutExpired, OSError):
                process.terminate()
                try:
                    process.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5.0)
        if self._log_stream is not None:
            self._log_stream.close()


def response_observation(response: Any, name: str) -> dict[str, Any] | None:
    payload = response.payload if response is not None and isinstance(response.payload, dict) else {}
    for item in payload.get("observations") or []:
        if isinstance(item, dict) and str(item.get("name")) == name:
            return item
    return None


def response_size(response: Any) -> int:
    from rynnrcp.interface.codec import pack_response

    return len(pack_response(response))


def wire_request_size(method: str, payload: Any, timeout_ms: int | None) -> int:
    from rynnrcp.interface.codec import InterfaceRequest, pack_request

    return len(
        pack_request(
            InterfaceRequest(
                method=method,
                payload=payload,
                timeout_ms=timeout_ms,
            )
        )
    )


def wire_response_size(response: Any) -> int:
    """Reconstruct the response size before client-local timing metadata is attached."""
    from rynnrcp.interface.codec import pack_response

    metadata_value = response.metadata if response is not None else {}
    metadata = dict(metadata_value or {})
    metadata.pop("client_timing_ms", None)
    return len(pack_response(replace(response, metadata=metadata)))


def select_observations(manifest: Any, args: argparse.Namespace) -> list[dict[str, Any]]:
    descriptors = [dict(item) for item in manifest.observations if isinstance(item, dict)]
    by_name = {str(item.get("name")): item for item in descriptors if item.get("name")}
    requested: list[str] = []
    requested.extend(csv_values(args.observations))
    for component in csv_values(args.arm):
        prefix = f"observation.{component}."
        requested.extend(
            name
            for name, descriptor in by_name.items()
            if name.startswith(prefix) and str(descriptor.get("type")) != "image"
        )
    for component in csv_values(args.camera):
        prefix = f"observation.{component}."
        requested.extend(
            name
            for name, descriptor in by_name.items()
            if name.startswith(prefix) and str(descriptor.get("type")) == "image"
        )
    if not requested:
        requested = list(by_name)
    unique = list(dict.fromkeys(requested))
    unknown = [name for name in unique if name not in by_name]
    if unknown:
        available = ", ".join(sorted(by_name))
        raise RuntimeError(f"unknown observations: {', '.join(unknown)}; available: {available}")
    if not unique:
        raise RuntimeError("the selected RynnRCP Server exposes no observations")
    return [by_name[name] for name in unique]


def preflight_observations(
    client: Any,
    descriptors: list[dict[str, Any]],
    timeout_ms: int,
    readiness_timeout_s: float,
) -> dict[str, Any]:
    results: dict[str, Any] = {}
    for descriptor in descriptors:
        name = str(descriptor["name"])
        started = time.monotonic()
        deadline = started + readiness_timeout_s
        attempts = 0
        last_error = "no data"
        while time.monotonic() < deadline:
            attempts += 1
            try:
                response = client.get_observations([name], timeout_ms=timeout_ms)
                item = response_observation(response, name)
                if response.ok and item is not None:
                    results[name] = {
                        "ready": True,
                        "attempts": attempts,
                        "ready_after_s": time.monotonic() - started,
                        "source_timestamp": item.get("timestamp"),
                        "binary_payload_bytes": binary_size(item.get("value")),
                    }
                    break
                last_error = response.message or "no observation data"
            except Exception as exc:
                last_error = str(exc)
            time.sleep(0.2)
        else:
            results[name] = {
                "ready": False,
                "attempts": attempts,
                "waited_s": time.monotonic() - started,
                "error": last_error,
            }
            raise RuntimeError(
                f"preflight failed for {name} after {readiness_timeout_s:.1f}s: {last_error}"
            )
    return results


def target_hz(descriptor: dict[str, Any], args: argparse.Namespace) -> float:
    requested = args.camera_hz if str(descriptor.get("type")) == "image" else args.arm_hz
    try:
        advertised = float(descriptor.get("frame_rate") or 0.0)
    except (TypeError, ValueError):
        advertised = 0.0
    if advertised > 0:
        return min(float(requested), advertised)
    return float(requested)


def timing_metadata(response: Any) -> dict[str, Any]:
    metadata_value = response.metadata if response is not None else {}
    return {
        "client_timing_ms": dict(metadata_value.get("client_timing_ms") or {}),
        "server_timing_ms": dict(metadata_value.get("server_timing_ms") or {}),
        "dispatcher_timing_ms": dict(metadata_value.get("dispatcher_timing_ms") or {}),
    }


def measure_transport_baseline(
    client: Any,
    args: argparse.Namespace,
    recorder: JsonlRecorder,
    phase_name: str,
) -> dict[str, Any]:
    payload_results: dict[str, Any] = {}
    for size in byte_sizes(args.transport_sizes):
        payload = os.urandom(size)
        request_payload = {"data": payload}
        request_bytes = wire_request_size("ping", request_payload, args.timeout_ms)
        latencies: list[float] = []
        request_pack: list[float] = []
        round_trips: list[float] = []
        response_unpack: list[float] = []
        server_request_unpack: list[float] = []
        server_handlers: list[float] = []
        response_sizes: list[float] = []
        failures = 0
        for index in range(args.transport_warmup + args.transport_samples):
            sample_phase = "warmup" if index < args.transport_warmup else "sample"
            started = time.perf_counter()
            try:
                response = client.ping(request_payload, timeout_ms=args.timeout_ms)
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                response_payload = response.payload if isinstance(response.payload, dict) else {}
                echoed_payload = response_payload.get("request_payload")
                echoed_data = echoed_payload.get("data") if isinstance(echoed_payload, dict) else None
                metadata = timing_metadata(response)
                client_timing = metadata["client_timing_ms"]
                server_timing = metadata["server_timing_ms"]
                ok = bool(response.ok and isinstance(echoed_data, bytes) and len(echoed_data) == size)
                record = {
                    "record_type": "transport_baseline",
                    "benchmark_phase": phase_name,
                    "phase": sample_phase,
                    "iteration": index,
                    "ok": ok,
                    "request_payload_bytes": size,
                    "wire_request_bytes": request_bytes,
                    "wire_response_bytes": wire_response_size(response),
                    "elapsed_ms": elapsed_ms,
                    **metadata,
                }
                if sample_phase == "sample" and ok:
                    latencies.append(elapsed_ms)
                    request_pack.append(float(client_timing.get("pack_request") or 0.0))
                    round_trips.append(float(client_timing.get("grpc_call") or 0.0))
                    response_unpack.append(float(client_timing.get("unpack_response") or 0.0))
                    server_request_unpack.append(float(server_timing.get("unpack_request") or 0.0))
                    server_handlers.append(float(server_timing.get("handler") or 0.0))
                    response_sizes.append(float(record["wire_response_bytes"]))
                elif sample_phase == "sample":
                    failures += 1
            except Exception as exc:
                record = {
                    "record_type": "transport_baseline",
                    "benchmark_phase": phase_name,
                    "phase": sample_phase,
                    "iteration": index,
                    "ok": False,
                    "request_payload_bytes": size,
                    "elapsed_ms": (time.perf_counter() - started) * 1000.0,
                    "error": str(exc),
                }
                if sample_phase == "sample":
                    failures += 1
            recorder.write(record)
        payload_results[str(size)] = {
            "request_payload_bytes": size,
            "wire_request_bytes": request_bytes,
            "samples": args.transport_samples,
            "client_call_latency_ms": summarize(latencies),
            "request_pack_ms": summarize(request_pack),
            "transport_round_trip_ms": summarize(round_trips),
            "response_unpack_ms": summarize(response_unpack),
            "server_request_unpack_ms": summarize(server_request_unpack),
            "server_handler_ms": summarize(server_handlers),
            "wire_response_bytes": summarize(response_sizes),
            "failures": failures,
            "failure_rate": failures / args.transport_samples,
        }
    return {
        "method": "ping",
        "pattern": "echo",
        "measurement_note": (
            "fixed-payload Interface baseline using one persistent client connection "
            "to the tested Server"
        ),
        "payloads": payload_results,
    }


def measure_observation_latency(
    client: Any,
    descriptor: dict[str, Any],
    args: argparse.Namespace,
    recorder: JsonlRecorder,
    phase_name: str,
) -> dict[str, Any]:
    name = str(descriptor["name"])
    latencies: list[float] = []
    data_ages: list[float] = []
    response_sizes: list[float] = []
    payload_sizes: list[float] = []
    failures = 0
    for index in range(args.warmup + args.samples):
        phase = "warmup" if index < args.warmup else "sample"
        started = time.perf_counter()
        try:
            response = client.get_observations([name], timeout_ms=args.timeout_ms)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            item = response_observation(response, name)
            received_at = time.time()
            source_timestamp = item.get("timestamp") if item else None
            data_age = (
                max(0.0, (received_at - float(source_timestamp)) * 1000.0)
                if isinstance(source_timestamp, (int, float))
                else None
            )
            record = {
                "record_type": "observation_latency",
                "benchmark_phase": phase_name,
                "phase": phase,
                "observation": name,
                "iteration": index,
                "ok": bool(response.ok and item is not None),
                "elapsed_ms": elapsed_ms,
                "data_age_ms": data_age,
                "source_timestamp": source_timestamp,
                "response_size_bytes": response_size(response),
                "binary_payload_bytes": binary_size(item.get("value")) if item else 0,
                **timing_metadata(response),
            }
            if phase == "sample" and record["ok"]:
                latencies.append(elapsed_ms)
                if data_age is not None:
                    data_ages.append(data_age)
                response_sizes.append(float(record["response_size_bytes"]))
                payload_sizes.append(float(record["binary_payload_bytes"]))
            elif phase == "sample":
                failures += 1
        except Exception as exc:
            record = {
                "record_type": "observation_latency",
                "benchmark_phase": phase_name,
                "phase": phase,
                "observation": name,
                "iteration": index,
                "ok": False,
                "elapsed_ms": (time.perf_counter() - started) * 1000.0,
                "error": str(exc),
            }
            if phase == "sample":
                failures += 1
        recorder.write(record)
    return {
        "descriptor": safe_json(descriptor),
        "latency_ms": summarize(latencies),
        "data_age_ms": summarize(data_ages),
        "response_size_bytes": summarize(response_sizes),
        "binary_payload_bytes": summarize(payload_sizes),
        "failures": failures,
        "failure_rate": failures / args.samples,
    }


def periodic_observation_worker(
    endpoint: str,
    descriptor: dict[str, Any],
    hz: float,
    duration_s: float,
    timeout_ms: int,
    recorder: JsonlRecorder,
    phase_name: str,
    result: dict[str, Any],
) -> None:
    from rynnrcp.interface.protocol_client import RcpProtocolClient

    name = str(descriptor["name"])
    client = RcpProtocolClient.connect(endpoint)
    period = 1.0 / hz
    wall_started = time.monotonic()
    deadline = wall_started + duration_s
    next_tick = wall_started
    latencies: list[float] = []
    data_ages: list[float] = []
    response_sizes: list[float] = []
    binary_sizes: list[float] = []
    source_timestamps: list[float] = []
    failures = 0
    deadline_misses = 0
    calls = 0
    target_calls = max(1, int(round(duration_s * hz)))
    try:
        for _ in range(target_calls):
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            started = time.perf_counter()
            calls += 1
            try:
                response = client.get_observations([name], timeout_ms=timeout_ms)
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                item = response_observation(response, name)
                received_at = time.time()
                source_timestamp = item.get("timestamp") if item else None
                data_age = (
                    max(0.0, (received_at - float(source_timestamp)) * 1000.0)
                    if isinstance(source_timestamp, (int, float))
                    else None
                )
                ok = bool(response.ok and item is not None)
                record = {
                    "record_type": "observation_load",
                    "benchmark_phase": phase_name,
                    "observation": name,
                    "iteration": calls - 1,
                    "ok": ok,
                    "elapsed_ms": elapsed_ms,
                    "data_age_ms": data_age,
                    "source_timestamp": source_timestamp,
                    "response_size_bytes": response_size(response),
                    "binary_payload_bytes": binary_size(item.get("value")) if item else 0,
                    **timing_metadata(response),
                }
                if ok:
                    latencies.append(elapsed_ms)
                    if data_age is not None:
                        data_ages.append(data_age)
                    if isinstance(source_timestamp, (int, float)):
                        source_timestamps.append(float(source_timestamp))
                    response_sizes.append(float(record["response_size_bytes"]))
                    binary_sizes.append(float(record["binary_payload_bytes"]))
                else:
                    failures += 1
            except Exception as exc:
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                failures += 1
                record = {
                    "record_type": "observation_load",
                    "benchmark_phase": phase_name,
                    "observation": name,
                    "iteration": calls - 1,
                    "ok": False,
                    "elapsed_ms": elapsed_ms,
                    "error": str(exc),
                }
            if elapsed_ms > period * 1000.0:
                deadline_misses += 1
            recorder.write(record)
            next_tick += period
        remaining = deadline - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        elapsed_s = max(1e-9, time.monotonic() - wall_started)
        unique_timestamps = list(dict.fromkeys(source_timestamps))
        source_hz = 0.0
        if len(unique_timestamps) > 1 and unique_timestamps[-1] > unique_timestamps[0]:
            source_hz = (len(unique_timestamps) - 1) / (
                unique_timestamps[-1] - unique_timestamps[0]
            )
        repeats = max(0, len(source_timestamps) - len(unique_timestamps))
        result[name] = {
            "descriptor": safe_json(descriptor),
            "target_hz": hz,
            "calls": calls,
            "successes": len(latencies),
            "failures": failures,
            "failure_rate": failures / calls if calls else 0.0,
            "achieved_request_hz": len(latencies) / elapsed_s,
            "fresh_source_samples": len(unique_timestamps),
            "estimated_source_hz": source_hz,
            "repeated_timestamp_count": repeats,
            "repeated_timestamp_rate": repeats / len(source_timestamps) if source_timestamps else 0.0,
            "deadline_misses": deadline_misses,
            "deadline_miss_rate": deadline_misses / calls if calls else 0.0,
            "latency_ms": summarize(latencies),
            "data_age_ms": summarize(data_ages),
            "response_size_bytes": summarize(response_sizes),
            "binary_payload_bytes": summarize(binary_sizes),
        }
    finally:
        client.close()


def measure_mixed_load(
    endpoint: str,
    descriptors: list[dict[str, Any]],
    args: argparse.Namespace,
    recorder: JsonlRecorder,
    phase_name: str,
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    threads = [
        threading.Thread(
            target=periodic_observation_worker,
            args=(
                endpoint,
                descriptor,
                target_hz(descriptor, args),
                args.duration,
                args.timeout_ms,
                recorder,
                phase_name,
                result,
            ),
            name=f"live_{str(descriptor['name']).replace('.', '_')}",
        )
        for descriptor in descriptors
    ]
    started = time.monotonic()
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    return {"duration_s": time.monotonic() - started, "observations": result}


def read_joint_positions(client: Any, observation_name: str, timeout_ms: int) -> list[float]:
    response = client.get_observations([observation_name], timeout_ms=timeout_ms)
    item = response_observation(response, observation_name)
    value = item.get("value") if item else None
    positions = value.get("joint_positions") if isinstance(value, dict) else None
    if not response.ok or not isinstance(positions, list) or not positions:
        raise RuntimeError(
            f"{observation_name} does not provide a non-empty joint_positions array"
        )
    return [float(item) for item in positions]


def measure_action_submission(
    client: Any,
    manifest: Any,
    args: argparse.Namespace,
    recorder: JsonlRecorder,
    phase_name: str,
) -> dict[str, Any] | None:
    if not args.action:
        return None
    actions = {
        str(item.get("name")): dict(item)
        for item in manifest.actions
        if isinstance(item, dict) and item.get("name")
    }
    descriptor = actions.get(args.action)
    if descriptor is None:
        raise RuntimeError(
            f"unknown action {args.action!r}; available: {', '.join(sorted(actions))}"
        )
    if str(descriptor.get("type")) != "joint_position":
        raise RuntimeError("live action benchmark supports only joint_position actions")
    positions = read_joint_positions(client, args.action_state, args.timeout_ms)
    frame = {"joint_positions": positions}
    period = 1.0 / args.action_hz
    started = time.monotonic()
    deadline = started + args.action_duration
    next_tick = started
    latencies: list[float] = []
    failures = 0
    deadline_misses = 0
    calls = 0
    target_calls = max(1, int(round(args.action_duration * args.action_hz)))
    for _ in range(target_calls):
        sleep_time = next_tick - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)
        call_started = time.perf_counter()
        calls += 1
        try:
            response = client.run_action_chunk(
                args.action,
                [frame],
                frame_rate=args.action_hz,
                timeout_ms=args.timeout_ms,
            )
            elapsed_ms = (time.perf_counter() - call_started) * 1000.0
            ok = bool(response.ok)
            if ok:
                latencies.append(elapsed_ms)
            else:
                failures += 1
            record = {
                "record_type": "action_submission",
                "benchmark_phase": phase_name,
                "action": args.action,
                "iteration": calls - 1,
                "ok": ok,
                "elapsed_ms": elapsed_ms,
                **timing_metadata(response),
            }
        except Exception as exc:
            elapsed_ms = (time.perf_counter() - call_started) * 1000.0
            failures += 1
            record = {
                "record_type": "action_submission",
                "benchmark_phase": phase_name,
                "action": args.action,
                "iteration": calls - 1,
                "ok": False,
                "elapsed_ms": elapsed_ms,
                "error": str(exc),
            }
        if elapsed_ms > period * 1000.0:
            deadline_misses += 1
        recorder.write(record)
        next_tick += period
    remaining = deadline - time.monotonic()
    if remaining > 0:
        time.sleep(remaining)
    elapsed_s = max(1e-9, time.monotonic() - started)
    return {
        "descriptor": safe_json(descriptor),
        "state_observation": args.action_state,
        "strategy": "send the measured current joint positions as a hold-position command",
        "target_hz": args.action_hz,
        "duration_s": elapsed_s,
        "calls": calls,
        "successes": len(latencies),
        "failures": failures,
        "failure_rate": failures / calls if calls else 0.0,
        "achieved_submission_hz": len(latencies) / elapsed_s,
        "deadline_misses": deadline_misses,
        "deadline_miss_rate": deadline_misses / calls if calls else 0.0,
        "submission_latency_ms": summarize(latencies),
        "measurement_boundary": (
            "the response confirms that RynnRCP accepted and published the frame; "
            "it does not confirm physical motor completion"
        ),
    }


def summarize_resources(rows: list[dict[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    phases = sorted({str(row.get("phase")) for row in rows if row.get("ok")})
    fields = (
        "system_cpu_percent",
        "system_memory_percent",
        "client_cpu_percent",
        "client_rss_kib",
        "server_process_tree_cpu_percent",
        "server_process_tree_rss_kib",
    )
    for phase in phases:
        phase_rows = [row for row in rows if row.get("ok") and str(row.get("phase")) == phase]
        result[phase] = {
            field: summarize([float(row[field]) for row in phase_rows if row.get(field) is not None])
            for field in fields
        }
    return result


def resource_overview(rows: list[dict[str, Any]]) -> dict[str, Any]:
    fields = (
        "system_cpu_percent",
        "system_memory_percent",
        "client_cpu_percent",
        "client_rss_kib",
        "server_process_tree_cpu_percent",
        "server_process_tree_rss_kib",
    )
    valid = [row for row in rows if row.get("ok")]
    groups = {
        "idle": [row for row in valid if str(row.get("phase")) == "idle"],
        "active": [row for row in valid if str(row.get("phase", "")).startswith("run_")],
    }
    result: dict[str, Any] = {}
    for group, group_rows in groups.items():
        result[group] = {
            field: summarize([float(row[field]) for row in group_rows if row.get(field) is not None])
            for field in fields
        }
    result["active_minus_idle_avg"] = {}
    for field in fields:
        idle_average = result["idle"][field].get("avg")
        active_average = result["active"][field].get("avg")
        result["active_minus_idle_avg"][field] = (
            float(active_average) - float(idle_average)
            if idle_average is not None and active_average is not None
            else None
        )
    return result


def wait_for_manifest(endpoint: str, timeout_ms: int, timeout_s: float) -> tuple[Any, Any]:
    from rynnrcp.interface.protocol_client import RcpProtocolClient

    deadline = time.monotonic() + timeout_s
    last_error = ""
    while time.monotonic() < deadline:
        client = None
        try:
            client = RcpProtocolClient.connect(endpoint)
            manifest = client.get_manifest(timeout_ms=timeout_ms)
            return client, manifest
        except Exception as exc:
            last_error = str(exc)
            if client is not None:
                client.close()
            time.sleep(0.2)
    raise RuntimeError(f"failed to read RynnRCP manifest from {endpoint}: {last_error}")


def print_available(manifest: Any) -> None:
    print(f"robot_id: {manifest.robot_id}")
    print("observations:")
    for item in manifest.observations:
        print(
            f"  - {item.get('name')}  type={item.get('type')}  frame_rate={item.get('frame_rate')}"
        )
    print("actions:")
    for item in manifest.actions:
        print(f"  - {item.get('name')}  type={item.get('type')}  frame_rate={item.get('frame_rate')}")


def run(args: argparse.Namespace) -> int:
    repo = bootstrap_repo(args.repo)
    output = resolve_output(args)
    output.mkdir(parents=True, exist_ok=True)
    config = Path(args.config).expanduser().resolve() if args.config else None
    if config is not None and not config.is_file():
        raise RuntimeError(f"server config does not exist: {config}")

    managed_server: ManagedServer | None = None
    client = None
    recorder: JsonlRecorder | None = None
    monitor: ResourceMonitor | None = None
    phase = Phase()
    try:
        if config is not None:
            managed_server = ManagedServer(config, repo, output / "server.log")
            endpoint = managed_server.start(args.startup_timeout)
            server_pid = managed_server.pid
        else:
            endpoint = str(args.endpoint)
            server_pid = args.server_pid

        client, manifest = wait_for_manifest(endpoint, args.timeout_ms, args.startup_timeout)
        (output / "environment.json").write_text(
            json.dumps(environment_record(repo, config), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        (output / "manifest.json").write_text(
            json.dumps(safe_json(manifest), ensure_ascii=False, indent=2), encoding="utf-8"
        )
        parameters = dict(vars(args))
        parameters["resolved_endpoint"] = endpoint
        parameters["resolved_server_pid"] = server_pid
        parameters["resolved_output"] = str(output)
        parameters["started_at"] = utc_now()
        (output / "parameters.json").write_text(
            json.dumps(safe_json(parameters), ensure_ascii=False, indent=2), encoding="utf-8"
        )

        if args.list_devices:
            print_available(manifest)
            return 0

        selected = [] if args.transport_only else select_observations(manifest, args)
        if args.action and not args.enable_action_test:
            raise RuntimeError("pass --enable-action-test to authorize real joint action submission")
        if args.enable_action_test and not args.action:
            raise RuntimeError("--enable-action-test requires --action")

        recorder = JsonlRecorder(output / "raw_samples.jsonl")
        preflight = preflight_observations(
            client,
            selected,
            timeout_ms=args.timeout_ms,
            readiness_timeout_s=args.preflight_timeout,
        )
        (output / "preflight.json").write_text(
            json.dumps(preflight, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        monitor = ResourceMonitor(
            output / "resources.jsonl",
            phase,
            server_pid=server_pid,
            interval_s=args.resource_interval,
        )
        monitor.start()
        phase.set("idle")
        time.sleep(args.idle_duration)

        transport_baseline = None
        if not args.skip_transport_baseline:
            phase.set("transport_baseline")
            transport_baseline = measure_transport_baseline(
                client,
                args,
                recorder,
                "transport_baseline",
            )

        runs: list[dict[str, Any]] = []
        repeat_count = 0 if args.transport_only else args.repeat
        for run_index in range(1, repeat_count + 1):
            run_name = f"run_{run_index:02d}"
            latency_results: dict[str, Any] = {}
            for descriptor in selected:
                observation_name = str(descriptor["name"])
                phase_name = f"{run_name}:latency:{observation_name}"
                phase.set(phase_name)
                latency_results[observation_name] = measure_observation_latency(
                    client, descriptor, args, recorder, phase_name
                )

            mixed_phase = f"{run_name}:mixed_load"
            phase.set(mixed_phase)
            mixed = measure_mixed_load(endpoint, selected, args, recorder, mixed_phase)

            action_result = None
            if args.action:
                action_phase = f"{run_name}:action"
                phase.set(action_phase)
                action_result = measure_action_submission(
                    client, manifest, args, recorder, action_phase
                )
            runs.append(
                {
                    "run": run_index,
                    "observation_latency": latency_results,
                    "mixed_load": mixed,
                    "action_submission": action_result,
                }
            )

        phase.set("complete")
        time.sleep(min(1.0, args.resource_interval))
        monitor.stop()
        summary = {
            "metadata": {
                "started_at": parameters["started_at"],
                "finished_at": utc_now(),
                "label": args.label or f"{platform.machine()}_{manifest.robot_id}",
                "endpoint": endpoint,
                "robot_id": manifest.robot_id,
                "robot_name": manifest.robot_name,
                "selected_observations": [str(item["name"]) for item in selected],
                "selection_mode": (
                    "transport baseline only"
                    if args.transport_only
                    else "all manifest observations"
                    if not any((args.arm, args.camera, args.observations))
                    else "advanced filter"
                ),
                "server_started_by_script": managed_server is not None,
                "server_pid": server_pid,
                "clock_note": (
                    "data age is comparable across hosts only when Client and Server clocks are synchronized"
                ),
                "resource_note": (
                    "server process-tree CPU can exceed 100 percent when multiple CPU cores are used"
                ),
            },
            "preflight": preflight,
            "transport_baseline": transport_baseline,
            "runs": runs,
            "resource_overview": resource_overview(monitor.rows),
            "resources_by_phase": summarize_resources(monitor.rows),
        }
        (output / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print(output)
        return 0
    except Exception as exc:
        (output / "failure.json").write_text(
            json.dumps({"failed_at": utc_now(), "error": str(exc)}, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        raise
    finally:
        if monitor is not None:
            monitor.stop()
        if recorder is not None:
            recorder.close()
        if client is not None:
            client.close()
        if managed_server is not None:
            managed_server.stop()


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    source = result.add_mutually_exclusive_group(required=True)
    source.add_argument("--config", help="real-device RynnRCP Server YAML; the script starts and stops it")
    source.add_argument("--endpoint", help="address of an already running RynnRCP Server")
    result.add_argument("--repo", help="RynnRCP repository root; normally auto-detected from this script")
    result.add_argument("--server-pid", type=int, help="root Server PID for resource sampling with --endpoint")
    result.add_argument(
        "--output",
        help="result directory; defaults to benchmark_results/<config>_<timestamp>",
    )
    result.add_argument("--label", default="")
    result.add_argument("--arm", default="", help="advanced filter: comma-separated arm component names")
    result.add_argument("--camera", default="", help="advanced filter: comma-separated camera component names")
    result.add_argument("--observations", default="", help="advanced filter: full Observation names")
    result.add_argument("--list-devices", action="store_true", help="print Manifest observations/actions and exit")
    result.add_argument("--samples", type=int, default=300)
    result.add_argument("--warmup", type=int, default=30)
    result.add_argument("--duration", type=float, default=300.0)
    result.add_argument("--idle-duration", type=float, default=30.0)
    result.add_argument("--repeat", type=int, default=3)
    result.add_argument("--arm-hz", type=float, default=60.0)
    result.add_argument("--camera-hz", type=float, default=30.0)
    result.add_argument("--timeout-ms", type=int, default=5000)
    result.add_argument("--startup-timeout", type=float, default=90.0)
    result.add_argument("--preflight-timeout", type=float, default=30.0)
    result.add_argument("--resource-interval", type=float, default=1.0)
    result.add_argument(
        "--transport-sizes",
        default="0,493,32768,65536",
        help="comma-separated fixed payload sizes for the Interface transport baseline",
    )
    result.add_argument("--transport-samples", type=int, default=300)
    result.add_argument("--transport-warmup", type=int, default=30)
    result.add_argument(
        "--skip-transport-baseline",
        action="store_true",
        help="run only real-device Observation and resource tests",
    )
    result.add_argument(
        "--transport-only",
        action="store_true",
        help="run the fixed-payload Interface baseline without Observation load tests",
    )
    result.add_argument("--action", help="full joint_position Action name")
    result.add_argument("--action-state", default="observation.robot.joint_state")
    result.add_argument("--action-hz", type=float, default=30.0)
    result.add_argument("--action-duration", type=float, default=10.0)
    result.add_argument("--enable-action-test", action="store_true")
    return result


def validate_args(args: argparse.Namespace) -> None:
    for name in ("samples", "repeat", "transport_samples"):
        if int(getattr(args, name)) < 1:
            raise ValueError(f"{name} must be at least 1")
    for name in ("warmup", "transport_warmup"):
        if int(getattr(args, name)) < 0:
            raise ValueError(f"{name} must be non-negative")
    byte_sizes(args.transport_sizes)
    if args.transport_only and args.skip_transport_baseline:
        raise ValueError("--transport-only cannot be combined with --skip-transport-baseline")
    if args.transport_only and (args.action or args.enable_action_test):
        raise ValueError("--transport-only cannot be combined with action testing")
    for name in (
        "duration",
        "arm_hz",
        "camera_hz",
        "startup_timeout",
        "preflight_timeout",
        "resource_interval",
        "action_hz",
        "action_duration",
    ):
        if float(getattr(args, name)) <= 0:
            raise ValueError(f"{name} must be greater than zero")
    if args.idle_duration < 0:
        raise ValueError("idle_duration must be non-negative")
    if args.server_pid is not None and args.server_pid <= 0:
        raise ValueError("server_pid must be a positive process ID")


def main() -> int:
    args = parser().parse_args()
    validate_args(args)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
