"""CollectionService: expose RCP Data Collection tools."""

from __future__ import annotations

import json
import logging
import os
import shutil
import threading
import time
from typing import Any, Callable, Dict, Iterable, Optional

import msgpack

from .base_service import BaseService
from .observation_service import _observation_type, _protocol_observation_name
from .resource_service import ResourceRegistry
from rynnrcp.protocol.action_codecs import protocol_action_value
from rynnrcp.protocol.methods import (
    DELETE_COLLECTION,
    GET_COLLECTION_STATUS,
    START_COLLECTION,
    STOP_COLLECTION,
)
from rynnrcp.protocol.observation_codecs import protocol_observation_value
from rynnrcp.config.runner_config import RunnerInputSpec
from rynnrcp.runtime.tool_bus import ToolBus
from rynnrcp.utils.log_gate import LogGate
from rynnrcp.utils.payload import CachedSharedRefParser
from rynnrcp.utils.redaction import describe_payload
from rynnrcp.utils import coerce_timestamp
from rynnrcp.utils import safe_name
from rynnrcp.utils.shared_data_store import SharedDataExpired


def _pack_safe(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool, bytes, bytearray)):
        return value
    if isinstance(value, dict):
        return {str(k): _pack_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_pack_safe(v) for v in value]
    raise TypeError(
        f"collection value is not msgpack compatible: {type(value).__name__}"
    )


def _write_json(path: str, obj: Dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)


def _sample_value_diagnostics(value: Any) -> str:
    positions = value.get("joint_positions") if isinstance(value, dict) else None
    if not isinstance(positions, list):
        return describe_payload(value)
    try:
        values = [float(item) for item in positions]
    except (TypeError, ValueError):
        return f"dim={len(positions)} values={describe_payload(positions)}"
    nonzero = sum(item != 0.0 for item in values)
    minimum = min(values) if values else 0.0
    maximum = max(values) if values else 0.0
    return f"dim={len(values)} nonzero={nonzero} min={minimum:.4f} max={maximum:.4f}"


SampleFn = Callable[[list[str], Optional[Dict[str, float]]], Dict[str, Dict[str, Any]]]
logger = logging.getLogger(__name__)


class _CollectionSession:
    """One background collection session with direct append-only file writes."""

    def __init__(self, sample_fn: SampleFn, root_dir: str) -> None:
        self._sample_fn = sample_fn
        self._root_dir = os.path.abspath(os.path.expanduser(str(root_dir)))
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._streams: Dict[str, Dict[str, Any]] = {}
        self._collection_id: Optional[str] = None
        self._episode_id: Optional[str] = None
        self._names: list[str] = []
        self._output_dir: Optional[str] = None
        self._fps = 30.0
        self._max_duration_s: Optional[float] = None
        self._metadata: Dict[str, Any] = {}
        self._items: list[Dict[str, Any]] = []
        self._counts: Dict[str, int] = {}
        self._stream_stats: Dict[str, Dict[str, Any]] = {}
        self._started_at = 0.0
        self._stopped_at: Optional[float] = None
        self._stop_reason: Optional[str] = None
        self._last_error: Optional[str] = None

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(
        self,
        *,
        collection_id: str,
        episode_id: str,
        names: list[str],
        items: list[Dict[str, Any]],
        fps: float,
        max_duration_s: Optional[float],
        metadata: Dict[str, Any],
    ) -> Dict[str, Any]:
        collection = str(collection_id or "").strip()
        episode = str(episode_id or "").strip()
        if not collection:
            raise ValueError("collection_id is required")
        if not episode:
            raise ValueError("episode_id is required")
        target_fps = float(fps)
        if target_fps <= 0:
            raise ValueError("frame_rate must be greater than 0")
        duration = None if max_duration_s is None else float(max_duration_s)
        if duration is not None and duration <= 0:
            raise ValueError("max_duration must be greater than 0 when provided")

        with self._lock:
            if self.is_running:
                raise RuntimeError("collection already running")
            output_dir = os.path.join(
                self._root_dir, safe_name(collection), safe_name(episode)
            )
            if os.path.exists(output_dir):
                shutil.rmtree(output_dir)
            collection_metadata = dict(metadata)
            meta = {
                "collection_id": collection,
                "episode_id": episode,
                "names": list(names),
                "items": list(items),
                "fps": target_fps,
                "metadata": collection_metadata,
                "started_at_unix": time.time(),
            }
            os.makedirs(os.path.join(output_dir, "streams"), exist_ok=True)
            _write_json(os.path.join(output_dir, "collection_meta.json"), meta)
            self._streams = {}
            self._collection_id = collection
            self._episode_id = episode
            self._names = list(names)
            self._output_dir = output_dir
            self._fps = target_fps
            self._max_duration_s = duration
            self._metadata = dict(metadata)
            self._items = list(items)
            self._counts = {name: 0 for name in self._names}
            self._stream_stats = {
                name: {"count": 0, "first_ts": None, "last_ts": None, "fps": 0.0}
                for name in self._names
            }
            self._started_at = time.time()
            self._stopped_at = None
            self._stop_reason = None
            self._last_error = None
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._loop,
                name=f"collection-{collection}-{episode}",
                daemon=True,
            )
            self._thread.start()
            logger.info(
                "[CollectionService][START] collection=%s episode=%s fps=%.2f "
                "max_duration_s=%s names=%s output_dir=%s",
                collection,
                episode,
                target_fps,
                duration,
                self._names,
                output_dir,
            )
            return self.status()

    def stop(self) -> Dict[str, Any]:
        thread: Optional[threading.Thread]
        with self._lock:
            thread = self._thread
            if thread is None:
                return self.status()
            self._stop_event.set()
        thread.join(timeout=5.0)
        with self._lock:
            if thread.is_alive():
                self._stop_reason = "stop_pending"
                return self.status()
            self._close_streams()
            self._thread = None
            self._stopped_at = time.time()
            self._stop_reason = self._stop_reason or "manual"
            return self.status()

    def status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "running": self.is_running,
                "collection_id": self._collection_id,
                "episode_id": self._episode_id,
                "output_dir": self._output_dir,
                "names": list(self._names),
                "items": list(self._items),
                "counts": dict(self._counts),
                "stream_stats": {
                    name: dict(stats) for name, stats in self._stream_stats.items()
                },
                "duration_s": self._duration_s_unlocked(),
                "started_at_unix": self._started_at or None,
                "stopped_at_unix": self._stopped_at,
                "max_duration_s": self._max_duration_s,
                "stop_reason": self._stop_reason,
                "last_error": self._last_error,
                "metadata": dict(self._metadata),
            }

    def _loop(self) -> None:
        loop_fps = max(self._fps * 2.0, 60.0)
        interval = 1.0 / loop_fps
        last_ts: Dict[str, float] = {}
        start_monotonic = time.monotonic()
        next_t = start_monotonic
        last_progress_log = start_monotonic
        stop_reason = "manual"
        try:
            while not self._stop_event.is_set():
                if (
                    self._max_duration_s is not None
                    and time.monotonic() - start_monotonic >= self._max_duration_s
                ):
                    stop_reason = "max_duration"
                    break
                samples = self._sample_fn(self._names, last_ts)
                for name, sample in samples.items():
                    if not isinstance(sample, dict):
                        raise ValueError(f"collection sample '{name}' must be a dict")
                    ts = coerce_timestamp(
                        sample.get("timestamp"), context=f"collection sample '{name}'"
                    )
                    if "value" not in sample:
                        raise ValueError(f"collection sample '{name}' missing value")
                    if last_ts.get(name) == ts:
                        continue
                    self._write_sample(name, ts, sample["value"])
                    last_ts[name] = ts
                    self._record_sample_written(name, ts, sample["value"])
                now = time.monotonic()
                if now - last_progress_log >= 5.0:
                    with self._lock:
                        counts = dict(self._counts)
                        stream_fps = {
                            name: round(float(stats.get("fps") or 0.0), 2)
                            for name, stats in self._stream_stats.items()
                        }
                    logger.info(
                        "[CollectionService][PROGRESS] collection=%s episode=%s "
                        "duration_s=%.1f counts=%s stream_fps=%s",
                        self._collection_id,
                        self._episode_id,
                        now - start_monotonic,
                        counts,
                        stream_fps,
                    )
                    last_progress_log = now
                next_t += interval
                coarse = next_t - time.monotonic() - 0.001
                if coarse > 0:
                    if self._stop_event.wait(coarse):
                        break
                while time.monotonic() < next_t:
                    if self._stop_event.is_set():
                        break
                if time.monotonic() - next_t > interval:
                    next_t = time.monotonic()
        except Exception as exc:
            stop_reason = "error"
            logger.exception(
                "[CollectionService][LOOP_ERROR] collection=%s episode=%s error=%s; "
                "inspect the failing stream and storage path",
                self._collection_id,
                self._episode_id,
                exc,
            )
            with self._lock:
                self._last_error = str(exc)
        finally:
            self._stop_event.set()
            with self._lock:
                self._close_streams()
                self._stopped_at = time.time()
                self._stop_reason = stop_reason
                if self._thread is threading.current_thread():
                    self._thread = None
                counts = dict(self._counts)
                stream_fps = {
                    name: round(float(stats.get("fps") or 0.0), 2)
                    for name, stats in self._stream_stats.items()
                }
            logger.info(
                "[CollectionService][STOP] collection=%s episode=%s reason=%s "
                "duration_s=%.1f counts=%s stream_fps=%s",
                self._collection_id,
                self._episode_id,
                stop_reason,
                time.monotonic() - start_monotonic,
                counts,
                stream_fps,
            )

    def _write_sample(self, name: str, timestamp: float, value: Any) -> None:
        stream = self._stream(name)
        stream["file"].write(
            msgpack.packb(
                {"timestamp": timestamp, "value": _pack_safe(value)}, use_bin_type=True
            )
        )
        stream["count"] += 1
        stream["pending"] += 1
        if stream["pending"] >= 16 or time.monotonic() - stream["last_flush"] >= 0.1:
            self._flush_stream(stream)

    def _stream(self, name: str) -> Dict[str, Any]:
        stream = self._streams.get(name)
        if stream is not None:
            return stream
        if self._output_dir is None:
            raise RuntimeError("collection output_dir is not set")
        stream_dir = os.path.join(self._output_dir, "streams", safe_name(name))
        os.makedirs(stream_dir, exist_ok=True)
        stream = {
            "name": name,
            "dir": stream_dir,
            "file": open(os.path.join(stream_dir, "samples.msgpack"), "ab"),
            "count": 0,
            "pending": 0,
            "last_flush": time.monotonic(),
        }
        self._streams[name] = stream
        return stream

    def _flush_stream(self, stream: Dict[str, Any]) -> None:
        stream["file"].flush()
        stream["pending"] = 0
        stream["last_flush"] = time.monotonic()

    def _close_streams(self) -> None:
        for stream in self._streams.values():
            try:
                self._flush_stream(stream)
                stream["file"].close()
            except OSError:
                pass
        self._streams.clear()

    def _record_sample_written(self, name: str, timestamp: float, value: Any) -> None:
        with self._lock:
            self._counts[name] = self._counts.get(name, 0) + 1
            stats = self._stream_stats.setdefault(
                name,
                {"count": 0, "first_ts": None, "last_ts": None, "fps": 0.0},
            )
            count = int(stats.get("count") or 0) + 1
            first_ts = stats.get("first_ts")
            if first_ts is None:
                first_ts = float(timestamp)
            last_ts = float(timestamp)
            elapsed = max(0.0, last_ts - float(first_ts))
            stats.update(
                {
                    "count": count,
                    "first_ts": float(first_ts),
                    "last_ts": last_ts,
                    "fps": ((count - 1) / elapsed)
                    if count > 1 and elapsed > 0
                    else 0.0,
                }
            )
            if count == 1:
                logger.info(
                    "[CollectionService][FIRST_SAMPLE] collection=%s episode=%s "
                    "name=%s timestamp=%.6f",
                    self._collection_id,
                    self._episode_id,
                    name,
                    timestamp,
                )
                logger.info(
                    "[CollectionService][FIRST_VALUE] name=%s %s",
                    name,
                    _sample_value_diagnostics(value),
                )

    def _duration_s_unlocked(self) -> float:
        if not self._started_at:
            return 0.0
        end = self._stopped_at or time.time()
        return max(0.0, end - self._started_at)


DEFAULT_MAX_COLLECTION_DURATION_S = 10 * 60


class CollectionService(BaseService):
    """Record configured Observation and Action objects as datasets."""

    def __init__(
        self,
        bus: ToolBus,
        inputs: Iterable[RunnerInputSpec],
        root_dir: str,
        resources: ResourceRegistry,
    ) -> None:
        super().__init__(bus, "collection_service")
        self._inputs = list(inputs)
        self._input_by_name = {
            _collection_object_name(spec): spec for spec in self._inputs
        }
        self._item_by_name = {
            name: _collection_item_descriptor(spec)
            for name, spec in self._input_by_name.items()
        }
        self._root_dir = os.path.abspath(os.path.expanduser(str(root_dir)))
        self._resources = resources
        for spec in self._inputs:
            self.subscribe_channel(spec.channel, spec.msg_size, spec.channel_transport)
        self._payload_parser = CachedSharedRefParser()
        self._session = _CollectionSession(self._sample, root_dir=self._root_dir)
        self._sample_error_logs: Dict[str, LogGate] = {}

    def bind(self) -> None:
        self._register_tool(
            START_COLLECTION.name,
            self.start_collection,
            input_schema=START_COLLECTION.input_schema,
            output_schema=START_COLLECTION.output_schema,
            description=START_COLLECTION.description,
        )
        self._register_tool(
            STOP_COLLECTION.name,
            self.stop_collection,
            input_schema=STOP_COLLECTION.input_schema,
            output_schema=STOP_COLLECTION.output_schema,
            description=STOP_COLLECTION.description,
        )
        self._register_tool(
            GET_COLLECTION_STATUS.name,
            self.get_collection_status,
            input_schema=GET_COLLECTION_STATUS.input_schema,
            output_schema=GET_COLLECTION_STATUS.output_schema,
            description=GET_COLLECTION_STATUS.description,
        )
        self._register_tool(
            DELETE_COLLECTION.name,
            self.delete_collection,
            input_schema=DELETE_COLLECTION.input_schema,
            output_schema=DELETE_COLLECTION.output_schema,
            description=DELETE_COLLECTION.description,
        )

    def start_collection(
        self,
        names: list[str],
        collection_id: str,
        episode_id: str,
        task_prompt: str,
        task_description: str,
        frame_rate: Optional[float] = None,
        max_duration: Optional[float] = DEFAULT_MAX_COLLECTION_DURATION_S,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        if not names:
            return ToolBus.make_result(
                False, result={}, message="names must be a non-empty list"
            )
        requested_names = [str(name) for name in names]
        duplicate_names = sorted(
            {name for name in requested_names if requested_names.count(name) > 1}
        )
        if duplicate_names:
            return ToolBus.make_result(
                False,
                result={"duplicate_names": duplicate_names},
                message=f"Duplicate collection name(s): {', '.join(duplicate_names)}",
            )
        unknown_names = sorted(
            name for name in requested_names if name not in self._input_by_name
        )
        if unknown_names:
            return ToolBus.make_result(
                False,
                result={
                    "unknown_names": unknown_names,
                    "available_names": sorted(self._input_by_name),
                },
                message=f"Unknown collection name(s): {', '.join(unknown_names)}",
            )
        if not str(collection_id or "").strip():
            return ToolBus.make_result(
                False, result={}, message="collection_id is required"
            )
        if not str(episode_id or "").strip():
            return ToolBus.make_result(
                False, result={}, message="episode_id is required"
            )
        if not str(task_prompt or "").strip():
            return ToolBus.make_result(
                False, result={}, message="task_prompt is required"
            )
        if not str(task_description or "").strip():
            return ToolBus.make_result(
                False, result={}, message="task_description is required"
            )
        try:
            requested_items = [
                dict(self._item_by_name[name]) for name in requested_names
            ]
            collection_metadata = dict(metadata or {})
            collection_metadata.update(
                {
                    "task_prompt": str(task_prompt),
                    "task_description": str(task_description),
                    "items": requested_items,
                }
            )
            status = self._session.start(
                collection_id=str(collection_id),
                episode_id=str(episode_id),
                names=requested_names,
                items=requested_items,
                fps=float(frame_rate) if frame_rate is not None else 30.0,
                max_duration_s=DEFAULT_MAX_COLLECTION_DURATION_S
                if max_duration is None
                else max_duration,
                metadata=collection_metadata,
            )
            collection_resource = self._collection_resource(status, mode="live")
            return ToolBus.make_result(
                True,
                result={
                    "collection_id": str(status.get("collection_id") or ""),
                    "episode_id": str(status.get("episode_id") or ""),
                    "collection_resource": collection_resource,
                    "names": list(status.get("names") or []),
                    "task_prompt": str(task_prompt),
                    "task_description": str(task_description),
                    "started_at": float(status.get("started_at_unix") or 0.0),
                },
                message="OK",
            )
        except Exception as exc:
            logger.exception(
                "[CollectionService][START_FAILED] collection=%s episode=%s names=%s error=%s; "
                "verify collection identifiers, streams, and output permissions",
                collection_id,
                episode_id,
                requested_names,
                exc,
            )
            return ToolBus.make_result(
                False, result=self.get_collection_status()["result"], message=str(exc)
            )

    def stop_collection(self) -> Dict[str, Any]:
        status = self._session.stop()
        collection_resource = (
            self._collection_resource(status, mode="snapshot")
            if status.get("output_dir")
            else None
        )
        return ToolBus.make_result(
            True,
            result={
                "collection_id": str(status.get("collection_id") or ""),
                "episode_id": str(status.get("episode_id") or ""),
                "duration": float(status.get("duration_s") or 0.0),
                "per_name_counts": dict(status.get("counts") or {}),
                "stream_stats": dict(status.get("stream_stats") or {}),
                "collection_resource": collection_resource or {},
            },
            message="OK",
        )

    def get_collection_status(self) -> Dict[str, Any]:
        status = self._session.status()
        result: Dict[str, Any] = {"running": bool(status.get("running"))}
        if status.get("collection_id") and status.get("episode_id"):
            metadata = status.get("metadata")
            metadata = metadata if isinstance(metadata, dict) else {}
            result.update(
                {
                    "collection_id": str(status["collection_id"]),
                    "episode_id": str(status["episode_id"]),
                    "collection_resource": self._collection_resource(
                        status, mode="live" if status.get("running") else "snapshot"
                    ),
                    "names": list(status.get("names") or []),
                    "counts": dict(status.get("counts") or {}),
                    "stream_stats": dict(status.get("stream_stats") or {}),
                    "duration": float(status.get("duration_s") or 0.0),
                    "task_prompt": str(metadata.get("task_prompt") or ""),
                    "task_description": str(metadata.get("task_description") or ""),
                }
            )
        return ToolBus.make_result(True, result=result, message="OK")

    def delete_collection(self, resource_id: str) -> Dict[str, Any]:
        rid = str(resource_id or "").strip()
        if not rid:
            return ToolBus.make_result(
                False, result={"deleted": False}, message="resource_id is required"
            )
        status = self._session.status()
        if status.get("running") and status.get("output_dir"):
            running_resource = self._collection_resource(status, mode="live")
            if running_resource.get("resource_id") == rid:
                return ToolBus.make_result(
                    False,
                    result={"deleted": False},
                    message="collection is currently being collected",
                )
        try:
            record = self._resources.resolve(rid)
            if record.metadata.get("kind") != "collection":
                return ToolBus.make_result(
                    False,
                    result={"deleted": False},
                    message="resource is not a collection",
                )
            deleted = self._resources.delete(rid)
        except Exception as exc:
            logger.warning(
                "[CollectionService][DELETE_FAILED] resource_id=%s error=%s; "
                "verify the resource exists and is writable",
                rid,
                exc,
                exc_info=True,
            )
            return ToolBus.make_result(
                False, result={"deleted": False}, message=str(exc)
            )
        return ToolBus.make_result(True, result={"deleted": deleted}, message="OK")

    def unbind(self) -> None:
        if self._session.is_running:
            self._session.stop()
        super().unbind()

    def _sample(
        self,
        names: list[str],
        last_timestamps: Optional[Dict[str, float]] = None,
    ) -> Dict[str, Dict[str, Any]]:
        requested = set(names)
        seen = last_timestamps or {}
        samples: Dict[str, Dict[str, Any]] = {}
        for name, spec in self._input_by_name.items():
            if name not in requested:
                continue
            ts, payload = self.latest_parsed(spec.channel)
            if ts is None or payload is None:
                continue
            if seen.get(name) == ts:
                continue
            try:
                value = self._payload_parser.parse(spec, payload)
                value = _collection_value(spec, value)
            except (SharedDataExpired, KeyError, TypeError, ValueError) as exc:
                gate = self._sample_error_logs.setdefault(
                    name,
                    LogGate(
                        logger,
                        f"CollectionService/SAMPLE_SKIPPED/{name}",
                        interval_s=5.0,
                        level=logging.WARNING,
                    ),
                )
                gate.failure(
                    "name=%s channel=%s error=%s; inspect the publisher and shared-data lifetime",
                    name,
                    spec.channel,
                    exc,
                )
                continue
            gate = self._sample_error_logs.pop(name, None)
            if gate is not None:
                gate.success()
            samples[name] = {"timestamp": float(ts), "value": value}
        return samples

    def _collection_resource(
        self, status: Dict[str, Any], *, mode: str
    ) -> Dict[str, Any]:
        output_dir = str(status.get("output_dir") or "")
        if not output_dir:
            return {}
        if not os.path.exists(output_dir):
            return {}
        collection_id = str(status.get("collection_id") or "")
        episode_id = str(status.get("episode_id") or "")
        return self._resources.register_path(
            output_dir,
            resource_type="directory",
            domain="data",
            name=f"{collection_id}/{episode_id}"
            if collection_id and episode_id
            else os.path.basename(output_dir),
            format="directory",
            mode=mode,
            metadata={
                "kind": "collection",
                "collection_id": collection_id,
                "episode_id": episode_id,
                "local_path": os.path.abspath(output_dir),
            },
        )


def _collection_object_name(spec: RunnerInputSpec) -> str:
    info = spec.info if isinstance(spec.info, dict) else {}
    if info.get("rcp_action_name"):
        return str(info["rcp_action_name"])
    return _protocol_observation_name(spec, _observation_type(spec))


def _collection_item_descriptor(spec: RunnerInputSpec) -> Dict[str, Any]:
    info = spec.info if isinstance(spec.info, dict) else {}
    protocol_name = _collection_object_name(spec)
    parts = protocol_name.split(".")
    category = "action" if info.get("rcp_action_name") else "observation"
    component_name = str(
        info.get("component_name") or (parts[1] if len(parts) >= 3 else "")
    )
    short_name = parts[2] if len(parts) >= 3 else protocol_name
    item_type = str(
        info.get("rcp_action_type") or info.get("rcp_observation_type") or short_name
    )
    descriptor: Dict[str, Any] = {
        "protocol_name": protocol_name,
        "category": category,
        "component_name": component_name,
        "name": short_name,
        "type": item_type,
    }
    description = info.get("description")
    if description:
        descriptor["description"] = str(description)
    frame_rate = info.get("frame_rate")
    if frame_rate is not None:
        descriptor["frame_rate"] = float(frame_rate)
    return descriptor


def _collection_value(spec: RunnerInputSpec, value: Any) -> Any:
    info = spec.info if isinstance(spec.info, dict) else {}
    action_type = info.get("rcp_action_type")
    if action_type:
        return protocol_action_value(
            str(action_type), value, str(info.get("rcp_action_name") or "")
        )
    if info.get("rcp_observation_type"):
        return protocol_observation_value(str(info["rcp_observation_type"]), value)
    return value
