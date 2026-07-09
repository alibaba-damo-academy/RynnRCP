"""Shared helpers for channel payload packing and parsing."""

from __future__ import annotations

import json
import struct
import time
from typing import Any, Dict, MutableMapping

from rynnrcp.utils.shared_data_store import (
    PAYLOAD_REF_PREFIX,
    PAYLOAD_REF_TYPE,
    SharedDataStore,
)

INLINE_JSON_PAYLOAD_MAX_BYTES = 4096
TIMESTAMP_HEADER = "<d"
BytesPart = bytes | bytearray | memoryview


def pack_channel_message(timestamp: float, payload: bytes) -> bytes:
    """Prefix a channel payload with the standard timestamp header."""
    return struct.pack(TIMESTAMP_HEADER, float(timestamp)) + payload


def json_dumps_bytes(payload: Any) -> bytes:
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    raise TypeError(f"value is not JSON compatible: {type(value).__name__}")


def bytes_payload(msg: Any) -> tuple[float, bytes]:
    """Normalize connector bytes payloads into channel payload bytes."""
    if isinstance(msg, (bytes, bytearray, memoryview)):
        return time.time(), bytes(msg)
    if isinstance(msg, (list, tuple)) and len(msg) == 3:
        success, timestamp, data = msg
        if not success:
            return float(timestamp), b""
        return float(timestamp), bytes(data)
    raise TypeError("bytes payload connector must return bytes or (ok, ts, bytes)")


def shared_payload_ref_payload(
    *,
    store: SharedDataStore,
    object_name: str,
    payload_type: str,
    timestamp: float,
    data: bytes | bytearray | memoryview,
    codec: str,
    meta: Dict[str, Any] | None = None,
) -> bytes:
    """Write payload bytes to shared memory and return a small channel ref."""
    ref = store.write(data, timestamp=timestamp, codec=codec, meta=meta or {}).to_dict()
    ref["payload_type"] = payload_type
    ref["object_name"] = object_name
    return json_dumps_bytes(ref)


def bytes_shared_payload(
    msg: Any,
    object_name: str,
    store: SharedDataStore,
) -> tuple[float, bytes]:
    """Pack bytes connector payload through the shared data store."""
    timestamp, data = bytes_payload(msg)
    if not data:
        return timestamp, b""
    return timestamp, shared_payload_ref_payload(
        store=store,
        object_name=object_name,
        payload_type="bytes",
        timestamp=timestamp,
        data=data,
        codec="bytes",
    )


def json_shared_payload(msg: Any, object_name: str, timestamp: float, store: SharedDataStore) -> bytes:
    """Pack JSON observation payload through the shared data store."""
    data = json_observation_payload(msg, object_name, timestamp)
    return shared_payload_ref_payload(
        store=store,
        object_name=object_name,
        payload_type="json",
        timestamp=timestamp,
        data=data,
        codec="json_observation",
    )


def image_adapter_shared_payload(
    msg: Any,
    input_adapter: Any,
    object_name: str,
    store: SharedDataStore,
) -> tuple[float, bytes]:
    """Pack image metadata into the channel and store image bytes in SHM."""
    timestamp, image = _image_message_to_dict(msg, input_adapter, object_name)
    if not image:
        return timestamp, b""
    image_bytes = _as_byte_part(image["image"])
    meta = {
        "width": int(image["width"]),
        "height": int(image["height"]),
        "encoding": str(image["encoding"]),
        "size_bytes": len(image_bytes),
    }
    return timestamp, shared_payload_ref_payload(
        store=store,
        object_name=object_name,
        payload_type="image",
        timestamp=timestamp,
        data=image_bytes,
        codec="image",
        meta=meta,
    )


def json_observation_payload(msg: Any, object_name: str, timestamp: float) -> bytes:
    """Pack a parsed observation into the standard JSON channel payload."""
    if isinstance(msg, (bytes, bytearray, memoryview)):
        try:
            decoded = json.loads(bytes(msg).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            decoded = bytes(msg).decode("utf-8", errors="replace")
        msg = decoded
    if isinstance(msg, dict) and object_name in msg:
        payload = {"timestamp": timestamp, "extra": json_safe(msg)}
    else:
        payload = {"timestamp": timestamp, "extra": {object_name: json_safe(msg)}}
    return json_dumps_bytes(payload)


def decode_action_payload(
    payload: bytes,
    channel_name: str,
    reader_cache: MutableMapping[str, SharedDataStore] | None = None,
) -> tuple[Dict[str, Any], Dict[str, Any]]:
    """Decode and validate one action-channel JSON payload."""
    if payload.startswith(PAYLOAD_REF_PREFIX):
        payload = _read_shared_ref_bytes(channel_name, payload, reader_cache=reader_cache)
    try:
        data = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"Action channel {channel_name} must publish JSON") from exc
    if not isinstance(data, dict):
        raise ValueError(f"Action channel {channel_name} payload must be a JSON object")
    extra = data.get("extra")
    if not isinstance(extra, dict):
        raise ValueError(f"Action channel {channel_name} payload.extra must be a JSON object")
    action_name = extra.get("action_name")
    if not isinstance(action_name, str) or not action_name:
        raise ValueError(f"Action channel {channel_name} payload.extra missing action_name")
    if action_name not in extra:
        raise ValueError(f"Action channel {channel_name} payload.extra missing {action_name}")
    if "fps" not in extra:
        raise ValueError(f"Action channel {channel_name} payload.extra missing fps")
    if "action_type" not in extra:
        raise ValueError(f"Action channel {channel_name} payload.extra missing action_type")
    return data, extra


def parse_channel_payload(
    spec: Any,
    payload: bytes,
    reader_cache: MutableMapping[str, SharedDataStore] | None = None,
) -> Any:
    """Parse one runner payload according to its input spec."""
    if not payload.startswith(PAYLOAD_REF_PREFIX):
        return _parse_json_observation_bytes(spec, payload)
    return _parse_shared_payload_ref(spec, payload, reader_cache=reader_cache)


def _parse_json_observation_bytes(spec: Any, payload: bytes) -> Any:
    """Parse JSON observation bytes loaded from the shared data store."""
    try:
        data = json.loads(payload.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise ValueError(
            f"Channel {spec.channel} must publish a JSON state payload for object_name {spec.object_name}"
        ) from exc

    if not isinstance(data, dict):
        raise ValueError(f"Channel {spec.channel} payload must be a JSON object")

    extra = data.get("extra")
    if not isinstance(extra, dict):
        raise ValueError(f"Channel {spec.channel} payload.extra must be a JSON object")
    if spec.object_name not in extra:
        raise ValueError(
            f"Channel {spec.channel} payload.extra missing configured object_name {spec.object_name}"
        )
    return extra[spec.object_name]


def _parse_shared_payload_ref(
    spec: Any,
    payload: bytes,
    reader_cache: MutableMapping[str, SharedDataStore] | None = None,
) -> Any:
    try:
        ref = json.loads(payload.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise ValueError(f"Channel {spec.channel} payload ref must be JSON") from exc
    if not isinstance(ref, dict) or ref.get("type") != PAYLOAD_REF_TYPE:
        raise ValueError(f"Channel {spec.channel} payload ref has unsupported type")
    if ref.get("object_name") != spec.object_name:
        raise ValueError(
            f"Channel {spec.channel} payload ref object_name mismatch: {ref.get('object_name')} != {spec.object_name}"
        )
    data = _read_shared_ref_data(ref, reader_cache=reader_cache)
    payload_type = str(ref.get("payload_type") or "")
    if payload_type == "image":
        meta = ref.get("meta") if isinstance(ref.get("meta"), dict) else {}
        return {
            "width": int(meta["width"]),
            "height": int(meta["height"]),
            "encoding": str(meta["encoding"]),
            "image": data,
        }
    if payload_type == "bytes":
        return data
    if payload_type == "json":
        return _parse_json_observation_bytes(spec, data)
    raise ValueError(f"Channel {spec.channel} payload ref has unsupported payload_type: {payload_type}")


def _read_shared_ref_bytes(
    channel_name: str,
    payload: bytes,
    reader_cache: MutableMapping[str, SharedDataStore] | None = None,
) -> bytes:
    try:
        ref = json.loads(payload.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise ValueError(f"Channel {channel_name} payload ref must be JSON") from exc
    if not isinstance(ref, dict) or ref.get("type") != PAYLOAD_REF_TYPE:
        raise ValueError(f"Channel {channel_name} payload ref has unsupported type")
    return _read_shared_ref_data(ref, reader_cache=reader_cache)


def _read_shared_ref_data(
    ref: Dict[str, Any],
    reader_cache: MutableMapping[str, SharedDataStore] | None = None,
) -> bytes:
    if reader_cache is None:
        store = SharedDataStore.open_reader(ref)
        try:
            return store.read(ref)
        finally:
            store.close()

    name = str(ref["name"])
    store = reader_cache.get(name)
    if store is None:
        store = SharedDataStore.open_reader(ref)
        reader_cache[name] = store
    try:
        return store.read(ref)
    except SharedDataExpired:
        try:
            store.close()
        finally:
            reader_cache.pop(name, None)
        store = SharedDataStore.open_reader(ref)
        reader_cache[name] = store
        return store.read(ref)


def close_shared_reader_cache(reader_cache: MutableMapping[str, SharedDataStore]) -> None:
    for store in list(reader_cache.values()):
        try:
            store.close()
        except Exception:
            pass
    reader_cache.clear()


class CachedSharedRefParser:
    """Stateful payload parser that caches SharedDataStore readers per channel.

    Equivalent to ``parse_channel_payload`` but keeps each shared-memory
    region attached across calls. ARM Linux ``shm_open + mmap + munmap``
    costs ~1-2ms per attach; for high-frequency consumers (e.g. recording)
    that adds up to ~8ms per tick across 4 channels. This cache makes each
    subsequent read effectively a memory copy (<10us).

    On ``SharedDataExpired`` (publisher recreated its store), the stale
    reader is dropped so the next call re-attaches transparently.

    Use one parser per long-lived consumer; call ``close()`` on shutdown.
    Not thread-safe — caller must serialize ``parse()`` calls.
    """

    def __init__(self) -> None:
        self._readers: Dict[str, SharedDataStore] = {}

    def parse(self, spec: Any, payload: bytes) -> Any:
        if not payload.startswith(PAYLOAD_REF_PREFIX):
            return _parse_json_observation_bytes(spec, payload)
        try:
            ref = json.loads(payload.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            raise ValueError(f"Channel {spec.channel} payload ref must be JSON") from exc
        if not isinstance(ref, dict) or ref.get("type") != PAYLOAD_REF_TYPE:
            raise ValueError(f"Channel {spec.channel} payload ref has unsupported type")
        if ref.get("object_name") != spec.object_name:
            raise ValueError(
                f"Channel {spec.channel} payload ref object_name mismatch: {ref.get('object_name')} != {spec.object_name}"
            )
        name = str(ref.get("name") or "")
        if not name:
            raise ValueError(f"Channel {spec.channel} payload ref missing 'name'")
        store = self._readers.get(name)
        if store is None:
            store = SharedDataStore.open_reader(ref)
            self._readers[name] = store
        try:
            data = store.read(ref)
        except SharedDataExpired:
            # Publisher likely recreated its store — drop the stale reader so
            # the next call will re-attach. Re-raise so the caller can skip
            # this tick (collection treats it as "no fresh sample").
            self._readers.pop(name, None)
            try:
                store.close()
            except Exception:
                pass
            raise
        payload_type = str(ref.get("payload_type") or "")
        if payload_type == "image":
            meta = ref.get("meta") if isinstance(ref.get("meta"), dict) else {}
            return {
                "width": int(meta["width"]),
                "height": int(meta["height"]),
                "encoding": str(meta["encoding"]),
                "image": data,
            }
        if payload_type == "bytes":
            return data
        if payload_type == "json":
            return _parse_json_observation_bytes(spec, data)
        raise ValueError(
            f"Channel {spec.channel} payload ref has unsupported payload_type: {payload_type}"
        )

    def close(self) -> None:
        for store in self._readers.values():
            try:
                store.close()
            except Exception:
                pass
        self._readers.clear()


def _image_message_to_dict(msg: Any, input_adapter: Any, object_name: str) -> tuple[float, Dict[str, Any]]:
    if isinstance(msg, (list, tuple)) and len(msg) == 5:
        success, width, height, encoding, data = msg
        if not success:
            return time.time(), {}
        return time.time(), {
            "width": int(width),
            "height": int(height),
            "encoding": str(encoding),
            "image": data,
        }

    timestamp, parsed = input_adapter.parse(msg)
    if object_name not in parsed:
        return timestamp, {}
    image = parsed[object_name]
    return timestamp, {
        "width": int(image["width"]),
        "height": int(image["height"]),
        "encoding": str(image["encoding"]),
        "image": image["image"],
    }


def _as_byte_part(data: Any) -> BytesPart:
    if isinstance(data, bytes):
        return data
    if isinstance(data, bytearray):
        return data
    if isinstance(data, memoryview):
        return _cast_memoryview_to_bytes(data)
    try:
        view = memoryview(data)
    except TypeError:
        return bytes(data)
    try:
        return _cast_memoryview_to_bytes(view)
    except TypeError:
        if hasattr(data, "tobytes"):
            return data.tobytes()
        return bytes(data)


def _cast_memoryview_to_bytes(view: memoryview) -> memoryview:
    if not view.c_contiguous:
        raise TypeError("non-contiguous buffer")
    if view.format == "B" and view.ndim == 1:
        return view
    return view.cast("B")
