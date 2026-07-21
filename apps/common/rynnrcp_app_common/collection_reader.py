"""Reader for RCP collection directories."""

from __future__ import annotations

import base64
import json
import os
from contextlib import nullcontext
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional

import msgpack

from rynnrcp.utils import coerce_timestamp, safe_name


@dataclass
class RawCaptureSample:
    key: str
    seq: int
    timestamp: float
    data: bytes
    meta: Dict[str, Any]


@dataclass(frozen=True)
class RawCaptureSampleRef:
    key: str
    seq: int
    timestamp: float
    offset: int
    length: int
    meta: Dict[str, Any]


class RawCaptureReader:
    """Small compatibility reader over collection ``samples.msgpack`` streams."""

    def __init__(self, capture_dir: str) -> None:
        self.capture_dir = os.path.abspath(os.path.expanduser(capture_dir))
        self.streams_dir = os.path.join(self.capture_dir, "streams")
        self._cache: Dict[str, List[RawCaptureSample]] = {}

    def capture_meta(self) -> Dict[str, Any]:
        for name in ("collection_meta.json", "capture_meta.json"):
            path = os.path.join(self.capture_dir, name)
            if os.path.isfile(path):
                with open(path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                return data if isinstance(data, dict) else {}
        return {}

    def stream_keys(self) -> List[str]:
        names = self.capture_meta().get("names")
        if isinstance(names, list):
            return [
                str(name)
                for name in names
                if os.path.isfile(os.path.join(self.streams_dir, safe_name(str(name)), "samples.msgpack"))
            ]
        if not os.path.isdir(self.streams_dir):
            return []
        return sorted(
            name for name in os.listdir(self.streams_dir)
            if os.path.isfile(os.path.join(self.streams_dir, name, "samples.msgpack"))
        )

    def read_stream(self, key: str) -> List[RawCaptureSample]:
        return list(self._samples(key))

    def read_stream_index(self, key: str) -> List[RawCaptureSampleRef]:
        return [
            RawCaptureSampleRef(
                key=sample.key,
                seq=sample.seq,
                timestamp=sample.timestamp,
                offset=index,
                length=len(sample.data),
                meta=dict(sample.meta),
            )
            for index, sample in enumerate(self._samples(key))
        ]

    def read_all_indexes(self, keys: Optional[Iterable[str]] = None) -> Dict[str, List[RawCaptureSampleRef]]:
        selected = list(keys) if keys is not None else self.stream_keys()
        return {key: self.read_stream_index(key) for key in selected}

    def read_ref(self, ref: RawCaptureSampleRef, data_file: Any = None) -> RawCaptureSample:
        return self._samples(ref.key)[int(ref.offset)]

    def open_stream_data(self, key: str) -> Any:
        return nullcontext(None)

    def read_all(self, keys: Optional[Iterable[str]] = None) -> Dict[str, List[RawCaptureSample]]:
        selected = list(keys) if keys is not None else self.stream_keys()
        return {key: self.read_stream(key) for key in selected}

    def decode_sample(self, sample: RawCaptureSample) -> Any:
        if sample.meta.get("type") == "image":
            return {
                "width": sample.meta.get("width"),
                "height": sample.meta.get("height"),
                "encoding": sample.meta.get("encoding"),
                "image": sample.data,
                "size_bytes": len(sample.data),
            }
        return msgpack.unpackb(sample.data, raw=False)

    def _samples(self, key: str) -> List[RawCaptureSample]:
        cached = self._cache.get(key)
        if cached is not None:
            return cached
        path = os.path.join(self.streams_dir, safe_name(key), "samples.msgpack")
        samples: List[RawCaptureSample] = []
        if not os.path.isfile(path):
            self._cache[key] = samples
            return samples
        with open(path, "rb") as f:
            for seq, item in enumerate(msgpack.Unpacker(f, raw=False)):
                if not isinstance(item, dict):
                    continue
                timestamp = coerce_timestamp(item.get("timestamp"), context=f"collection stream '{key}'")
                samples.append(_sample_from_value(key, seq, timestamp, item.get("value")))
        self._cache[key] = samples
        return samples


def _sample_from_value(key: str, seq: int, timestamp: float, value: Any) -> RawCaptureSample:
    if isinstance(value, dict) and "image" in value:
        data = _image_bytes(value["image"])
        meta = {
            "type": "image",
            "width": int(value.get("width") or 0),
            "height": int(value.get("height") or 0),
            "encoding": str(value.get("encoding") or "bytes"),
            "channels": int(value.get("channels") or 3),
        }
        return RawCaptureSample(key=key, seq=seq, timestamp=timestamp, data=data, meta=meta)
    return RawCaptureSample(
        key=key,
        seq=seq,
        timestamp=timestamp,
        data=msgpack.packb(value, use_bin_type=True),
        meta={"encoding": "msgpack"},
    )


def _image_bytes(value: Any) -> bytes:
    if isinstance(value, bytes):
        return value
    if isinstance(value, bytearray):
        return bytes(value)
    if isinstance(value, memoryview):
        return value.tobytes()
    if isinstance(value, str):
        try:
            return base64.b64decode(value, validate=True)
        except Exception:
            return value.encode("utf-8")
    if hasattr(value, "tobytes"):
        return value.tobytes()
    raise TypeError("image value must be bytes or base64 string")
