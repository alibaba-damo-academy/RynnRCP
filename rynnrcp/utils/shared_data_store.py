"""Shared-memory latest-frame store for runner payload bytes.

Each logical key owns one fixed-size shared memory region. The region stores a
small slot table plus a circular byte area; each write appends one variable-size
payload and updates one of a fixed number of slots. Readers validate the slot
before and after copying bytes so partially overwritten payloads are rejected.
"""

from __future__ import annotations

import hashlib
import os
import struct
import threading
import uuid
from dataclasses import dataclass
from typing import Any, Dict

from rynnrcp.native import shm_close, shm_create, shm_open, shm_unlink


DEFAULT_SHARED_DATA_BUFFER_SIZE = 100 * 1024 * 1024
DEFAULT_SHARED_DATA_SLOT_COUNT = 10

ENV_SHARED_DATA_BUFFER_SIZE = "RYNNRCP_SHARED_DATA_BUFFER_SIZE"
ENV_SHARED_DATA_SLOT_COUNT = "RYNNRCP_SHARED_DATA_SLOT_COUNT"

PAYLOAD_REF_TYPE = "rynnrcp.shared_payload_ref.v1"
PAYLOAD_REF_PREFIX = b'{"type":"rynnrcp.shared_payload_ref.v1"'

_HEADER_SIZE = 4096
_MAGIC = 0x5259435053484453  # "RYCPSHDS"
_VERSION = 1
_HEADER = struct.Struct("<QIIQQQQQ")
# magic, version, slot_count, total_size, data_offset, write_seq, write_cursor, store_id
_HEADER_MAGIC_OFF = 0
_HEADER_SLOT_COUNT_OFF = 12
_HEADER_TOTAL_SIZE_OFF = 16
_HEADER_DATA_OFFSET_OFF = 24
_HEADER_WRITE_SEQ_OFF = 32
_HEADER_WRITE_CURSOR_OFF = 40
_HEADER_STORE_ID_OFF = 48

_SLOT_SIZE = 64
_SLOT = struct.Struct("<QQQdQ")
# seq, offset, size, timestamp, status
_STATUS_INVALID = 0
_STATUS_READY = 1


class SharedDataExpired(RuntimeError):
    """Raised when a referenced payload was overwritten before it was read."""


@dataclass(frozen=True)
class SharedDataRef:
    key: str
    name: str
    seq: int
    slot: int
    size: int
    timestamp: float
    codec: str
    meta: Dict[str, Any]
    buffer_size: int
    slot_count: int
    store_id: int

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": PAYLOAD_REF_TYPE,
            "key": self.key,
            "name": self.name,
            "seq": self.seq,
            "slot": self.slot,
            "size": self.size,
            "timestamp": self.timestamp,
            "codec": self.codec,
            "meta": dict(self.meta),
            "buffer_size": self.buffer_size,
            "slot_count": self.slot_count,
            "store_id": self.store_id,
        }


class SharedDataStore:
    """One shared-memory variable-size latest buffer for a logical key."""

    def __init__(
        self,
        key: str,
        *,
        name: str | None = None,
        create: bool = False,
        buffer_size: int | None = None,
        slot_count: int | None = None,
    ) -> None:
        self.key = str(key)
        self.name = name or shared_data_name(self.key)
        self.buffer_size = int(buffer_size or _env_int(ENV_SHARED_DATA_BUFFER_SIZE, DEFAULT_SHARED_DATA_BUFFER_SIZE))
        self.slot_count = int(slot_count or _env_int(ENV_SHARED_DATA_SLOT_COUNT, DEFAULT_SHARED_DATA_SLOT_COUNT))
        if self.slot_count <= 0:
            raise ValueError("slot_count must be greater than 0")
        min_size = _HEADER_SIZE + self.slot_count * _SLOT_SIZE + 1
        if self.buffer_size < min_size:
            raise ValueError(f"buffer_size must be at least {min_size}")
        self._lock = threading.Lock()
        self._closed = False
        self._owner = bool(create)
        if create:
            try:
                shm_unlink(self.name)
            except Exception:
                pass
            self._region = shm_create(self.name, self.buffer_size)
            if not self._region.is_valid():
                raise RuntimeError(f"failed to create shared data store: {self.name}")
            self.name = self._region.name
            self._init_header()
        else:
            self._region = shm_open(self.name, self.buffer_size)
            if not self._region.is_valid():
                raise RuntimeError(f"failed to open shared data store: {self.name}")
            self.name = self._region.name
            self._validate_header()

    @classmethod
    def open_reader(cls, ref: Dict[str, Any]) -> "SharedDataStore":
        name = str(ref["name"])
        key = str(ref.get("key") or name)
        return SharedDataStore(
            key,
            name=name,
            create=False,
            buffer_size=int(ref.get("buffer_size") or DEFAULT_SHARED_DATA_BUFFER_SIZE),
            slot_count=int(ref.get("slot_count") or DEFAULT_SHARED_DATA_SLOT_COUNT),
        )

    def write(
        self,
        data: bytes | bytearray | memoryview,
        *,
        timestamp: float,
        codec: str,
        meta: Dict[str, Any] | None = None,
    ) -> SharedDataRef:
        payload = memoryview(data)
        size = len(payload)
        if size > self.data_capacity:
            raise ValueError(
                f"payload for {self.key} exceeds shared buffer capacity: {size} > {self.data_capacity}"
            )
        with self._lock:
            seq = self._read_u64(_HEADER_WRITE_SEQ_OFF) + 1
            slot = int((seq - 1) % self.slot_count)
            cursor = self._read_u64(_HEADER_WRITE_CURSOR_OFF)
            if cursor + size > self.data_capacity:
                cursor = 0
            self._invalidate_overlaps(cursor, size)
            self._write_slot(slot, 0, cursor, size, timestamp, _STATUS_INVALID)
            self._region.write_bytes(self.data_offset + cursor, payload)
            self._write_slot(slot, seq, cursor, size, timestamp, _STATUS_READY)
            self._write_u64(_HEADER_WRITE_SEQ_OFF, seq)
            self._write_u64(_HEADER_WRITE_CURSOR_OFF, cursor + size)
        return SharedDataRef(
            key=self.key,
            name=self.name,
            seq=seq,
            slot=slot,
            size=size,
            timestamp=float(timestamp),
            codec=str(codec),
            meta=dict(meta or {}),
            buffer_size=int(self._region.size),
            slot_count=int(self.slot_count),
            store_id=int(self.store_id),
        )

    def read(self, ref: Dict[str, Any]) -> bytes:
        ref_store_id = ref.get("store_id")
        if ref_store_id is not None and int(ref_store_id) != self.store_id:
            raise SharedDataExpired(
                f"payload store was recreated for {ref.get('key')}: "
                f"{int(ref_store_id)} != {self.store_id}"
            )
        slot = int(ref["slot"])
        seq = int(ref["seq"])
        first = self._read_slot_fields(slot)
        first_seq, offset, size, _timestamp, status = first
        if status != _STATUS_READY or first_seq != seq:
            raise SharedDataExpired(f"payload expired for {ref.get('key')}: seq={seq}")
        if size != int(ref.get("size", size)):
            raise SharedDataExpired(f"payload size changed for {ref.get('key')}: seq={seq}")
        if offset < 0 or size < 0 or offset + size > self.data_capacity:
            raise SharedDataExpired(f"payload slot is invalid for {ref.get('key')}: seq={seq}")
        data = self._region.read_bytes(self.data_offset + offset, size)
        second = self._read_slot_fields(slot)
        if second != first:
            raise SharedDataExpired(f"payload changed while reading {ref.get('key')}: seq={seq}")
        return data

    def close(self, *, unlink: bool = False) -> None:
        if self._closed:
            return
        self._closed = True
        if self._region.is_valid():
            shm_close(self._region)
        if unlink or self._owner:
            try:
                shm_unlink(self.name)
            except Exception:
                pass

    @property
    def data_offset(self) -> int:
        return _HEADER_SIZE + self.slot_count * _SLOT_SIZE

    @property
    def data_capacity(self) -> int:
        return int(self._region.size) - self.data_offset

    @property
    def store_id(self) -> int:
        return self._read_u64(_HEADER_STORE_ID_OFF)

    def _init_header(self) -> None:
        self._region.write_bytes(0, b"\x00" * _HEADER_SIZE)
        self._region.write_bytes(
            0,
            _HEADER.pack(
                _MAGIC,
                _VERSION,
                self.slot_count,
                self._region.size,
                self.data_offset,
                0,
                0,
                uuid.uuid4().int & ((1 << 64) - 1),
            ),
        )

    def _validate_header(self) -> None:
        magic = self._read_u64(_HEADER_MAGIC_OFF)
        if magic != _MAGIC:
            raise RuntimeError(f"shared data store {self.name} has invalid magic")
        version = self._region.read_u32(8)
        if version != _VERSION:
            raise RuntimeError(f"shared data store {self.name} version mismatch: {version}")
        self.slot_count = int(self._region.read_u32(_HEADER_SLOT_COUNT_OFF))
        self.buffer_size = int(self._read_u64(_HEADER_TOTAL_SIZE_OFF))

    def _slot_offset(self, slot: int) -> int:
        if slot < 0 or slot >= self.slot_count:
            raise ValueError(f"slot out of range: {slot}")
        return _HEADER_SIZE + slot * _SLOT_SIZE

    def _read_slot(self, slot: int) -> Dict[str, Any]:
        seq, offset, size, timestamp, status = self._read_slot_fields(slot)
        return {
            "seq": int(seq),
            "offset": int(offset),
            "size": int(size),
            "timestamp": float(timestamp),
            "status": int(status),
        }

    def _read_slot_fields(self, slot: int) -> tuple[int, int, int, float, int]:
        return _SLOT.unpack_from(self._region.buffer, self._slot_offset(slot))

    def _write_slot(self, slot: int, seq: int, offset: int, size: int, timestamp: float, status: int) -> None:
        _SLOT.pack_into(
            self._region.buffer,
            self._slot_offset(slot),
            int(seq),
            int(offset),
            int(size),
            float(timestamp),
            int(status),
        )

    def _invalidate_overlaps(self, offset: int, size: int) -> None:
        start = int(offset)
        end = start + int(size)
        for slot in range(self.slot_count):
            _seq, old_start, old_size, old_timestamp, status = self._read_slot_fields(slot)
            if status != _STATUS_READY:
                continue
            old_end = old_start + old_size
            if start < old_end and old_start < end:
                self._write_slot(slot, 0, old_start, old_size, old_timestamp, _STATUS_INVALID)

    def _read_u64(self, offset: int) -> int:
        return self._region.read_u64(offset)

    def _write_u64(self, offset: int, value: int) -> None:
        self._region.write_u64(offset, value)


def shared_data_name(key: str) -> str:
    digest = hashlib.sha1(str(key).encode("utf-8")).hexdigest()[:20]
    return f"rcpd_{digest}"


def _env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        return default
