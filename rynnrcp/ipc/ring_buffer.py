# Copyright 2026 RynnRCP Authors. All rights reserved.
# Core layer: SHM-backed Ring Buffer (SPMC - single producer, multiple consumer).

from __future__ import annotations

import logging
import struct
import threading
from typing import Callable, Optional, Sequence

from rynnrcp.native import (
    CACHE_LINE_SIZE,
    shm_close,
    shm_create,
    shm_open,
    shm_unlink,
)
from rynnrcp.native.native_detect import get_native_module, is_native_available

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# SHM Memory Layout
# ---------------------------------------------------------------------------
# Header (CACHE_LINE_SIZE bytes, aligned):
#   offset 0:  write_index  (uint64) - monotonically increasing write counter
#   offset 8:  slot_size    (uint64) - size of each slot in bytes
#   offset 16: slot_count   (uint64) - number of slots (must be power of 2)
#   offset 24: flags        (uint64) - reserved for future use
#   offset 32..CACHE_LINE_SIZE-1: reserved/padding
#
# Data region (immediately after header):
#   Slot 0:  [slot_size bytes]
#   Slot 1:  [slot_size bytes]
#   ...
#   Slot N-1: [slot_size bytes]
# ---------------------------------------------------------------------------

_HEADER_SIZE = CACHE_LINE_SIZE  # 64 bytes, cache-line aligned
_FMT_HEADER = "<QQQQ"  # write_index, slot_size, slot_count, flags
_HEADER_PACK_SIZE = struct.calcsize(_FMT_HEADER)  # 32 bytes

# Magic value to identify initialized buffers
_MAGIC_FLAGS = 0x524F424F_52494E47  # "ROBORING" in hex-ish
_TRANSPORT_ENVELOPE_MAGIC = b"RCT1"
_TRANSPORT_ENVELOPE_HEADER_SIZE = 16


class RingBuffer:
    """SHM-backed ring buffer for zero-copy inter-process communication.

    SPMC model: one writer, multiple readers.
    The writer advances write_index monotonically. Readers track their own
    read position and use `index & (slot_count - 1)` to find the physical slot.

    Args:
        name: Unique SHM name for this ring buffer.
        slot_size: Size of each slot in bytes.
        slot_count: Number of slots (must be power of 2). Default 16.
        create: True to create new SHM, False to open existing.
    """

    def __init__(self, name: str, slot_size: int, slot_count: int = 16,
                 create: bool = True) -> None:
        assert slot_count > 0 and (slot_count & (slot_count - 1)) == 0, \
            f"slot_count must be power of 2, got {slot_count}"
        assert slot_size > 0, f"slot_size must be > 0, got {slot_size}"

        self._name = name
        self._slot_size = slot_size
        self._slot_count = slot_count
        self._slot_mask = slot_count - 1
        self._data_offset = _HEADER_SIZE
        self._total_size = _HEADER_SIZE + slot_size * slot_count
        self._lock = threading.Lock()  # Protects write operations within process
        self._closed = False
        self._native_rb = self._open_native(create)
        if self._native_rb is not None:
            logger.debug("Using native C++ RingBuffer for '%s'", name)
            return

        if create:
            # Clean up any stale SHM
            shm_unlink(name)
            self._region = shm_create(name, self._total_size)
            if not self._region.is_valid():
                raise RuntimeError(f"Failed to create SHM ring buffer '{name}'")
            # Write header
            self._write_header(0, slot_size, slot_count, _MAGIC_FLAGS)
        else:
            self._region = shm_open(name, self._total_size)
            if not self._region.is_valid():
                raise RuntimeError(f"Failed to open SHM ring buffer '{name}'")
            # Verify header
            _, ss, sc, flags = self._read_header()
            if flags != _MAGIC_FLAGS:
                shm_close(self._region)
                raise RuntimeError(f"SHM '{name}' is not a valid RingBuffer (bad magic)")
            if ss != slot_size or sc != slot_count:
                shm_close(self._region)
                raise RuntimeError(
                    f"RingBuffer config mismatch: expected slot_size={slot_size}, "
                    f"slot_count={slot_count}, got {ss}, {sc}")

    def _open_native(self, create: bool):
        if not is_native_available():
            return None
        try:
            native = get_native_module()
            rb_factory = native.RingBuffer.create if create else native.RingBuffer.open
            rb = rb_factory(self._name, self._slot_size, self._slot_count)
            if rb is not None and rb.is_valid():
                return rb
            logger.warning(
                "Native C++ RingBuffer failed to %s '%s'; falling back to Python SHM RingBuffer",
                "create" if create else "open",
                self._name,
            )
        except Exception as exc:
            logger.warning(
                "Native C++ RingBuffer unavailable for '%s'; falling back to Python SHM RingBuffer: %s",
                self._name,
                exc,
            )
        return None

    # ----- Header access -----

    def _write_header(self, write_index: int, slot_size: int,
                      slot_count: int, flags: int) -> None:
        data = struct.pack(_FMT_HEADER, write_index, slot_size, slot_count, flags)
        self._region.write_bytes(0, data)

    def _read_header(self) -> tuple:
        """Returns (write_index, slot_size, slot_count, flags)."""
        raw = self._region.read_bytes(0, _HEADER_PACK_SIZE)
        return struct.unpack(_FMT_HEADER, raw)

    def _get_write_index(self) -> int:
        return self._region.read_u64(0)

    def _set_write_index(self, value: int) -> None:
        self._region.write_u64(0, value)

    # ----- Slot access -----

    def _slot_offset(self, index: int) -> int:
        """Physical offset in SHM for a given logical index."""
        physical = index & self._slot_mask
        return self._data_offset + physical * self._slot_size

    # ----- Public API -----

    def write(self, data: bytes, *, zero_pad: bool = True) -> int:
        """Write data to the next slot in the ring buffer.

        Args:
            data: Bytes to write. Must be <= slot_size.
                  If shorter than slot_size, remaining bytes are zero-padded.

        Returns:
            The logical write index (monotonically increasing).

        Raises:
            RuntimeError: If buffer is closed or data too large.
        """
        return self.write_parts((data,), zero_pad=zero_pad)

    def write_parts(self, parts: Sequence[bytes | bytearray | memoryview], *, zero_pad: bool = True) -> int:
        """Write one logical message from multiple byte chunks.

        This avoids building a temporary ``header + payload`` bytes object for
        SHM envelope writes. ``zero_pad`` remains available for callers that
        intentionally need fixed-width slots; enveloped messages set it to
        False because the envelope carries the true payload length.
        """
        if self._closed:
            raise RuntimeError("RingBuffer is closed")
        total_len = sum(len(part) for part in parts)
        if total_len > self._slot_size:
            raise ValueError(
                f"Data size {total_len} exceeds slot_size {self._slot_size}")
        if self._native_rb is not None:
            index = self._native_rb.write(b"".join(bytes(part) for part in parts))
            if index < 0:
                raise RuntimeError("Native RingBuffer write failed")
            return index
        with self._lock:
            idx = self._get_write_index()
            self._write_parts_at_locked(idx, parts, total_len, zero_pad)
            self._set_write_index(idx + 1)
            return idx

    def write_indexed_parts(
        self,
        build_parts: Callable[[int], Sequence[bytes | bytearray | memoryview]],
        *,
        zero_pad: bool = False,
    ) -> int:
        """Build and write parts after the logical write index is known."""
        if self._closed:
            raise RuntimeError("RingBuffer is closed")
        if self._native_rb is not None:
            raise RuntimeError(
                "write_indexed_parts is not available on native RingBuffer; "
                "use write_envelope_parts for SHM transport messages")
        with self._lock:
            idx = self._get_write_index()
            parts = tuple(build_parts(idx))
            total_len = sum(len(part) for part in parts)
            if total_len > self._slot_size:
                raise ValueError(
                    f"Data size {total_len} exceeds slot_size {self._slot_size}")
            self._write_parts_at_locked(idx, parts, total_len, zero_pad)
            self._set_write_index(idx + 1)
            return idx

    def write_envelope_parts(self, payload_size: int, parts: Sequence[bytes | bytearray | memoryview]) -> int:
        """Write one SHM transport v1 envelope from payload chunks."""
        if self._closed:
            raise RuntimeError("RingBuffer is closed")
        if payload_size > self._slot_size - _TRANSPORT_ENVELOPE_HEADER_SIZE:
            raise ValueError(
                f"Data size {payload_size} exceeds payload capacity "
                f"{self._slot_size - _TRANSPORT_ENVELOPE_HEADER_SIZE}")
        if self._native_rb is not None:
            index = self._native_rb.write_envelope_parts(
                int(payload_size),
                tuple(parts),
            )
            if index < 0:
                raise RuntimeError("Native RingBuffer envelope write failed")
            return index
        return self.write_indexed_parts(
            lambda index: (_encode_transport_envelope_header(payload_size, index), *parts),
            zero_pad=False,
        )

    def _write_parts_at_locked(
        self,
        index: int,
        parts: Sequence[bytes | bytearray | memoryview],
        total_len: int,
        zero_pad: bool,
    ) -> None:
        offset = self._slot_offset(index)
        cursor = offset
        for part in parts:
            if part:
                self._region.write_bytes(cursor, part)
                cursor += len(part)
        if zero_pad and total_len < self._slot_size:
            self._region.write_bytes(cursor, b"\x00" * (self._slot_size - total_len))

    def read(self, index: int) -> bytes:
        """Read data from a specific slot.

        Args:
            index: Logical index to read. The physical slot is index % slot_count.

        Returns:
            Copy of the slot data as bytes.

        Raises:
            RuntimeError: If buffer is closed.
            IndexError: If index is too old (overwritten) or not yet written.
        """
        if self._closed:
            raise RuntimeError("RingBuffer is closed")
        if self._native_rb is not None:
            data = self._native_rb.read(index)
            if data is None:
                raise IndexError(f"Index {index} unavailable")
            return data

        current_write = self._get_write_index()
        if index >= current_write:
            raise IndexError(
                f"Index {index} not yet written (current write_index={current_write})")
        if current_write - index > self._slot_count:
            raise IndexError(
                f"Index {index} has been overwritten "
                f"(oldest available={current_write - self._slot_count})")

        offset = self._slot_offset(index)
        return self._region.read_bytes(offset, self._slot_size)

    def read_at(self, index: int, offset_in_slot: int, length: int) -> bytes:
        """Read a bounded byte range from a specific logical slot."""
        if self._closed:
            raise RuntimeError("RingBuffer is closed")
        if offset_in_slot < 0 or length < 0 or offset_in_slot + length > self._slot_size:
            raise ValueError("read_at range exceeds slot bounds")
        if self._native_rb is not None:
            data = self._native_rb.read_at(index, offset_in_slot, length)
            if data is None:
                raise IndexError(f"Index {index} unavailable")
            return data

        self._validate_read_index(index)
        offset = self._slot_offset(index) + offset_in_slot
        data = self._region.read_bytes(offset, length)
        self._validate_read_index(index)
        return data

    def read_exact(self, index: int, length: int) -> bytes:
        """Read the first ``length`` bytes from a logical slot."""
        return self.read_at(index, 0, length)

    def _validate_read_index(self, index: int) -> None:
        current_write = self._get_write_index()
        if index >= current_write:
            raise IndexError(
                f"Index {index} not yet written (current write_index={current_write})")
        if current_write - index > self._slot_count:
            raise IndexError(
                f"Index {index} has been overwritten "
                f"(oldest available={current_write - self._slot_count})")

    def try_read(self, index: int) -> Optional[bytes]:
        """Read data from a slot, returning None if unavailable."""
        try:
            return self.read(index)
        except (IndexError, RuntimeError):
            return None

    def latest_index(self) -> int:
        """Get the index of the most recently written slot.

        Returns:
            The latest write index - 1, or -1 if nothing has been written.
        """
        if self._native_rb is not None:
            return int(self._native_rb.latest_index())
        wi = self._get_write_index()
        return wi - 1 if wi > 0 else -1

    def oldest_index(self) -> int:
        """Get the oldest logical index still available, or -1 if empty."""
        if self._native_rb is not None:
            return int(self._native_rb.oldest_index())
        wi = self._get_write_index()
        if wi <= 0:
            return -1
        return max(0, wi - self._slot_count)

    def write_count(self) -> int:
        """Get the total number of writes performed."""
        if self._native_rb is not None:
            return int(self._native_rb.write_count())
        return self._get_write_index()

    @property
    def name(self) -> str:
        return self._name

    @property
    def slot_size(self) -> int:
        return self._slot_size

    @property
    def slot_count(self) -> int:
        return self._slot_count

    def is_valid(self) -> bool:
        if self._native_rb is not None:
            return bool(self._native_rb.is_valid())
        return not self._closed and self._region.is_valid()

    def close(self) -> None:
        """Close the ring buffer (unmap SHM). Does not remove SHM."""
        if not self._closed:
            self._closed = True
            if self._native_rb is not None:
                self._native_rb.close()
            else:
                shm_close(self._region)

    def unlink(self) -> None:
        """Remove the SHM from the system."""
        if self._native_rb is not None:
            self._native_rb.unlink()
        else:
            shm_unlink(self._name)

    def __del__(self) -> None:
        if not self._closed:
            self.close()


def _encode_transport_envelope_header(size: int, index: int) -> bytes:
    return (
        _TRANSPORT_ENVELOPE_MAGIC
        + int(size).to_bytes(4, "little", signed=False)
        + int(index).to_bytes(8, "little", signed=False)
    )
