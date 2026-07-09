# Copyright 2026 RynnRCP Authors. All rights reserved.
# Native abstraction: shared memory operations (cross-platform).

from __future__ import annotations

import hashlib
import logging
import struct
import threading
from dataclasses import dataclass, field
from multiprocessing import shared_memory
from typing import Optional

from rynnrcp.native.config import align_to_page

logger = logging.getLogger(__name__)

_MAX_SHM_NAME_LEN = 30  # macOS POSIX shm limit is 31 bytes including leading slash.
_U32 = struct.Struct("<I")
_U64 = struct.Struct("<Q")
_RESOURCE_TRACKER_PATCH_LOCK = threading.Lock()


def _normalize_shm_name(name: str) -> str:
    """Return a cross-platform-safe shared memory name."""
    clean = str(name).lstrip("/").replace("/", "_")
    encoded = clean.encode("utf-8")
    if len(encoded) <= _MAX_SHM_NAME_LEN:
        return clean

    digest = hashlib.sha1(encoded).hexdigest()[:16]
    prefix_budget = _MAX_SHM_NAME_LEN - len(digest) - 1
    prefix = encoded[:max(0, prefix_budget)].decode("utf-8", errors="ignore")
    return f"{prefix}_{digest}"[:_MAX_SHM_NAME_LEN]


@dataclass
class ShmRegion:
    """Represents a mapped shared memory region."""

    name: str = ""
    size: int = 0
    _shm: Optional[shared_memory.SharedMemory] = field(default=None, repr=False)

    @property
    def ptr(self) -> Optional[memoryview]:
        """Get a memoryview of the shared memory buffer."""
        return self._shm.buf if self._shm is not None else None

    @property
    def buffer(self) -> Optional[memoryview]:
        """Get raw writable buffer access."""
        return self._shm.buf if self._shm is not None else None

    def is_valid(self) -> bool:
        """Check if the region is valid."""
        return self._shm is not None

    def read_view(self, offset: int, length: int) -> memoryview:
        """Return a memoryview over a byte range without copying."""
        self._check_bounds(offset, length)
        return self._shm.buf[offset:offset + length]

    def read_bytes(self, offset: int, length: int) -> bytes:
        """Read bytes from the shared memory at given offset."""
        return bytes(self.read_view(offset, length))

    def write_bytes(self, offset: int, data: bytes | bytearray | memoryview) -> None:
        """Write bytes to the shared memory at given offset."""
        self._check_bounds(offset, len(data))
        self._shm.buf[offset:offset + len(data)] = data

    def read_u32(self, offset: int) -> int:
        """Read a uint32 from shared memory at given offset."""
        self._check_bounds(offset, _U32.size)
        return _U32.unpack_from(self._shm.buf, offset)[0]

    def write_u32(self, offset: int, value: int) -> None:
        """Write a uint32 to shared memory at given offset."""
        self._check_bounds(offset, _U32.size)
        _U32.pack_into(self._shm.buf, offset, int(value) & 0xFFFFFFFF)

    def read_u64(self, offset: int) -> int:
        """Read a uint64 from shared memory at given offset."""
        self._check_bounds(offset, _U64.size)
        return _U64.unpack_from(self._shm.buf, offset)[0]

    def write_u64(self, offset: int, value: int) -> None:
        """Write a uint64 to shared memory at given offset."""
        self._check_bounds(offset, _U64.size)
        _U64.pack_into(self._shm.buf, offset, int(value) & 0xFFFFFFFFFFFFFFFF)

    def _check_bounds(self, offset: int, length: int) -> None:
        if self._shm is None:
            raise RuntimeError("ShmRegion is not valid")
        if offset < 0 or length < 0 or offset + length > self.size:
            raise ValueError(
                f"SHM access out of bounds: offset={offset}, length={length}, size={self.size}"
            )


def shm_create(name: str, size: int, *, zero_init: bool = False, track: bool = True) -> ShmRegion:
    """Create a new shared memory region.

    New SHM objects are normally zero-filled by the OS. ``zero_init=True``
    remains available for tests or platforms where callers want an explicit
    clear, but the default avoids allocating and writing a full-size zero buffer.
    """
    aligned_size = align_to_page(size)
    clean_name = _normalize_shm_name(name)

    try:
        shm = _new_shared_memory(clean_name, create=True, size=aligned_size, track=track)
        if zero_init:
            _zero_memory(shm.buf, aligned_size)
        return ShmRegion(name=clean_name, size=shm.size, _shm=shm)
    except FileExistsError:
        logger.warning("SHM '%s' already exists", name)
        return ShmRegion()
    except Exception as exc:
        logger.error("shm_create failed: %s", exc)
        return ShmRegion()


def shm_open(name: str, size: int, *, track: bool = False) -> ShmRegion:
    """Open an existing shared memory region."""
    aligned_size = align_to_page(size)
    clean_name = _normalize_shm_name(name)

    try:
        shm = _new_shared_memory(clean_name, create=False, track=track)
        if shm.size < aligned_size:
            shm.close()
            logger.error(
                "SHM '%s' size mismatch: actual=%d, expected>=%d",
                name,
                shm.size,
                aligned_size,
            )
            return ShmRegion()
        return ShmRegion(name=clean_name, size=shm.size, _shm=shm)
    except FileNotFoundError:
        logger.error("SHM '%s' not found", name)
        return ShmRegion()
    except Exception as exc:
        logger.error("shm_open failed: %s", exc)
        return ShmRegion()


def shm_close(region: ShmRegion) -> None:
    """Close (unmap) a shared memory region from this process."""
    if region._shm is not None:
        region._shm.close()
        region._shm = None
        region.size = 0


def shm_unlink(name: str) -> None:
    """Remove the shared memory object from the system."""
    clean_name = _normalize_shm_name(name)
    try:
        shm = _new_shared_memory(clean_name, create=False)
        shm.close()
        shm.unlink()
    except FileNotFoundError:
        pass
    except Exception as exc:
        logger.error("shm_unlink failed: %s", exc)


def _new_shared_memory(
    name: str,
    *,
    create: bool,
    size: int = 0,
    track: bool = True,
) -> shared_memory.SharedMemory:
    try:
        shm = shared_memory.SharedMemory(name=name, create=create, size=size, track=track)
    except TypeError:
        if track:
            return shared_memory.SharedMemory(name=name, create=create, size=size)
        return _new_untracked_shared_memory(name=name, create=create, size=size)
    return shm


def _new_untracked_shared_memory(
    *,
    name: str,
    create: bool,
    size: int = 0,
) -> shared_memory.SharedMemory:
    """Create/open SharedMemory without registering it with resource_tracker.

    Python < 3.13 has no ``track`` argument, so creating and then manually
    unregistering can race when multiple threads open the same SHM name. The
    resource tracker keeps a set, not a reference count, so duplicate
    unregister calls can make its process print ``KeyError`` on shutdown.
    Suppressing registration during construction gives the intended
    ``track=False`` behavior without sending any unregister command.
    """
    from multiprocessing import resource_tracker

    with _RESOURCE_TRACKER_PATCH_LOCK:
        original_register = resource_tracker.register

        def register(name: str, rtype: str) -> None:
            if rtype == "shared_memory":
                return
            original_register(name, rtype)

        resource_tracker.register = register
        try:
            return shared_memory.SharedMemory(name=name, create=create, size=size)
        finally:
            resource_tracker.register = original_register


def _zero_memory(buf: memoryview, size: int) -> None:
    chunk = b"\x00" * min(size, 1024 * 1024)
    offset = 0
    while offset < size:
        end = min(size, offset + len(chunk))
        buf[offset:end] = chunk[:end - offset]
        offset = end
