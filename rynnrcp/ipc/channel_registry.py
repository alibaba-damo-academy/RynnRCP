"""
SharedChannelRegistry – SHM-based channel registry for cross-process discovery.

The registry is lifecycle metadata, not the data path. It records channel SHM
names and subscriber notifier names so publishers can wake cross-process
subscribers without busy polling.
"""

from __future__ import annotations

import os
import struct
import tempfile
from dataclasses import dataclass
from typing import List, Optional

from rynnrcp.native import shm_close, shm_create, shm_open, shm_unlink


_MAGIC = 0x52434852  # "RCHR"
_VERSION = 2
_HEADER_SIZE = 32
_ENTRY_SIZE = 256
_MAX_ENTRIES = 255
_SUB_ENTRY_SIZE = 256
_MAX_SUBSCRIBERS = 512
_CHANNELS_OFFSET = _HEADER_SIZE
_SUBSCRIBERS_OFFSET = _CHANNELS_OFFSET + _ENTRY_SIZE * _MAX_ENTRIES
_SHM_SIZE = _SUBSCRIBERS_OFFSET + _SUB_ENTRY_SIZE * _MAX_SUBSCRIBERS

_HDR_PACK = struct.Struct("<IIII")
_SUBSCRIBER_VERSION_OFF = 20

_E_NAME_OFF = 0
_E_NAME_LEN = 128
_E_MSG_SIZE_OFF = 128
_E_PID_OFF = 136
_E_TRANSPORT_OFF = 144
_E_SHM_NAME_OFF = 148
_E_SHM_NAME_LEN = 64

_S_CHANNEL_OFF = 0
_S_CHANNEL_LEN = 128
_S_ID_OFF = 128
_S_ID_LEN = 64
_S_PID_OFF = 192
_S_NOTIFIER_OFF = 200
_S_NOTIFIER_LEN = 56


@dataclass
class ChannelEntry:
    """A single channel registration record."""

    name: str
    msg_size: int
    owner_pid: int
    transport: int
    shm_name: str


@dataclass
class SubscriberEntry:
    """A subscriber wakeup registration record."""

    channel_name: str
    subscriber_id: str
    pid: int
    notifier_name: str


class _RegistryFileLock:
    """Cross-process registry lock backed by the OS file-lock API."""

    def __init__(self, registry_name: str) -> None:
        safe_name = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in registry_name)
        self._path = os.path.join(tempfile.gettempdir(), f"{safe_name}.lock")
        self._fh = None

    def acquire(self) -> None:
        self._fh = open(self._path, "a+b")
        if os.name == "nt":
            import msvcrt

            self._fh.seek(0)
            msvcrt.locking(self._fh.fileno(), msvcrt.LK_LOCK, 1)
        else:
            import fcntl

            fcntl.flock(self._fh.fileno(), fcntl.LOCK_EX)

    def release(self) -> None:
        if self._fh is None:
            return
        if os.name == "nt":
            import msvcrt

            self._fh.seek(0)
            msvcrt.locking(self._fh.fileno(), msvcrt.LK_UNLCK, 1)
        else:
            import fcntl

            fcntl.flock(self._fh.fileno(), fcntl.LOCK_UN)
        self._fh.close()
        self._fh = None


class SharedChannelRegistry:
    """SHM-based channel registry shared across processes."""

    SHM_NAME = "rynnrcp_channel_registry"

    def __init__(self, create: bool = True, name: str | None = None) -> None:
        self.shm_name = str(name or self.SHM_NAME)
        self._owner = bool(create)
        self._active_lock: _RegistryFileLock | None = None
        if create:
            try:
                shm_unlink(self.shm_name)
            except Exception:
                pass
            self._region = shm_create(self.shm_name, _SHM_SIZE)
            if not self._region.is_valid():
                raise RuntimeError("Failed to create channel registry SHM")
            self._region.write_bytes(0, _HDR_PACK.pack(_MAGIC, _VERSION, 0, 0))
            self._write_u32(16, 0)
            self._write_u32(_SUBSCRIBER_VERSION_OFF, 0)
        else:
            self._region = shm_open(self.shm_name, _SHM_SIZE)
            if not self._region.is_valid():
                raise RuntimeError("Failed to open channel registry SHM")
            magic = self._read_u32(0)
            if magic != _MAGIC:
                raise RuntimeError(
                    f"Registry SHM magic mismatch: 0x{magic:08X} != 0x{_MAGIC:08X}"
                )
            version = self._read_u32(4)
            if version != _VERSION:
                raise RuntimeError(
                    f"Registry SHM version mismatch: {version} != {_VERSION}"
                )

    def _read_u32(self, offset: int) -> int:
        return int.from_bytes(self._region.read_bytes(offset, 4), "little", signed=False)

    def _write_u32(self, offset: int, value: int) -> None:
        self._region.write_bytes(offset, value.to_bytes(4, "little", signed=False))

    def _read_u64(self, offset: int) -> int:
        return int.from_bytes(self._region.read_bytes(offset, 8), "little", signed=False)

    def _write_u64(self, offset: int, value: int) -> None:
        self._region.write_bytes(offset, value.to_bytes(8, "little", signed=False))

    def _acquire_lock(self) -> None:
        lock = _RegistryFileLock(self.shm_name)
        lock.acquire()
        self._active_lock = lock

    def _release_lock(self) -> None:
        if self._active_lock is not None:
            self._active_lock.release()
            self._active_lock = None

    def _entry_offset(self, idx: int) -> int:
        return _CHANNELS_OFFSET + idx * _ENTRY_SIZE

    def _subscriber_offset(self, idx: int) -> int:
        return _SUBSCRIBERS_OFFSET + idx * _SUB_ENTRY_SIZE

    def _read_entry(self, idx: int) -> Optional[ChannelEntry]:
        base = self._entry_offset(idx)
        name = self._read_str(base + _E_NAME_OFF, _E_NAME_LEN)
        if not name:
            return None
        return ChannelEntry(
            name=name,
            msg_size=self._read_u64(base + _E_MSG_SIZE_OFF),
            owner_pid=self._read_u64(base + _E_PID_OFF),
            transport=self._read_u32(base + _E_TRANSPORT_OFF),
            shm_name=self._read_str(base + _E_SHM_NAME_OFF, _E_SHM_NAME_LEN),
        )

    def _write_entry(self, idx: int, entry: ChannelEntry) -> None:
        base = self._entry_offset(idx)
        self._write_str(base + _E_NAME_OFF, _E_NAME_LEN, entry.name)
        self._write_u64(base + _E_MSG_SIZE_OFF, entry.msg_size)
        self._write_u64(base + _E_PID_OFF, entry.owner_pid)
        self._write_u32(base + _E_TRANSPORT_OFF, entry.transport)
        self._write_str(base + _E_SHM_NAME_OFF, _E_SHM_NAME_LEN, entry.shm_name)

    def _clear_entry(self, idx: int) -> None:
        self._region.write_bytes(self._entry_offset(idx), b"\x00" * _ENTRY_SIZE)

    def _read_subscriber(self, idx: int) -> Optional[SubscriberEntry]:
        base = self._subscriber_offset(idx)
        channel_name = self._read_str(base + _S_CHANNEL_OFF, _S_CHANNEL_LEN)
        if not channel_name:
            return None
        return SubscriberEntry(
            channel_name=channel_name,
            subscriber_id=self._read_str(base + _S_ID_OFF, _S_ID_LEN),
            pid=self._read_u64(base + _S_PID_OFF),
            notifier_name=self._read_str(base + _S_NOTIFIER_OFF, _S_NOTIFIER_LEN),
        )

    def _write_subscriber(self, idx: int, entry: SubscriberEntry) -> None:
        base = self._subscriber_offset(idx)
        self._write_str(base + _S_CHANNEL_OFF, _S_CHANNEL_LEN, entry.channel_name)
        self._write_str(base + _S_ID_OFF, _S_ID_LEN, entry.subscriber_id)
        self._write_u64(base + _S_PID_OFF, entry.pid)
        self._write_str(base + _S_NOTIFIER_OFF, _S_NOTIFIER_LEN, entry.notifier_name)

    def _clear_subscriber(self, idx: int) -> None:
        self._region.write_bytes(self._subscriber_offset(idx), b"\x00" * _SUB_ENTRY_SIZE)

    def _read_str(self, offset: int, length: int) -> str:
        raw = self._region.read_bytes(offset, length)
        return raw.split(b"\x00", 1)[0].decode("utf-8", errors="replace")

    def _write_str(self, offset: int, length: int, value: str) -> None:
        data = value.encode("utf-8")[:length].ljust(length, b"\x00")
        self._region.write_bytes(offset, data)

    def _get_count(self) -> int:
        return self._read_u32(8)

    def _set_count(self, n: int) -> None:
        self._write_u32(8, n)

    def _get_subscriber_count(self) -> int:
        return self._read_u32(16)

    def _set_subscriber_count(self, n: int) -> None:
        self._write_u32(16, n)

    def subscriber_version(self) -> int:
        """Return a monotonic version bumped when subscriber registrations change."""
        return self._read_u32(_SUBSCRIBER_VERSION_OFF)

    def _bump_subscriber_version(self) -> None:
        self._write_u32(_SUBSCRIBER_VERSION_OFF, (self.subscriber_version() + 1) & 0xFFFFFFFF)

    def register(self, name: str, msg_size: int, pid: int, shm_name: str, transport: int = 2) -> None:
        """Register a channel. Overwrites if the channel name already exists."""
        self._acquire_lock()
        try:
            count = self._get_count()
            for i in range(count):
                entry = self._read_entry(i)
                if entry and entry.name == name:
                    self._write_entry(i, ChannelEntry(name, msg_size, pid, transport, shm_name))
                    return
            if count >= _MAX_ENTRIES:
                raise RuntimeError("Channel registry is full")
            self._write_entry(count, ChannelEntry(name, msg_size, pid, transport, shm_name))
            self._set_count(count + 1)
        finally:
            self._release_lock()

    def lookup(self, name: str) -> Optional[ChannelEntry]:
        """Look up a channel by name."""
        self._acquire_lock()
        try:
            count = self._get_count()
            for i in range(count):
                entry = self._read_entry(i)
                if entry and entry.name == name:
                    return entry
            return None
        finally:
            self._release_lock()

    def unregister(self, name: str, pid: int) -> bool:
        """Remove a channel entry by name and owner pid."""
        self._acquire_lock()
        try:
            count = self._get_count()
            for i in range(count):
                entry = self._read_entry(i)
                if entry and entry.name == name and entry.owner_pid == pid:
                    if i < count - 1:
                        last = self._read_entry(count - 1)
                        if last:
                            self._write_entry(i, last)
                    self._clear_entry(count - 1)
                    self._set_count(count - 1)
                    return True
            return False
        finally:
            self._release_lock()

    def list_all(self) -> List[ChannelEntry]:
        """List all registered channels."""
        self._acquire_lock()
        try:
            entries: List[ChannelEntry] = []
            for i in range(self._get_count()):
                entry = self._read_entry(i)
                if entry:
                    entries.append(entry)
            return entries
        finally:
            self._release_lock()

    def register_subscriber(self, channel_name: str, subscriber_id: str, pid: int, notifier_name: str) -> None:
        """Register a subscriber notifier for publisher wakeups."""
        self._acquire_lock()
        try:
            count = self._get_subscriber_count()
            for i in range(count):
                entry = self._read_subscriber(i)
                if entry and entry.channel_name == channel_name and entry.subscriber_id == subscriber_id:
                    self._write_subscriber(i, SubscriberEntry(channel_name, subscriber_id, pid, notifier_name))
                    self._bump_subscriber_version()
                    return
            if count >= _MAX_SUBSCRIBERS:
                raise RuntimeError("Channel subscriber registry is full")
            self._write_subscriber(count, SubscriberEntry(channel_name, subscriber_id, pid, notifier_name))
            self._set_subscriber_count(count + 1)
            self._bump_subscriber_version()
        finally:
            self._release_lock()

    def unregister_subscriber(self, channel_name: str, subscriber_id: str, pid: int) -> bool:
        """Remove a subscriber notifier entry."""
        self._acquire_lock()
        try:
            count = self._get_subscriber_count()
            for i in range(count):
                entry = self._read_subscriber(i)
                if (
                    entry
                    and entry.channel_name == channel_name
                    and entry.subscriber_id == subscriber_id
                    and entry.pid == pid
                ):
                    if i < count - 1:
                        last = self._read_subscriber(count - 1)
                        if last:
                            self._write_subscriber(i, last)
                    self._clear_subscriber(count - 1)
                    self._set_subscriber_count(count - 1)
                    self._bump_subscriber_version()
                    return True
            return False
        finally:
            self._release_lock()

    def list_subscribers(self, channel_name: str) -> List[SubscriberEntry]:
        """List subscriber notifier entries for a channel."""
        self._acquire_lock()
        try:
            entries: List[SubscriberEntry] = []
            for i in range(self._get_subscriber_count()):
                entry = self._read_subscriber(i)
                if entry and entry.channel_name == channel_name:
                    entries.append(entry)
            return entries
        finally:
            self._release_lock()

    def close(self) -> None:
        """Close the registry SHM; only the creator unlinks it."""
        if self._region.is_valid():
            shm_close(self._region)
        if self._owner:
            try:
                shm_unlink(self.shm_name)
            except Exception:
                pass
