# Copyright 2026 RynnRCP Authors. All rights reserved.
# Core layer: Transport abstraction with multiple levels.
# L0+L1 (IntraProcess): direct reference passing within same process.
# L2 (SHM): cross-process via SHM Ring Buffer.

from __future__ import annotations

import collections
import hashlib
import logging
import os
import threading
import time
import uuid
from abc import ABC, abstractmethod
from enum import IntEnum
from typing import Any, Callable, Dict, List, Optional, Sequence

from rynnrcp.native import (
    Event,
    NotifierUnavailable,
    WaitResult,
    create_notifier,
    open_notifier,
    open_or_create_notifier,
)
from rynnrcp.ipc.ring_buffer import RingBuffer

logger = logging.getLogger(__name__)
_SHM_ENVELOPE_MAGIC = b"RCT1"
_SHM_ENVELOPE_HEADER_SIZE = 16
_DEFAULT_SHM_POLL_INTERVAL_US = 1000


class TransportLevel(IntEnum):
    """Transport routing levels."""
    L0_INTRA_THREAD = 0    # Same thread: direct callback
    L1_INTRA_PROCESS = 1   # Same process: reference + Event notification
    L2_SHM = 2             # Cross-process: SHM Ring Buffer


def parse_transport_level(value: TransportLevel | str | int | None) -> TransportLevel | None:
    """Parse a user/config transport value into a TransportLevel.

    ``None`` means auto-detect at the ChannelManager layer. Explicit "shm" is
    required to force cross-process shared memory.
    """
    if value is None:
        return None
    if isinstance(value, TransportLevel):
        return value
    if isinstance(value, int):
        return TransportLevel(value)

    normalized = str(value).strip().lower().replace("-", "_")
    aliases = {
        "l0": TransportLevel.L0_INTRA_THREAD,
        "l0_intra_thread": TransportLevel.L0_INTRA_THREAD,
        "intra_thread": TransportLevel.L0_INTRA_THREAD,
        "l1": TransportLevel.L1_INTRA_PROCESS,
        "l1_intra_process": TransportLevel.L1_INTRA_PROCESS,
        "intra_process": TransportLevel.L1_INTRA_PROCESS,
        "in_process": TransportLevel.L1_INTRA_PROCESS,
        "memory": TransportLevel.L1_INTRA_PROCESS,
        "l2": TransportLevel.L2_SHM,
        "l2_shm": TransportLevel.L2_SHM,
        "shm": TransportLevel.L2_SHM,
        "shared_memory": TransportLevel.L2_SHM,
        "auto": None,
        "default": None,
    }
    try:
        return aliases[normalized]
    except KeyError as exc:
        raise ValueError(f"Unknown transport level: {value!r}") from exc


class TransportBase(ABC):
    """Abstract base class for all transport implementations."""

    @abstractmethod
    def publish(self, data: bytes) -> None:
        """Publish data to all subscribers."""
        ...

    def publish_parts(self, parts: Sequence[bytes | bytearray | memoryview]) -> None:
        """Publish one logical message from byte chunks."""
        self.publish(b"".join(bytes(part) for part in parts))

    @abstractmethod
    def subscribe(self, callback: Callable[[bytes], None]) -> None:
        """Register a subscriber callback."""
        ...

    @abstractmethod
    def close(self) -> None:
        """Release resources."""
        ...

    @property
    @abstractmethod
    def level(self) -> TransportLevel:
        ...


class IntraProcessTransport(TransportBase):
    """L0+L1 transport: in-process message passing.

    Uses a bounded deque as buffer and threading.Event for notification.
    All subscribers share the same deque and are notified on publish.
    Subscribers run their callbacks in the publisher's thread context
    (synchronous) for L0, or can be polled for L1.
    """

    def __init__(self, name: str, buffer_size: int = 64) -> None:
        self._name = name
        self._buffer: collections.deque = collections.deque(maxlen=buffer_size)
        self._lock = threading.Lock()
        self._event = Event()
        self._callbacks: List[Callable[[bytes], None]] = []
        self._closed = False
        self._next_index = 0
        self._last_read_index = -1

    @property
    def level(self) -> TransportLevel:
        return TransportLevel.L1_INTRA_PROCESS

    def publish(self, data: bytes) -> None:
        """Publish data: store in buffer and notify all subscribers."""
        if self._closed:
            return
        with self._lock:
            self._buffer.append((self._next_index, data))
            self._next_index += 1
        # Notify waiting subscribers
        self._event.signal()
        # Also invoke direct callbacks (L0 style)
        for cb in self._callbacks:
            try:
                cb(data)
            except Exception as e:
                logger.error("Callback error: %s", e)

    def subscribe(self, callback: Callable[[bytes], None]) -> None:
        """Register a callback invoked on each publish."""
        self._callbacks.append(callback)

    def poll(self, timeout_ms: int = 100) -> Optional[bytes]:
        """Poll for the next message (blocking with timeout).

        Returns:
            The oldest unread message, or None on timeout.
        """
        self._last_read_index, data = self.poll_from(self._last_read_index, timeout_ms)
        return data

    def poll_from(self, last_read_index: int, timeout_ms: int = 100) -> tuple[int, Optional[bytes]]:
        """Poll from a reader-specific cursor."""
        deadline = time.monotonic() + timeout_ms / 1000.0
        while True:
            with self._lock:
                for index, data in self._buffer:
                    if index > last_read_index:
                        return index, data
            if time.monotonic() >= deadline:
                return last_read_index, None
            self._event.wait(timeout_ms=min(10, max(0, int((deadline - time.monotonic()) * 1000))))

    def latest(self) -> Optional[bytes]:
        """Get the latest message without blocking (non-consuming peek)."""
        _index, data = self.read_latest_from(self._last_read_index)
        return data

    def read_latest_from(self, last_read_index: int) -> tuple[int, Optional[bytes]]:
        """Get the latest message and advance a reader-specific cursor."""
        with self._lock:
            if self._buffer:
                index, data = self._buffer[-1]
                if index > last_read_index:
                    return index, data
        return last_read_index, None

    @property
    def pending_count(self) -> int:
        """Number of messages in the buffer."""
        with self._lock:
            return len(self._buffer)

    def close(self) -> None:
        self._closed = True
        self._event.close()
        self._callbacks.clear()


class ShmTransport(TransportBase):
    """L2 transport: cross-process communication via SHM Ring Buffer.

    Publisher writes to a shared ring buffer.
    Subscriber polls the ring buffer for new data.
    """

    def __init__(
        self,
        name: str,
        msg_size: int,
        slot_count: int = 16,
        create: bool = True,
        poll_interval_us: int | None = None,
        notify_mode: str = "hybrid",
        registry: Any = None,
    ) -> None:
        self._name = name
        self._payload_size = msg_size
        self._owns_ring = bool(create)
        self._registry = registry
        self._subscriber_ids: List[str] = []
        self._subscriber_notifier = None
        self._subscriber_notifier_name: str | None = None
        self._publisher_notifiers: Dict[str, Any] = {}
        self._publisher_notifier_failures: set[str] = set()
        self._publisher_notifier_refresh_at = 0.0
        self._publisher_notifier_registry_version: int | None = None
        self._notify_mode = str(notify_mode or "hybrid").lower()
        ring_name = f"rc_transport_{name}"
        self._ring = RingBuffer(
            name=ring_name,
            slot_size=msg_size + _SHM_ENVELOPE_HEADER_SIZE,
            slot_count=slot_count,
            create=create,
        )
        self._callbacks: List[Callable[[bytes], None]] = []
        self._event = Event()
        self._closed = False
        self._io_lock = threading.RLock()
        self._last_read_index = -1
        self._poll_interval = _resolve_poll_interval_us(poll_interval_us) / 1_000_000.0
        self._channel_notifier = self._open_channel_notifier(create=create)

    @property
    def level(self) -> TransportLevel:
        return TransportLevel.L2_SHM

    def publish(self, data: bytes) -> None:
        """Write data to the SHM ring buffer and notify."""
        self.publish_parts((data,))

    def publish_parts(self, parts: Sequence[bytes | bytearray | memoryview]) -> None:
        """Write byte chunks to the SHM ring buffer without joining first."""
        with self._io_lock:
            if self._closed:
                return
            total_len = sum(len(part) for part in parts)
            if total_len > self._payload_size:
                raise ValueError(f"Data size {total_len} exceeds msg_size {self._payload_size}")
            self._ring.write_envelope_parts(total_len, parts)
            self._event.signal()
            self._notify_waiters()
            # Also invoke local callbacks if any
            if self._callbacks:
                data = b"".join(bytes(part) for part in parts)
                for cb in self._callbacks:
                    try:
                        cb(data)
                    except Exception as e:
                        logger.error("Callback error: %s", e)

    def subscribe(self, callback: Callable[[bytes], None]) -> None:
        """Register a local callback for this process."""
        self._callbacks.append(callback)

    def register_subscriber(self) -> None:
        """Register this transport instance as a polling subscriber."""
        if self._closed or self._registry is None or self._notify_mode == "poll":
            return
        if self._subscriber_ids:
            return
        subscriber_id = f"{os.getpid()}-{uuid.uuid4().hex[:12]}"
        notifier_name = _subscriber_notifier_name(self._name, subscriber_id)
        try:
            notifier = create_notifier(notifier_name)
        except NotifierUnavailable as exc:
            logger.warning(
                "IPC notifier unavailable for subscriber on %s; falling back to hybrid polling: %s",
                self._name,
                exc,
            )
            return
        self._registry.register_subscriber(self._name, subscriber_id, os.getpid(), notifier_name)
        self._subscriber_ids.append(subscriber_id)
        self._subscriber_notifier = notifier
        self._subscriber_notifier_name = notifier_name

    def poll(self, timeout_ms: int = 100) -> Optional[bytes]:
        """Poll for new data from the ring buffer.

        Uses a hybrid strategy: first checks for an intra-process Event
        signal (zero latency for same-process publisher), then waits on the
        cross-process notifier when available. If the notifier backend is
        unavailable, it falls back to polling the SHM write index with a short
        sleep interval.

        Returns:
            The next unread message, or None on timeout.
        """
        self._last_read_index, data = self.poll_from(self._last_read_index, timeout_ms)
        return data

    def poll_from(self, last_read_index: int, timeout_ms: int = 100) -> tuple[int, Optional[bytes]]:
        """Poll for new data using a reader-specific cursor."""
        with self._io_lock:
            if self._closed:
                return last_read_index, None

            deadline = None if timeout_ms < 0 else time.monotonic() + timeout_ms / 1000.0

            while True:
                # Check ring buffer for new data
                next_idx, data = self._read_next_available(last_read_index)
                if data is not None:
                    return next_idx, data

                # Try intra-process Event (zero-cost non-blocking check)
                if self._event.wait(timeout_ms=0) == WaitResult.SIGNALED:
                    continue  # Event fired -> re-check ring buffer immediately

                # Deadline check
                if timeout_ms == 0 or (deadline is not None and time.monotonic() >= deadline):
                    return last_read_index, None

                self._wait_for_cross_process_signal(deadline)

    def read_latest(self) -> Optional[bytes]:
        """Read the most recent message, skipping any intermediate ones."""
        self._last_read_index, data = self.read_latest_from(self._last_read_index)
        return data

    def read_latest_from(self, last_read_index: int) -> tuple[int, Optional[bytes]]:
        """Read the most recent message with a reader-specific cursor."""
        with self._io_lock:
            if self._closed:
                return last_read_index, None
            latest = self._ring.latest_index()
            if latest < 0:
                return last_read_index, None
            if latest <= last_read_index:
                return last_read_index, None
            data = self._read_index(latest)
            if data is not None:
                return latest, data
            return last_read_index, None

    def close(self) -> None:
        with self._io_lock:
            if self._closed:
                return
            self._closed = True
            self._event.close()
            self._ring.close()
            if self._registry is not None:
                for subscriber_id in list(self._subscriber_ids):
                    try:
                        self._registry.unregister_subscriber(self._name, subscriber_id, os.getpid())
                    except Exception:
                        pass
            if self._subscriber_notifier is not None:
                self._subscriber_notifier.close()
                self._subscriber_notifier.unlink()
                self._subscriber_notifier = None
            if self._channel_notifier is not None:
                self._channel_notifier.close()
                if self._owns_ring:
                    self._channel_notifier.unlink()
                self._channel_notifier = None
            for notifier in self._publisher_notifiers.values():
                try:
                    notifier.close()
                except Exception:
                    pass
            self._publisher_notifiers.clear()
            self._callbacks.clear()

    def unlink(self) -> None:
        """Remove the SHM ring buffer from the system."""
        with self._io_lock:
            if self._owns_ring:
                self._ring.unlink()

    def _open_channel_notifier(self, *, create: bool):
        if self._notify_mode == "poll":
            return None
        name = f"rc_notify_{self._name}"
        try:
            return create_notifier(name) if create else open_or_create_notifier(name)
        except NotifierUnavailable as exc:
            logger.warning(
                "IPC notifier unavailable for channel %s; falling back to hybrid polling: %s",
                self._name,
                exc,
            )
            return None

    def _notify_waiters(self) -> None:
        if self._notify_mode == "poll":
            return
        if self._channel_notifier is not None:
            self._channel_notifier.notify()
        if self._registry is None:
            return
        now = time.monotonic()
        registry_version = self._subscriber_registry_version()
        if registry_version != self._publisher_notifier_registry_version or now >= self._publisher_notifier_refresh_at:
            self._refresh_publisher_notifiers()
            self._publisher_notifier_registry_version = registry_version
            self._publisher_notifier_refresh_at = now + 60.0
        for notifier in list(self._publisher_notifiers.values()):
            try:
                notifier.notify()
            except Exception:
                pass

    def _subscriber_registry_version(self) -> int | None:
        if self._registry is None or not hasattr(self._registry, "subscriber_version"):
            return None
        try:
            return int(self._registry.subscriber_version())
        except Exception:
            return None

    def _refresh_publisher_notifiers(self) -> None:
        if self._registry is None:
            return
        try:
            entries = self._registry.list_subscribers(self._name)
        except Exception as exc:
            logger.debug("Failed to list subscribers for %s: %s", self._name, exc)
            return
        active = {entry.subscriber_id: entry for entry in entries}
        for subscriber_id in list(self._publisher_notifiers):
            if subscriber_id not in active:
                self._publisher_notifiers.pop(subscriber_id).close()
                self._publisher_notifier_failures.discard(subscriber_id)
        for subscriber_id, entry in active.items():
            if subscriber_id in self._publisher_notifiers:
                continue
            try:
                self._publisher_notifiers[subscriber_id] = open_notifier(entry.notifier_name)
                self._publisher_notifier_failures.discard(subscriber_id)
            except NotifierUnavailable as exc:
                first_failure = subscriber_id not in self._publisher_notifier_failures
                self._publisher_notifier_failures.add(subscriber_id)
                log = logger.warning if first_failure else logger.debug
                log(
                    "IPC notifier unavailable for subscriber %s on %s; pruning notifier and using polling: %s",
                    subscriber_id,
                    self._name,
                    exc,
                )
                try:
                    self._registry.unregister_subscriber(self._name, subscriber_id, entry.pid)
                except Exception as cleanup_exc:
                    logger.debug(
                        "Failed to unregister stale subscriber %s on %s: %s",
                        subscriber_id,
                        self._name,
                        cleanup_exc,
                    )
                continue

    def _wait_for_cross_process_signal(self, deadline: float | None) -> None:
        if self._notify_mode != "poll":
            notifier = self._subscriber_notifier or self._channel_notifier
            if notifier is not None:
                wait_ms = _remaining_wait_ms(deadline)
                result = notifier.wait(timeout_ms=wait_ms)
                if result == WaitResult.SIGNALED:
                    notifier.drain()
                    return
                if result == WaitResult.TIMEOUT:
                    return
                logger.debug("IPC notifier wait failed for %s; falling back to polling", self._name)
        if deadline is None:
            time.sleep(self._poll_interval)
            return
        sleep_s = min(self._poll_interval, max(0.0, deadline - time.monotonic()))
        if sleep_s > 0:
            time.sleep(sleep_s)

    def _read_next_available(self, last_read_index: int) -> tuple[int, Optional[bytes]]:
        latest = self._ring.latest_index()
        if latest < 0:
            return last_read_index, None
        next_idx = last_read_index + 1
        oldest = self._ring.oldest_index()
        if oldest >= 0 and next_idx < oldest:
            next_idx = oldest
        if latest < next_idx:
            return last_read_index, None
        data = self._read_index(next_idx)
        if data is not None:
            return next_idx, data
        return last_read_index, None

    def _read_index(self, index: int) -> Optional[bytes]:
        return _read_v1_payload(self._ring, index)


def _read_v1_payload(ring: RingBuffer, index: int) -> Optional[bytes]:
    try:
        header = ring.read_exact(index, _SHM_ENVELOPE_HEADER_SIZE)
        size, stored_index = _decode_v1_header(header)
        if stored_index != index:
            return None
        if size > ring.slot_size - _SHM_ENVELOPE_HEADER_SIZE:
            return None
        payload = ring.read_at(index, _SHM_ENVELOPE_HEADER_SIZE, size)
        header_after = ring.read_exact(index, _SHM_ENVELOPE_HEADER_SIZE)
        if header_after != header:
            return None
        return payload
    except (IndexError, RuntimeError, ValueError):
        return None


def _decode_v1_header(header: bytes) -> tuple[int, int]:
    if len(header) < _SHM_ENVELOPE_HEADER_SIZE or header[:4] != _SHM_ENVELOPE_MAGIC:
        raise ValueError("invalid SHM envelope header")
    size = int.from_bytes(header[4:8], "little", signed=False)
    index = int.from_bytes(header[8:16], "little", signed=False)
    return size, index


def _remaining_wait_ms(deadline: float | None, cap_ms: int | None = None) -> int:
    if deadline is None:
        return -1 if cap_ms is None else cap_ms
    remaining_ms = int(max(0.0, (deadline - time.monotonic()) * 1000.0))
    if cap_ms is None:
        return remaining_ms
    return max(0, min(cap_ms, remaining_ms))


def _resolve_poll_interval_us(value: int | None) -> int:
    if value is not None:
        return max(1, int(value))
    raw = os.environ.get("RYNNRCP_SHM_POLL_INTERVAL_US")
    if raw is None:
        return _DEFAULT_SHM_POLL_INTERVAL_US
    try:
        return max(1, int(raw))
    except ValueError:
        return _DEFAULT_SHM_POLL_INTERVAL_US


def _subscriber_notifier_name(channel_name: str, subscriber_id: str) -> str:
    """Return a registry-safe subscriber notifier name.

    The shared registry stores notifier names in a fixed-size field. Channel
    names can be much longer than that field, so use a deterministic digest
    instead of the raw channel path.
    """
    digest = hashlib.sha1(f"{channel_name}:{subscriber_id}".encode("utf-8")).hexdigest()[:24]
    return f"rc_sub_{digest}"
