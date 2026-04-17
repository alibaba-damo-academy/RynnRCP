# rcp_core/common/utils/global_buffer.py

"""
Global, per-server partitioned time-series buffer.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.utils.global_buffer.GlobalBuffer`, a
process-wide singleton used to store recent time-stamped values produced by server
input subscriptions.

Data model
----------
Buffers are partitioned by ``server_name``:

    {
      server_name: {
        key: deque([(ts, value), ...]),
        ...
      },
      ...
    }

Each server partition has its own mutex in ``_locks``. A separate ``_meta_lock`` protects
creation of new server partitions. Within a server partition, each key has its own
per-key lock so that concurrent writers to different keys do not block each other.

Core operations
---------------
- :meth:`GlobalBuffer.get`:
  returns the singleton instance (creating it with ``queue_len`` and ``expire_seconds``
  on first use).
- :meth:`GlobalBuffer.push(server_name, key, ts, value)`:
  appends ``(ts, value)`` to the per-key deque and enforces ``max_queue_len`` if set.
- :meth:`GlobalBuffer.snapshot(server_name)`:
  returns a *reference* to the internal buffer dict for that server after applying
  expiration cleanup (drops items older than ``expire_seconds``).
- :meth:`GlobalBuffer.snapshot_all()`:
  returns a shallow snapshot of the global mapping ``{server_name: buffer_ref}``
  (buffer dicts are still internal references).

Thread-safety and caveats
-------------------------
- Writes and per-server cleanup are serialized by the per-server lock.
- ``snapshot()`` and ``snapshot_all()`` return internal references; callers should treat
  returned structures as read-only and avoid mutating keys/deque objects directly.
"""

from __future__ import annotations

import threading
import time
from collections import deque, defaultdict
from typing import Any, Dict, Deque, List, Tuple, Optional


class GlobalBuffer:
    """
    Global shared buffer.

    Top-level partitioned by server_name. Within each partition every key has its
    own lock so concurrent writers to different keys (e.g. three camera threads)
    do not block each other:

        _buffers : { server_name: { key: Deque[(ts, value)] } }
        _key_locks: { server_name: { key: Lock } }
        _meta_lock: protects creation of new server_name entries
    """

    _instance: Optional["GlobalBuffer"] = None
    _instance_lock = threading.Lock()

    # FPS report interval (seconds)
    FPS_REPORT_INTERVAL: float = 5.0

    def __init__(self, queue_len: Optional[int] = 100, expire_seconds: float = 1.0):
        # server_name -> (key -> deque[(ts, value)])
        self._buffers: Dict[str, Dict[str, Deque[Tuple[float, Any]]]] = {}
        # server_name -> (key -> Lock)  — per-key locks for fine-grained concurrency
        self._key_locks: Dict[str, Dict[str, threading.Lock]] = {}
        # Protects the structure of _buffers/_key_locks themselves (creating new
        # server partitions or new keys within a partition)
        self._meta_lock = threading.Lock()

        self.max_queue_len: Optional[int] = queue_len
        self.expire_seconds: float = expire_seconds

        # FPS tracking: (server_name, key) -> push count in current window
        self._fps_counts: Dict[Tuple[str, str], int] = defaultdict(int)
        self._fps_window_start: float = time.time()
        self._fps_lock = threading.Lock()

    @classmethod
    def get(
        cls, queue_len: Optional[int] = 100, expire_seconds: float = 1.0
    ) -> "GlobalBuffer":
        """Get the singleton GlobalBuffer instance, creating it if necessary."""

        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = GlobalBuffer(
                    queue_len=queue_len, expire_seconds=expire_seconds
                )
            return cls._instance

    def _ensure_server_structs(self, server_name: str) -> None:
        """
        Ensure that the given server_name has a corresponding buffer dict and
        key-lock dict. Only called inside _meta_lock.
        """
        if server_name not in self._buffers:
            self._buffers[server_name] = {}
        if server_name not in self._key_locks:
            self._key_locks[server_name] = {}

    def _get_key_lock(self, server_name: str, key: str) -> Tuple[Deque, threading.Lock]:
        """
        Return (deque, lock) for (server_name, key), creating them if needed.
        Uses _meta_lock only for the creation step; the returned lock is per-key.
        """
        with self._meta_lock:
            self._ensure_server_structs(server_name)
            buf = self._buffers[server_name]
            kl = self._key_locks[server_name]
            if key not in buf:
                buf[key] = deque()
                kl[key] = threading.Lock()
            return buf[key], kl[key]

    def _maybe_report_fps(self, now: float) -> None:
        """Print per-key FPS report if the reporting interval has elapsed."""
        elapsed = now - self._fps_window_start
        if elapsed < self.FPS_REPORT_INTERVAL:
            return

        # Import lazily to avoid circular imports
        from rcp_core.common.utils.logger import server_logger
        _logger = server_logger()

        lines: List[str] = []
        for (sname, key), count in sorted(self._fps_counts.items()):
            fps = count / elapsed if elapsed > 0 else 0.0
            lines.append(f"  [{sname}] {key}: {fps:.1f} fps ({count} frames / {elapsed:.1f}s)")

        self._fps_counts.clear()
        self._fps_window_start = now

        if lines:
            _logger.info(
                "[GlobalBuffer] FPS report (last {:.1f}s):\n{}".format(
                    elapsed, "\n".join(lines)
                )
            )

    def push(self, server_name: str, key: str, ts: float, value: Any):
        """Append a record to buffer[key] for the given server.

        Uses a per-key lock so concurrent pushes to different keys
        (e.g. three camera threads) do not block each other.
        """
        q, lock = self._get_key_lock(server_name, key)
        with lock:
            q.append((ts, value))
            if self.max_queue_len is not None:
                while len(q) > self.max_queue_len:
                    q.popleft()

        # FPS tracking
        now = time.time()
        with self._fps_lock:
            self._fps_counts[(server_name, key)] += 1
            self._maybe_report_fps(now)

    def snapshot(self, server_name: str) -> Dict[str, Deque[Tuple[float, Any]]]:
        """
        Get the buffer reference for the given server, performing expiration
        cleanup before returning.

        Note:
          The returned value is a reference to internal structures, and should
          be treated as read-only by callers.
        """
        with self._meta_lock:
            self._ensure_server_structs(server_name)
            buf = self._buffers[server_name]
            kl = self._key_locks[server_name]

        now = time.time()
        expire_before = now - self.expire_seconds
        for key, q in list(buf.items()):
            lock = kl.get(key)
            if lock is None:
                continue
            with lock:
                while q and q[0][0] < expire_before:
                    q.popleft()

        return buf

    def snapshot_all(self) -> Dict[str, Dict[str, Deque[Tuple[float, Any]]]]:
        """
        Get a snapshot view of buffers for all servers.

        This performs a shallow snapshot:

          - Returns a new dict: { server_name: buf_ref, ... }
          - buf_ref is a reference to the internal buffer dict of that server.
          - Callers should treat it as read-only: iterate and read from the
            deques, but do not modify the structure itself (do not add/remove keys).
        """
        with self._meta_lock:
            # Shallow copy the mapping: server_name -> buffer_ref
            return dict(self._buffers)
