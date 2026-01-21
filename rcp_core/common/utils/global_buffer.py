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
creation of new server partitions.

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
from collections import deque
from typing import Any, Dict, Deque, Tuple, Optional


class GlobalBuffer:
    """
    Global shared buffer.

    Top-level partitioned by server_name. Each partition has its own buffer and lock:

        {
          server_name0: { key: Deque[(ts, value)], ... },
          server_name1: { key: Deque[(ts, value)], ... },
          ...
        }

    And:
        _locks: { server_name: Lock }
    """

    _instance: Optional["GlobalBuffer"] = None
    _instance_lock = threading.Lock()

    def __init__(self, queue_len: Optional[int] = 100, expire_seconds: float = 1.0):
        # server_name -> (key -> deque[(ts, value)])
        self._buffers: Dict[str, Dict[str, Deque[Tuple[float, Any]]]] = {}
        # server_name -> Lock
        self._locks: Dict[str, threading.Lock] = {}
        # Protects the structure of _buffers/_locks themselves (creating new
        # server partitions)
        self._meta_lock = threading.Lock()

        self.max_queue_len: Optional[int] = queue_len
        self.expire_seconds: float = expire_seconds

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
        Ensure that the given server_name has a corresponding buffer and lock.
        Only called inside _meta_lock.
        """
        if server_name not in self._buffers:
            self._buffers[server_name] = {}
        if server_name not in self._locks:
            self._locks[server_name] = threading.Lock()

    def _get_server_buffer_and_lock(
        self, server_name: str
    ) -> Tuple[Dict[str, Deque[Tuple[float, Any]]], threading.Lock]:
        """
        Return (buffer, lock) for the given server_name, where:
          buffer: { key: Deque[(ts, value)] }
          lock:   mutex protecting that buffer
        """
        with self._meta_lock:
            self._ensure_server_structs(server_name)
            buf = self._buffers[server_name]
            lock = self._locks[server_name]
        return buf, lock

    def push(self, server_name: str, key: str, ts: float, value: Any):
        """Append a record to buffer[key] for the given server."""
        buf, lock = self._get_server_buffer_and_lock(server_name)
        with lock:
            q = buf.setdefault(key, deque())
            q.append((ts, value))

            if self.max_queue_len is not None:
                while len(q) > self.max_queue_len:
                    q.popleft()

    def snapshot(self, server_name: str) -> Dict[str, Deque[Tuple[float, Any]]]:
        """
        Get the buffer reference for the given server, performing expiration
        cleanup before returning.

        Note:
          The returned value is a reference to internal structures, and should
          be treated as read-only by callers.
        """
        buf, lock = self._get_server_buffer_and_lock(server_name)
        with lock:
            now = time.time()
            expire_before = now - self.expire_seconds

            for key, q in list(buf.items()):
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
