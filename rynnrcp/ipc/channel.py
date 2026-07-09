# Copyright 2026 RynnRCP Authors. All rights reserved.
# Core layer: Channel - high-level Pub/Sub communication API.

from __future__ import annotations

import hashlib
import os
import threading
from typing import Any, Callable, Dict, List, Optional, Sequence

from rynnrcp.ipc.transport import (
    TransportLevel, TransportBase, IntraProcessTransport, ShmTransport,
)


class Publisher:
    """Publishes data to a named channel."""

    def __init__(self, channel_name: str, transport: TransportBase) -> None:
        self._channel_name = channel_name
        self._transport = transport

    @property
    def channel_name(self) -> str:
        return self._channel_name

    def publish(self, data: bytes) -> None:
        """Publish data to the channel."""
        self._transport.publish(data)

    def publish_parts(self, parts: Sequence[bytes | bytearray | memoryview]) -> None:
        """Publish one logical message from byte chunks."""
        if hasattr(self._transport, "publish_parts"):
            self._transport.publish_parts(parts)
            return
        self._transport.publish(b"".join(bytes(part) for part in parts))


class Subscriber:
    """Subscribes to a named channel and receives data via callback or polling."""

    def __init__(self, channel_name: str, transport: TransportBase,
                 callback: Optional[Callable[[bytes], None]] = None) -> None:
        self._channel_name = channel_name
        self._transport = transport
        self._callback = callback
        self._last_read_index = -1
        if callback is not None:
            self._transport.subscribe(callback)

    @property
    def channel_name(self) -> str:
        return self._channel_name

    def spin_once(self, timeout_ms: int = 100) -> bool:
        """Poll for one message.

        Returns:
            True if a message was received, False on timeout.
        """
        data = self.poll(timeout_ms=timeout_ms)
        if data is not None:
            return True
        return False

    def poll(self, timeout_ms: int = 100) -> Optional[bytes]:
        """Poll for one raw message without invoking the callback."""
        if hasattr(self._transport, "poll_from"):
            self._last_read_index, data = self._transport.poll_from(
                self._last_read_index,
                timeout_ms=timeout_ms,
            )
            return data
        return self._transport.poll(timeout_ms=timeout_ms)

    def read_latest(self) -> Optional[bytes]:
        """Read the latest message (non-blocking, skips intermediate)."""
        if hasattr(self._transport, "read_latest_from"):
            self._last_read_index, data = self._transport.read_latest_from(self._last_read_index)
            return data
        if isinstance(self._transport, ShmTransport):
            return self._transport.read_latest()
        if isinstance(self._transport, IntraProcessTransport):
            return self._transport.latest()
        return None


# Channel registry entry
class _ChannelEntry:
    def __init__(self, name: str, msg_size: int, transport_level: TransportLevel,
                 transport: TransportBase) -> None:
        self.name = name
        self.msg_size = msg_size
        self.transport_level = transport_level
        self.transport = transport


class ChannelManager:
    """Singleton channel registry. Manages transport lifecycle and auto-routing.

    Usage:
        mgr = ChannelManager.instance()
        pub = mgr.create_publisher("imu", msg_size=64)
        sub = mgr.create_subscriber("imu", msg_size=64, callback=on_imu)
        pub.publish(data)
    """

    _instance: Optional[ChannelManager] = None
    _lock = threading.Lock()

    def __init__(self) -> None:
        self._channels: Dict[str, _ChannelEntry] = {}
        self._channel_lock = threading.Lock()
        self._registry: Any = None  # Optional SharedChannelRegistry

    @classmethod
    def instance(cls) -> ChannelManager:
        """Get or create the singleton ChannelManager."""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = ChannelManager()
        return cls._instance

    @classmethod
    def reset(cls) -> None:
        """Reset the singleton (for testing). Closes all channels."""
        with cls._lock:
            if cls._instance is not None:
                cls._instance.close_all()
                cls._instance = None

    def _get_or_create_channel(self, name: str, msg_size: int,
                                transport_level: Optional[TransportLevel]
                                ) -> _ChannelEntry:
        """Get existing channel or create new one."""
        with self._channel_lock:
            if name in self._channels:
                entry = self._channels[name]
                if entry.msg_size != msg_size:
                    raise ValueError(
                        f"Channel '{name}' already exists with msg_size={entry.msg_size}, "
                        f"requested {msg_size}")
                if transport_level is not None and entry.transport_level != transport_level:
                    raise ValueError(
                        f"Channel '{name}' already exists with transport={entry.transport_level.name}, "
                        f"requested {transport_level.name}")
                return entry

            # Create new channel
            level = transport_level or self.auto_detect_transport(name)
            transport = self._create_transport(name, msg_size, level)
            entry = _ChannelEntry(name, msg_size, level, transport)
            self._channels[name] = entry
            return entry

    def _create_transport(self, name: str, msg_size: int,
                           level: TransportLevel) -> TransportBase:
        """Factory method for creating transport instances."""
        if level in (TransportLevel.L0_INTRA_THREAD, TransportLevel.L1_INTRA_PROCESS):
            return IntraProcessTransport(name, buffer_size=64)
        elif level == TransportLevel.L2_SHM:
            # Check registry: if another process owns it, open existing
            create = True
            if self._registry is not None:
                entry = self._registry.lookup(name)
                if entry is not None and entry.owner_pid != os.getpid():
                    create = False
            transport_name = self._physical_channel_name(name)
            t = ShmTransport(
                transport_name,
                msg_size=msg_size,
                slot_count=16,
                create=create,
                registry=self._registry,
            )
            # Register in shared registry if creating
            if create and self._registry is not None:
                self._registry.register(
                    name, msg_size, os.getpid(), f"rc_transport_{transport_name}"
                )
            return t
        else:
            raise ValueError(f"Unknown transport level: {level}")

    def _physical_channel_name(self, name: str) -> str:
        if self._registry is None:
            return name
        namespace = str(getattr(self._registry, "shm_name", "") or "")
        digest = hashlib.sha1(f"{namespace}:{name}".encode("utf-8")).hexdigest()[:24]
        return f"chn_{digest}"

    def attach_registry(self, registry: Any) -> None:
        """Attach a SharedChannelRegistry for cross-process channel routing.

        When attached, auto_detect_transport will check the registry to
        decide whether to use SHM transport for cross-process channels.
        """
        self._registry = registry

    def auto_detect_transport(self, name: str) -> TransportLevel:
        """Auto-detect the best transport level for a channel.

        Heuristic:
        1. If channel already exists in this process -> L1 (intra-process)
        2. If shared registry shows another process owns it -> L2 (SHM)
        3. Default -> L1 (intra-process)
        """
        if name in self._channels:
            return TransportLevel.L1_INTRA_PROCESS
        # Check shared registry for cross-process channels
        if self._registry is not None:
            entry = self._registry.lookup(name)
            if entry is not None and entry.owner_pid != os.getpid():
                return TransportLevel.L2_SHM
        return TransportLevel.L1_INTRA_PROCESS

    def create_publisher(self, name: str, msg_size: int,
                          transport: Optional[TransportLevel] = None) -> Publisher:
        """Create a publisher for the named channel.

        Args:
            name: Channel name.
            msg_size: Maximum message size in bytes.
            transport: Force a specific transport level, or None for auto-detect.

        Returns:
            A Publisher instance.
        """
        entry = self._get_or_create_channel(name, msg_size, transport)
        return Publisher(name, entry.transport)

    def create_subscriber(self, name: str, msg_size: int,
                           callback: Optional[Callable[[bytes], None]] = None,
                           transport: Optional[TransportLevel] = None) -> Subscriber:
        """Create a subscriber for the named channel.

        Args:
            name: Channel name.
            msg_size: Maximum message size in bytes.
            callback: Optional callback invoked on each message.
            transport: Force a specific transport level, or None for auto-detect.

        Returns:
            A Subscriber instance.
        """
        entry = self._get_or_create_channel(name, msg_size, transport)
        if isinstance(entry.transport, ShmTransport):
            entry.transport.register_subscriber()
        return Subscriber(name, entry.transport, callback)

    def get_channel_info(self, name: str) -> Optional[dict]:
        """Get info about a channel."""
        with self._channel_lock:
            if name in self._channels:
                e = self._channels[name]
                return {
                    "name": e.name,
                    "msg_size": e.msg_size,
                    "transport_level": e.transport_level.name,
                }
        return None

    def list_channels(self) -> List[str]:
        """List all registered channel names."""
        with self._channel_lock:
            return list(self._channels.keys())

    def close_all(self) -> None:
        """Close all channels and their transports."""
        with self._channel_lock:
            for entry in self._channels.values():
                entry.transport.close()
                if isinstance(entry.transport, ShmTransport):
                    entry.transport.unlink()
            self._channels.clear()
            self._registry = None
