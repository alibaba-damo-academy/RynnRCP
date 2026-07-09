"""Base helpers for services that expose ToolBus methods."""

import logging
import struct
from typing import Any, Dict

from rynnrcp.runtime.tool_bus import ToolBus

logger = logging.getLogger(__name__)


class BaseService:
    """Shared ToolBus registration and latest-channel cache."""

    def __init__(self, bus: ToolBus, name: str):
        self._bus = bus
        self._name = name
        self._subscribers: Dict[str, Any] = {}
        self._latest_cache: Dict[str, bytes] = {}
        self._latest_parsed_cache: Dict[str, tuple[bytes, float, bytes]] = {}
        self._tool_names: list[str] = []

    @property
    def name(self) -> str:
        return self._name

    @property
    def bus(self) -> ToolBus:
        return self._bus

    @property
    def tool_names(self) -> list[str]:
        return list(self._tool_names)

    def subscribe_channel(self, channel_name: str, msg_size: int, transport: str = "shm") -> None:
        """Subscribe to a runner channel for latest-value reads."""
        if channel_name in self._subscribers:
            return
        try:
            from rynnrcp.ipc.channel import ChannelManager
            from rynnrcp.ipc.transport import parse_transport_level

            sub = ChannelManager.instance().create_subscriber(
                channel_name,
                msg_size,
                transport=parse_transport_level(transport),
            )
            self._subscribers[channel_name] = sub
            logger.debug("[%s] Subscribed to channel: %s", self._name, channel_name)
        except Exception as e:
            logger.warning(
                "[%s] Failed to subscribe to %s (transport=%s, msg_size=%s): %s",
                self._name,
                channel_name,
                transport,
                msg_size,
                e,
                exc_info=True,
            )

    def latest(self, channel_name: str) -> bytes | None:
        """Return the newest raw channel message, falling back to cache."""
        sub = self._subscribers.get(channel_name)
        if sub is None:
            return None
        try:
            raw = sub.read_latest()
        except Exception:
            raw = None
        if raw is not None:
            self._latest_cache[channel_name] = raw
            return raw
        return self._latest_cache.get(channel_name)

    def latest_parsed(self, channel_name: str):
        """Return ``(timestamp, payload_bytes)`` or ``(None, None)``."""
        raw = self.latest(channel_name)
        if raw is None or len(raw) < 8:
            return None, None
        cached = self._latest_parsed_cache.get(channel_name)
        if cached is not None and cached[0] is raw:
            return cached[1], cached[2]
        ts = struct.unpack("<d", raw[:8])[0]
        payload = raw[8:]
        self._latest_parsed_cache[channel_name] = (raw, ts, payload)
        return ts, payload

    def bind(self) -> None:
        raise NotImplementedError

    def unbind(self) -> None:
        for name in list(self._tool_names):
            self._bus.remove_tool(name)
        self._tool_names.clear()
        logger.debug("[%s] Unbound all tools", self._name)

    def _register_tool(self, name: str, handler, **kwargs) -> None:
        self._bus.add_tool(name, handler, **kwargs)
        self._tool_names.append(name)
