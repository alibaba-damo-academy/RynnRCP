# rcp_core/common/middleware/base_protocol_adapter.py

"""
Base interface for middleware/protocol adapters.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.middleware.base_protocol_adapter.BaseProtocolAdapter`,
an abstract base class that standardizes how rcp_core publishes to and subscribes from
different middleware transports (e.g., ROS 2, LCM, in-process "module" adapters).

Concrete protocol adapters must implement:
- :meth:`BaseProtocolAdapter.pub` — publish a message using protocol-specific params
  (topic, QoS, etc.)
- :meth:`BaseProtocolAdapter.sub` — subscribe using protocol-specific params and a callback
- :meth:`BaseProtocolAdapter.stop` — release/cleanup any held resources
"""

from abc import ABC, abstractmethod
from typing import Any, Callable, Dict


class BaseProtocolAdapter(ABC):
    """Unified interface for middleware protocol adapters."""

    @abstractmethod
    def pub(self, params: Dict[str, Any], msg: Any) -> None:
        """Publish a message with given params (topic, etc.)."""
        ...

    @abstractmethod
    def sub(self, params: Dict[str, Any], callback: Callable) -> None:
        """Subscribe with given params and callback."""
        ...

    @abstractmethod
    def stop(self) -> None:
        """Clean up resources."""
        ...
