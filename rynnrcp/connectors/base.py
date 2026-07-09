"""
Base interface for external protocol connectors.

Defines :class:`BaseConnector`, the abstract base class that all concrete
connectors (ROS2, LCM, Module, Port) must implement.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Callable, Dict


class BaseConnector(ABC):
    """Unified interface for external protocol connectors.

    Concrete connectors must implement:
    - :meth:`pub`  – publish a message
    - :meth:`sub`  – subscribe with a callback
    - :meth:`stop` – release resources
    """

    @abstractmethod
    def pub(self, params: Dict[str, Any], msg: Any) -> None:
        """Publish *msg* using connector-specific *params* (topic, etc.)."""
        ...

    @abstractmethod
    def sub(self, params: Dict[str, Any], callback: Callable) -> None:
        """Subscribe using connector-specific *params* and forward data to *callback*."""
        ...

    @abstractmethod
    def stop(self) -> None:
        """Clean up all resources held by this connector."""
        ...
