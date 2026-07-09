"""
Base input adapter.

Defines the abstract interface for normalizing raw messages from any
connector into a standardized ``(timestamp, {key: value})`` tuple.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict, Tuple


class BaseInputAdapter(ABC):
    """Abstract base for input adapters.

    Subclasses implement :meth:`parse` to convert connector-specific messages
    into a ``(timestamp, data_dict)`` tuple.
    """

    @abstractmethod
    def parse(self, msg: Any) -> Tuple[float, Dict[str, Any]]:
        """Parse a raw message into ``(timestamp, {object_name: value})``."""
        ...
