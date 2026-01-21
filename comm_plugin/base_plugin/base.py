# comm_plugin/base.py

"""
Base interface for communication plugins.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~comm_plugin.base.RcpPlugin`, the minimal contract a
transport/backend must implement to integrate with :class:`rcp_core.RcpCore`.

A plugin is expected to follow this lifecycle:

- ``bind_rcp_core(core)``: attach the plugin instance to the core runtime
- ``start()``: start the backend and begin processing
- ``stop()``: stop the backend and tear down any associated resources

Concrete implementations should live in ``comm_plugin.*``.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from rcp_core import RcpCore


class RcpPlugin(ABC):
    """Base interface for all communication plugins that need to be implemented."""

    @abstractmethod
    def bind_rcp_core(self, core: "RcpCore") -> None:
        """Bind the communication plugin to the RcpCore instance."""
        ...

    @abstractmethod
    def start(self) -> None:
        """Start the communication plugin to begin operation."""
        ...

    @abstractmethod
    def stop(self) -> None:
        """Stop the plugin and release any allocated resources."""
        ...
