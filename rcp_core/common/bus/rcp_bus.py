# rcp_core/common/bus/rcp_bus.py

"""
Tool bus / RPC registry for rcp_core.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.bus.rcp_bus.RcpBus`, a lightweight registry
and dispatch mechanism for "tools" (callable capabilities) exposed by servers within
``rcp_core``.

Key responsibilities:
- registration via :meth:`RcpBus.add_tool`, storing the handler along with optional
  input/output schema metadata and a human-readable description
- introspection via :meth:`RcpBus.list_tool`, returning a structured description of
  all registered tools
- invocation via :meth:`RcpBus.call_tool`, dispatching calls by tool name
- standard result formatting via :meth:`RcpBus.make_result`, returning a unified
  ``{"success": bool, "message": str, "result": Any}`` dict used across the system
"""

import inspect
from typing import Any, Callable, Dict, Optional
from rcp_core.common.utils.logger import server_logger

logger = server_logger()


class RcpBus:
    def __init__(self):
        """Initialize the tool bus, maintaining a mapping from tool name to tool info."""
        self.tools: Dict[str, Dict[str, Any]] = {}

    def add_tool(
        self,
        name: str,
        handler: Callable,
        input_schema=None,
        output_schema=None,
        description: str = "",
        internal: bool = False,
    ):
        """
        Register a tool with its handler, input/output schema descriptions, and an optional description.

        :param name
        :param handler
        :param input_schema
        :param output_schema
        :param description
        """
        logger.info(
            f"Registering tool: {name}, internal={internal}, description: {description}"
        )

        self.tools[name] = {
            "handler": handler,
            "input_schema": input_schema,
            "output_schema": output_schema,
            "description": description,
            "internal": bool(internal),
        }

    def list_tool(self):
        """Return all registered tools with their descriptions and input/output schemas."""
        return {
            n: {
                "description": t.get("description", ""),
                "input": t.get("input_schema"),
                "output": t.get("output_schema"),
            }
            for n, t in self.tools.items()
            if not bool(t.get("internal", False))
        }

    def call_tool(
        self,
        name: str,
        *args,
        _progress_callback: Optional[Callable] = None,
        **kwargs,
    ):
        """Invoke a registered tool handler by name.

        If *_progress_callback* is provided **and** the handler declares a
        ``_progress_callback`` parameter (detected via :func:`inspect.signature`),
        the callback is forwarded automatically.  Handlers that do not declare
        this parameter are called without it—no hardcoded allowlists needed.

        :param name: Registered tool name.
        :param _progress_callback: Optional ``(current, total, message) -> None``
                                   callable supplied by the caller (plugin).
        """
        if name not in self.tools:
            raise ValueError(f"Tool {name} not found.")
        handler = self.tools[name]["handler"]
        if _progress_callback is not None:
            sig = inspect.signature(handler)
            if "_progress_callback" in sig.parameters:
                kwargs["_progress_callback"] = _progress_callback
        return handler(*args, **kwargs)

    def make_result(
        self, success: bool, result: Any = None, message: str | None = None
    ) -> Dict[str, Any]:
        """Create a unified result structure for tool calls."""
        return {
            "success": success,
            "message": message or "",
            "result": result,
        }
