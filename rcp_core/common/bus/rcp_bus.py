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

from typing import Any, Callable, Dict
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
    ):
        """
        Register a tool with its handler, input/output schema descriptions, and an optional description.

        :param name
        :param handler
        :param input_schema
        :param output_schema
        :param description
        """
        logger.info(f"Registering tool: {name}, description: {description}")

        self.tools[name] = {
            "handler": handler,
            "input_schema": input_schema,
            "output_schema": output_schema,
            "description": description,
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
        }

    def call_tool(self, name: str, *args, **kwargs):
        """Invoke a registered tool handler by name."""
        if name not in self.tools:
            raise ValueError(f"Tool {name} not found.")
        return self.tools[name]["handler"](*args, **kwargs)

    def make_result(
        self, success: bool, result: Any = None, message: str | None = None
    ) -> Dict[str, Any]:
        """Create a unified result structure for tool calls."""
        return {
            "success": success,
            "message": message or "",
            "result": result,
        }
