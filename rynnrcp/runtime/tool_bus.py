"""Tool registry and dispatch for AI Agent interaction.

Ported from RynnRCP's RcpBus with improvements:
- Thread-safe (RLock protected)
- remove_tool() for dynamic service lifecycle
- list_tools() returns structured ToolInfo
"""

import inspect
import logging
import threading
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

logger = logging.getLogger(__name__)


@dataclass
class ToolInfo:
    """Metadata for a registered tool."""

    handler: Callable
    input_schema: Optional[Dict[str, Any]] = None
    output_schema: Optional[Dict[str, Any]] = None
    description: str = ""
    internal: bool = False
    accepts_progress_callback: bool = False


class ToolBus:
    """Tool registry and dispatch for AI Agent interaction.

    Thread-safe registry that allows services to register callable tools
    and external callers (AI Agents) to discover and invoke them.

    Example::

        bus = ToolBus()
        bus.add_tool("get_observations", handler_fn, description="Get robot observations")
        result = bus.call_tool("get_observations", names=["observation.<component>.joint_state"])
    """

    def __init__(self):
        self._tools: Dict[str, ToolInfo] = {}
        self._lock = threading.RLock()

    # --- Registration -------------------------------------------------

    def add_tool(
        self,
        name: str,
        handler: Callable,
        input_schema: Optional[Dict[str, Any]] = None,
        output_schema: Optional[Dict[str, Any]] = None,
        description: str = "",
        internal: bool = False,
    ) -> None:
        """Register a tool with its handler and metadata.

        Args:
            name: Unique tool name.
            handler: Callable invoked when tool is called.
            input_schema: Optional description of expected inputs.
            output_schema: Optional description of expected outputs.
            description: Human-readable description.
            internal: If True, tool is hidden from list_tools().
        """
        with self._lock:
            self._tools[name] = ToolInfo(
                handler=handler,
                input_schema=input_schema,
                output_schema=output_schema,
                description=description,
                internal=internal,
                accepts_progress_callback=_accepts_progress_callback(handler),
            )
        logger.debug("Registered tool: %s (internal=%s)", name, internal)

    def remove_tool(self, name: str) -> bool:
        """Remove a tool by name. Returns True if removed, False if not found."""
        with self._lock:
            if name in self._tools:
                del self._tools[name]
                logger.debug("Removed tool: %s", name)
                return True
            return False

    # --- Discovery ----------------------------------------------------

    def list_tools(self) -> Dict[str, Dict[str, Any]]:
        """Return all non-internal tools with descriptions and schemas.

        Returns:
            Dict mapping tool name to {"description", "input", "output"}.
        """
        with self._lock:
            return {
                name: {
                    "description": info.description,
                    "input": info.input_schema,
                    "output": info.output_schema,
                }
                for name, info in self._tools.items()
                if not info.internal
            }

    def has_tool(self, name: str) -> bool:
        """Check if a tool is registered."""
        with self._lock:
            return name in self._tools

    # --- Invocation ---------------------------------------------------

    def call_tool(
        self,
        tool_name: str,
        /,
        *args,
        _progress_callback: Optional[Callable] = None,
        **kwargs,
    ) -> Any:
        """Invoke a registered tool by name.

        If _progress_callback is provided and the handler declares a
        _progress_callback parameter, it is forwarded automatically.

        Args:
            name: Tool name to invoke.
            *args: Positional arguments passed to handler.
            _progress_callback: Optional (current, total, message) callable.
            **kwargs: Keyword arguments passed to handler.

        Returns:
            Whatever the handler returns.

        Raises:
            KeyError: If tool name is not registered.
        """
        with self._lock:
            info = self._tools.get(tool_name)
            if info is None:
                raise KeyError(f"Tool '{tool_name}' not found. Available: {list(self._tools.keys())}")
            handler = info.handler
            accepts_progress_callback = info.accepts_progress_callback

        if _progress_callback is not None and accepts_progress_callback:
            kwargs["_progress_callback"] = _progress_callback

        return handler(*args, **kwargs)

    # --- Result Helper ------------------------------------------------

    @staticmethod
    def make_result(
        success: bool, result: Any = None, message: Optional[str] = None
    ) -> Dict[str, Any]:
        """Create a unified result structure for tool returns.

        Args:
            success: Whether the operation succeeded.
            result: Payload data.
            message: Human-readable status message.

        Returns:
            {"success": bool, "message": str, "result": Any}
        """
        return {
            "success": success,
            "message": message or "",
            "result": result,
        }


def _accepts_progress_callback(handler: Callable) -> bool:
    try:
        sig = inspect.signature(handler)
    except (TypeError, ValueError):
        return False
    return "_progress_callback" in sig.parameters
