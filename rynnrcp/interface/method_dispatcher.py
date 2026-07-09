"""Dispatch Interface requests to server methods."""

from __future__ import annotations

import time
from typing import Any, Mapping

from rynnrcp.interface.codec import InterfaceError, InterfaceRequest, MethodNotFoundError, ok_response


class InterfaceMethodDispatcher:
    """Translate Interface method/payload calls into server method calls."""

    def __init__(
        self,
        methods: Any,
        *,
        server_id: str,
        server_instance_id: str,
        config_name: str,
        display_name: str | None = None,
        capabilities: Mapping[str, Any] | None = None,
        metadata: Mapping[str, Any] | None = None,
    ):
        self.methods = methods
        self.server_id = server_id
        self.server_instance_id = server_instance_id
        self.config_name = config_name
        self.display_name = display_name or server_id
        self.capabilities = dict(capabilities or {})
        self.metadata = dict(metadata or {})

    def __call__(self, request: InterfaceRequest) -> Any:
        if request.method == "ping":
            return {
                "server_id": self.server_id,
                "server_time": time.time(),
                "request_payload": request.payload,
            }
        if request.method == "list_tools":
            return self.methods.list_tools()
        if not self.methods.has_tool(request.method):
            raise MethodNotFoundError(f"method not found: {request.method}")
        started = time.perf_counter()
        result = self._call_method(request.method, request.payload)
        called_at = time.perf_counter()
        payload = _protocol_payload(result)
        finished = time.perf_counter()
        return ok_response(
            request.request_id,
            payload,
            metadata={
                "dispatcher_timing_ms": {
                    "tool_call": _ms(called_at - started),
                    "protocol_unwrap": _ms(finished - called_at),
                    "total": _ms(finished - started),
                }
            },
        )

    def _call_method(self, method: str, payload: Any) -> Any:
        if payload is None:
            return self.methods.call_tool(method)
        if isinstance(payload, dict):
            return self.methods.call_tool(method, **payload)
        return self.methods.call_tool(method, payload)


def _protocol_payload(result: Any) -> Any:
    if not isinstance(result, dict) or "success" not in result or "result" not in result:
        return result
    if bool(result.get("success")):
        return result.get("result")
    raise InterfaceError(str(result.get("message") or "method failed"))


def _ms(seconds: float) -> float:
    return round(float(seconds) * 1000.0, 3)
