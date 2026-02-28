# comm_plugin/mcp_plugin/mcp_plugin.py
"""
MCP Plugin for RCP Core (FastMCP bridge).

This module implements :class:`McpPlugin`, which exposes all tools registered in
:class:`rcp_core.RcpCore` through a FastMCP server.

Key features
------------
- Configuration resolution with the following precedence:
  1) Environment variables (highest priority)
  2) Explicit constructor arguments OR a YAML config file (mutually exclusive)
  3) Code defaults as fallback

- Environment variables:
  Required (must all be set if any MCP_* is set):
    - MCP_SERVER_NAME
    - MCP_HOST
    - MCP_PORT
  Optional:
    - MCP_PATH
    - MCP_TRANSPORT

- YAML config:
  Reads keys from ``mcp.<key>`` first; if missing, falls back to top-level ``<key>``.
  Supported keys: server_name, host, port, path, transport

- Tool exposure:
  Every tool returned by ``rcp_core.tool_list()`` is registered as an MCP tool.
  The tool description is augmented with JSON-formatted input/output specs when available.

- JSON safety:
  ``to_json_safe`` converts bytes-like objects to base64 strings and prevents circular
  references when serializing nested structures.

Notes
-----
- ``stop()`` is currently a no-op (server shutdown is not implemented here).
"""

from __future__ import annotations

import os
import json
import yaml
import base64
from dataclasses import dataclass
from collections.abc import Mapping, Sequence, Set
from typing import Any

from fastmcp import FastMCP, Context

from ..base_plugin.base import RcpPlugin
from rcp_core import RcpCore


def to_json_safe(obj: Any, _seen: set[int] | None = None) -> Any:
    if _seen is None:
        _seen = set()

    if isinstance(obj, (bytes, bytearray, memoryview)):
        return base64.b64encode(bytes(obj)).decode("ascii")

    if obj is None or isinstance(obj, (str, int, float, bool)):
        return obj

    oid = id(obj)
    if oid in _seen:
        return "<circular_ref>"
    _seen.add(oid)

    try:
        if isinstance(obj, Mapping):
            out = {}
            for k, v in obj.items():
                if not isinstance(k, str):
                    k = str(k)
                out[k] = to_json_safe(v, _seen)
            return out

        if isinstance(obj, Sequence):
            return [to_json_safe(v, _seen) for v in obj]

        if isinstance(obj, Set):
            return [to_json_safe(v, _seen) for v in obj]

        return str(obj)
    finally:
        _seen.remove(oid)


@dataclass
class McpConfig:
    server_name: str | None = None
    host: str | None = None
    port: int | None = None
    path: str | None = None
    transport: str | None = None


def _load_yaml(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _deep_get(d: dict[str, Any], keys: list[str], default=None):
    cur: Any = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def _env_int(name: str) -> int | None:
    v = os.getenv(name)
    if v is None or v == "":
        return None
    try:
        return int(v)
    except Exception:
        raise ValueError(f"Invalid env var {name}={v!r}, expected int")


class McpPlugin(RcpPlugin):
    def __init__(
        self,
        *,
        server_name: str | None = None,
        host: str | None = None,
        port: int | None = None,
        path: str | None = None,
        transport: str | None = None,
        config_file: str | None = None,
    ) -> None:
        cfg = self._resolve_config(
            server_name=server_name,
            host=host,
            port=port,
            path=path,
            transport=transport,
            config_file=config_file,
        )

        # Default fallback values: allows specifying only server_name/host/port, etc.
        self.server_name = cfg.server_name or "rcp_server"
        self.host = cfg.host or "0.0.0.0"
        self.port = int(cfg.port or 8000)
        self.path = cfg.path or "/mcp"
        self.transport = cfg.transport or "streamable-http"

        self.rcp_core: RcpCore | None = None
        self.app: FastMCP | None = None

    def _resolve_config(
        self,
        *,
        server_name: str | None,
        host: str | None,
        port: int | None,
        path: str | None,
        transport: str | None,
        config_file: str | None,
    ) -> McpConfig:
        """
        Configuration resolution rules:

        1) Environment variables have the highest priority.
           - If any MCP_* env var is set, then ALL required env vars must be set,
             otherwise an error is raised:
               MCP_SERVER_NAME / MCP_HOST / MCP_PORT
           - MCP_PATH / MCP_TRANSPORT are optional; if not set, defaults apply.

        2) Constructor args and config_file are mutually exclusive.
           - If any explicit args are provided together with config_file => error.
           - (Env vars are still allowed and take priority.)

        3) Any unspecified fields fall back to code defaults (see __init__).

        Env vars:
          Required: MCP_SERVER_NAME / MCP_HOST / MCP_PORT
          Optional: MCP_PATH / MCP_TRANSPORT

        YAML:
          Reads mcp.<key> first, then falls back to top-level <key>.
        """
        any_args_specified = any(
            v is not None for v in (server_name, host, port, path, transport)
        )
        if config_file and any_args_specified:
            raise ValueError(
                "McpPlugin config conflict: provide either explicit args "
                "or config_file, not both. (Env vars are allowed and take priority.)"
            )

        env_required = ["MCP_SERVER_NAME", "MCP_HOST", "MCP_PORT"]
        env_optional = ["MCP_PATH", "MCP_TRANSPORT"]

        present_required = {k: (os.getenv(k) not in (None, "")) for k in env_required}
        present_optional = {k: (os.getenv(k) not in (None, "")) for k in env_optional}

        any_env = any(present_required.values()) or any(present_optional.values())
        if any_env and not all(present_required.values()):
            missing = [k for k, ok in present_required.items() if not ok]
            raise ValueError(
                "Incomplete MCP env config: since some MCP_* env var is set, "
                "the required env vars must all be set: "
                f"{env_required}. Missing: {missing}"
            )

        # 1) Env has top priority (required trio is complete here).
        if any_env:
            return McpConfig(
                server_name=os.getenv("MCP_SERVER_NAME"),
                host=os.getenv("MCP_HOST"),
                port=_env_int("MCP_PORT"),
                path=os.getenv("MCP_PATH") or None,
                transport=os.getenv("MCP_TRANSPORT") or None,
            )

        # 2) Otherwise use args XOR config_file.
        base = McpConfig()

        if config_file:
            if not os.path.exists(config_file):
                raise FileNotFoundError(
                    f"McpPlugin config_file not found: {config_file}"
                )
            raw = _load_yaml(config_file)

            def from_file(key: str, default=None):
                return _deep_get(raw, ["mcp", key], _deep_get(raw, [key], default))

            v = from_file("server_name")
            if v not in (None, ""):
                base.server_name = str(v)

            v = from_file("host")
            if v not in (None, ""):
                base.host = str(v)

            v = from_file("port")
            if v not in (None, ""):
                try:
                    base.port = int(v)
                except Exception:
                    raise ValueError(f"Invalid mcp.port in config_file: {v!r}")

            v = from_file("path")
            if v not in (None, ""):
                base.path = str(v)

            v = from_file("transport")
            if v not in (None, ""):
                base.transport = str(v)

        else:
            base.server_name = server_name
            base.host = host
            base.port = int(port) if port is not None else None
            base.path = path
            base.transport = transport

        return base

    def bind_rcp_core(self, rcp_core: RcpCore) -> None:
        self.rcp_core = rcp_core

    def start(self) -> None:
        if self.rcp_core is None:
            raise RuntimeError("rcp_core is not bound. Call bind_rcp_core(...) first.")

        self.app = self._build_mcp_app(self.rcp_core)
        self.app.run(
            transport=self.transport,
            host=self.host,
            port=self.port,
            path=self.path,
            show_banner=False,
        )

    def list_tools(self) -> list[str]:
        if self.rcp_core is None:
            raise RuntimeError("rcp_core is not bound. Call bind_rcp_core(...) first.")
        return list(self.rcp_core.tool_list().keys())

    def call_tool(self, name: str, *args, **kwargs) -> Any:
        if self.rcp_core is None:
            raise RuntimeError("rcp_core is not bound. Call bind_rcp_core(...) first.")
        return self.rcp_core.tool_call(name, *args, **kwargs)

    def _build_mcp_app(self, rcp_core: RcpCore) -> FastMCP:
        app = FastMCP(self.server_name)

        tools_meta: dict[str, dict[str, Any]] = rcp_core.tool_list()

        for tool_name, meta in tools_meta.items():
            description: str = meta.get("description", "") or ""
            input_meta = meta.get("input")
            output_meta = meta.get("output")

            if input_meta:
                description += "\n\nInput spec:\n" + json.dumps(
                    input_meta, ensure_ascii=False, indent=2
                )
            if output_meta:
                description += "\n\nOutput spec:\n" + json.dumps(
                    output_meta, ensure_ascii=False, indent=2
                )

            def make_tool(name: str, desc: str):
                @app.tool(name=name, description=desc)
                async def _tool(ctx: Context, params: dict[str, Any] | None = None):
                    params = params or {}
                    try:
                        result = rcp_core.tool_call(name, **params)
                        return to_json_safe(result)
                    except Exception as e:
                        return {
                            "success": False,
                            "message": f"Exception in tool '{name}': {e}",
                            "result": None,
                        }

                return _tool

            make_tool(tool_name, description)

        return app

    def stop(self) -> None:
        pass
