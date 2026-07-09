"""
McpApp - Exposes one RynnRCP Server's Interface tools through FastMCP.

Key features:
- Tool exposure: every server tool is registered as an MCP tool
- JSON safety: handles bytes, circular refs in tool results
"""

from __future__ import annotations

import asyncio
import argparse
import json
import logging
import sys
import yaml
import base64
from importlib import resources
from collections.abc import Mapping, Sequence, Set
from typing import Any, Dict, Optional

from fastmcp import FastMCP, Context

from rynnrcp.interface.client import ClientInterface
from rynnrcp.interface.protocol_client import RcpProtocolClient, ServerManifest, resolve_server_manifest
from rynnrcp.utils.logging import configure_logging
from rynnrcp.utils.user_paths import log_file_from_config
from rynnrcp_app_common import AppLifecycle

logger = logging.getLogger(__name__)


# --------------------------------------------------------------------------- #
# JSON Safety
# --------------------------------------------------------------------------- #

def to_json_safe(obj: Any, _seen: Optional[set] = None) -> Any:
    """Convert object to JSON-safe representation.

    Handles bytes (base64), circular references, and nested structures.
    """
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


# --------------------------------------------------------------------------- #
# Config helpers
# --------------------------------------------------------------------------- #

def _load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return _app_config_values(raw)


def _load_default_config() -> Dict[str, Any]:
    path = resources.files(__package__).joinpath("default_config.yaml")
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise TypeError("MCP default_config.yaml must contain a YAML mapping")
    return _app_config_values(raw)


def _merge_default_config(config: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    merged = _load_default_config()
    if config:
        merged.update(config)
    return merged


def _app_config_values(raw: Mapping[str, Any]) -> Dict[str, Any]:
    values: Dict[str, Any] = {}
    for key in ("config_type", "version"):
        if key in raw:
            values[key] = raw[key]
    if isinstance(raw.get("defaults"), dict):
        values.update(raw["defaults"])
    else:
        values.update({key: value for key, value in raw.items() if key not in ("config_type", "version")})
    app = raw.get("app")
    if isinstance(app, Mapping):
        values.update({key: value for key, value in app.items() if key != "name"})
        values["app"] = dict(app)
    return values


# --------------------------------------------------------------------------- #
# McpApp
# --------------------------------------------------------------------------- #

class McpApp(AppLifecycle):
    """MCP app: exposes all configured server tools through a FastMCP server.

    Config dict keys:
        server_name: MCP server name (default: "rynnrcp_mcp")
        host: Bind host (default: "0.0.0.0")
        port: Bind port (default: 28403)
        path: URL path (default: "/mcp")
        transport: MCP transport type (default: "streamable-http")
    """

    VERSION = "0.1.0"
    DESCRIPTION = "Exposes server tools via MCP protocol for AI Agent access"
    AUTHOR = "RynnRCP Team"

    def __init__(self, name: str = "mcp", config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(name, _merge_default_config(config))

        self._server_name = str(self.config["server_name"])
        self._host = str(self.config["host"])
        self._port = int(self.config["port"])
        self._path = str(self.config["path"])
        self._transport_type = str(self.config["transport"])

        self._manifest: ServerManifest | None = None
        self._server_client: RcpProtocolClient | None = None
        self._mcp_app: Optional[FastMCP] = None

    def connect_server(self) -> None:
        """Resolve the configured RynnRCP robot and create a protocol client."""
        if self._server_client is None:
            target = _target_robot_config(self.config)
            request_timeout_ms = int(target.get("request_timeout_ms") or 1000)
            interface = ClientInterface.with_defaults(local_registry=True, mdns=True)
            self._manifest = resolve_server_manifest(
                robot_id=str(target["robot_id"]),
                embodiment_type=str(target.get("embodiment_type") or "") or None,
                interface=interface,
                request_timeout_ms=request_timeout_ms,
            )
            if self._manifest.endpoint is None:
                raise RuntimeError(f"robot_id {self._manifest.robot_id!r} has no endpoint")
            self._server_client = RcpProtocolClient(interface.connect(self._manifest.endpoint))
        self._logger.info("Connected MCP app to robot_id=%s", self._manifest.robot_id if self._manifest else "")

    def start(self) -> None:
        """Build and start the FastMCP server."""
        if self._running:
            return
        if self._server_client is None:
            self.connect_server()

        self._mcp_app = self._build_mcp_app()
        self._mark_started()

        # Run the MCP server (this typically blocks)
        self._mcp_app.run(
            transport=self._transport_type,
            host=self._host,
            port=self._port,
            path=self._path,
            show_banner=False,
        )

    def stop(self) -> None:
        """Stop the MCP server."""
        self._mark_stopped()
        # FastMCP doesn't expose a clean shutdown API yet
        self._mcp_app = None
        if self._server_client is not None and hasattr(self._server_client, "close"):
            self._server_client.close()
            self._server_client = None
        self._logger.info("Stopped")

    # ─── Convenience API ─────────────────────────────────────────────

    def list_tools(self) -> list:
        """List available tool names."""
        if self._server_client is None:
            return []
        return list(self._list_server_tools().keys())

    def call_tool(self, name: str, params: Optional[Dict[str, Any]] = None) -> Any:
        """Call a tool directly (bypass MCP protocol)."""
        if self._server_client is None:
            raise RuntimeError("server_client not available")
        response = self._server_client.request(name, params or {})
        if response.ok:
            return response.payload
        return {"success": False, "message": response.message, "result": None}

    # ─── Private ─────────────────────────────────────────────────────

    def _build_mcp_app(self) -> FastMCP:
        """Build a FastMCP app with all server tools registered."""
        app = FastMCP(self._server_name)

        tools_meta = self._list_server_tools()

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
                async def _tool(ctx: Context, params: Optional[Dict[str, Any]] = None):
                    params = params or {}
                    try:
                        loop = asyncio.get_running_loop()
                        result = await loop.run_in_executor(
                            None,
                            lambda: self.call_tool(name, params),
                        )
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

    def _list_server_tools(self) -> Dict[str, Dict[str, Any]]:
        if self._server_client is None:
            return {}
        response = self._server_client.list_tools()
        if not response.ok or not isinstance(response.payload, dict):
            raise RuntimeError(response.message or "list_tools failed")
        return dict(response.payload)

    def health_check(self) -> Dict[str, Any]:
        base = super().health_check()
        base.update({
            "server_name": self._server_name,
            "host": self._host,
            "port": self._port,
            "path": self._path,
            "transport": self._transport_type,
            "tools_count": len(self.list_tools()) if self._server_client else 0,
        })
        return base


def _target_robot_config(config: Mapping[str, Any]) -> Dict[str, Any]:
    target = config.get("target_robot")
    if not isinstance(target, Mapping):
        raise ValueError("target_robot must be configured")
    robot_id = str(target.get("robot_id") or "").strip()
    if not robot_id:
        raise ValueError("target_robot.robot_id is required")
    return dict(target)


def _bind_target_robot_config(config: Mapping[str, Any], server_config: str) -> Dict[str, Any]:
    merged = dict(config)
    raw = _load_yaml(server_config)
    manifest = raw.get("manifest") if isinstance(raw, Mapping) else None
    if not isinstance(manifest, Mapping) or not str(manifest.get("robot_id") or "").strip():
        raise ValueError(f"{server_config} must define manifest.robot_id")
    target = dict(merged.get("target_robot") or {})
    target["robot_id"] = str(manifest["robot_id"])
    merged["target_robot"] = target
    return merged


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Run the RynnRCP MCP app.")
    parser.add_argument("--server-config", required=True, help="RynnRCP server YAML config used to select target server.")
    parser.add_argument("--config", help="Optional MCP app YAML config path.")
    args = parser.parse_args(argv)
    config = _merge_default_config(_load_yaml(args.config) if args.config else {})
    config = _bind_target_robot_config(config, args.server_config)
    configure_logging(
        level=logging.INFO,
        sinks=["stderr", "file"],
        file_path=str(log_file_from_config(config, "mcp.log")),
    )
    app = McpApp(config=config)
    try:
        app.start()
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"Failed to start MCP app: {exc}", file=sys.stderr)
        return 2
    finally:
        app.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
