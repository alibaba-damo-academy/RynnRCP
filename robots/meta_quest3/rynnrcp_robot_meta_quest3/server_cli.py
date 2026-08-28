"""Start a Meta Quest 3 leader Server with runtime network parameters."""

from __future__ import annotations

import argparse
import ipaddress
import logging
from pathlib import Path
import sys
from typing import Any

from rynnrcp.config.loader import load_config
from rynnrcp.interface.mdns import find_lan_ips
from rynnrcp.server import RynnRCPServer
from rynnrcp.utils.logging import configure_server_logging
from rynnrcp.utils.user_paths import new_log_session_id


def load_config_with_source_ip(path: str, source_ip: str) -> dict[str, Any]:
    """Load one Quest Server YAML and override its accepted UDP source address."""
    try:
        normalized_ip = str(ipaddress.IPv4Address(str(source_ip).strip()))
    except ipaddress.AddressValueError as exc:
        raise ValueError(f"Quest source IP is not a valid IPv4 address: {source_ip}") from exc

    config = load_config(path)
    components = config.get("components")
    robot = components.get("robot") if isinstance(components, dict) else None
    if not isinstance(robot, dict):
        raise ValueError("Quest Server config requires components.robot")
    robot["source_ip"] = normalized_ip
    return config


def _run_server(config: dict[str, Any], config_label: str) -> int:
    """Run the configured leader without requiring a RynnRCP CLI extension."""
    log_session_id = new_log_session_id()
    configure_server_logging(
        config,
        level=logging.INFO,
        session_id=log_session_id,
        process_name="quest3_server",
    )
    server = RynnRCPServer(config, log_session_id=log_session_id)
    try:
        server.start()
    except Exception as exc:
        logging.exception("Failed to start Meta Quest 3 leader server")
        print(f"Failed to start Meta Quest 3 leader server: {exc}", file=sys.stderr)
        return 2

    try:
        print("RynnRCP server is running.")
        print(f"Config: {config_label}")
        print(f"Instance: {server.server_instance_id}")
        print(f"Log session: {log_session_id}")
        print(f"Local gRPC: 127.0.0.1:{server.bound_port}")
        for ip in find_lan_ips():
            print(f"LAN gRPC:   {ip}:{server.bound_port}")
        if server.visualization_urls:
            print(f"Debug UI Local: {server.visualization_urls[0]}")
            for url in server.visualization_urls[1:]:
                print(f"Debug UI LAN:   {url}")
        print("Press Ctrl+C to stop.")
        server.wait_for_termination()
    except KeyboardInterrupt:
        print("\nStopping RynnRCP server.")
        return 130
    finally:
        server.stop()
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run a Meta Quest 3 leader Server with an explicit UDP source IP."
    )
    parser.add_argument("--config", required=True, help="Meta Quest 3 Server YAML path.")
    parser.add_argument(
        "--source-ip",
        required=True,
        help="IPv4 address that sends Quest controller UDP packets.",
    )
    args = parser.parse_args(argv)

    config_path = str(Path(args.config).expanduser())
    config = load_config_with_source_ip(config_path, args.source_ip)
    normalized_ip = config["components"]["robot"]["source_ip"]
    print(f"Quest UDP source IP: {normalized_ip}")
    return _run_server(
        config,
        f"{config_path} (source_ip={normalized_ip})",
    )


if __name__ == "__main__":
    raise SystemExit(main())
