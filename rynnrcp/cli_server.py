"""CLI for running a RynnRCP server."""

from __future__ import annotations

import argparse
import logging
import sys

from rynnrcp.interface.mdns import find_lan_ips
from rynnrcp.server import RynnRCPServer
from rynnrcp.utils.logging import configure_server_logging
from rynnrcp.utils.user_paths import new_log_session_id


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run a RynnRCP server.")
    parser.add_argument("--config", required=True, help="Server YAML config path.")
    args = parser.parse_args(argv)
    log_session_id = new_log_session_id()
    configure_server_logging(
        args.config,
        level=logging.INFO,
        session_id=log_session_id,
        process_name="server",
    )

    server = RynnRCPServer(args.config, log_session_id=log_session_id)
    try:
        server.start()
    except Exception as exc:
        logging.exception("Failed to start RynnRCP server")
        print(f"Failed to start RynnRCP server: {exc}", file=sys.stderr)
        return 2

    try:
        print("RynnRCP server is running.")
        print(f"Config: {args.config}")
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


if __name__ == "__main__":
    raise SystemExit(main())
