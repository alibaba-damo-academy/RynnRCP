"""Browser URL helpers for local web UIs."""

from __future__ import annotations

import socket


WILDCARD_HOSTS = {"", "0.0.0.0", "::"}


def browser_urls(host: str, port: int) -> list[str]:
    """Return user-openable URLs for a bind host and port."""
    host = str(host or "")
    port = int(port)
    if host not in WILDCARD_HOSTS:
        return [f"http://{host}:{port}/"]
    urls = [f"http://127.0.0.1:{port}/"]
    urls.extend(f"http://{ip}:{port}/" for ip in find_lan_ips())
    return urls


def primary_browser_url(host: str, port: int) -> str:
    return browser_urls(host, port)[0]


def find_lan_ips() -> list[str]:
    ips = {_probe_default_route_ip(), *_hostname_ips()}
    return sorted(ip for ip in ips if ip and not ip.startswith("127."))


def _probe_default_route_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            return str(sock.getsockname()[0])
    except OSError:
        return ""


def _hostname_ips() -> list[str]:
    try:
        return [item[4][0] for item in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)]
    except OSError:
        return []
