# rynnrcp_app_rynnbot/auth.py

"""
Authentication helper for the Rynnbot communication app.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module implements :class:`~rynnrcp_app_rynnbot.auth.RynnAuthClient`,
a small HTTP client used to fetch connection credentials/configuration for
Rynnbot backends (e.g. MQTT and WebSocket).

The client builds a signed authentication payload (HMAC-SHA1 + base64) using the
device secret, sends it to the configured HTTP endpoint, and returns the
backend-specific fields required by the transport layer.

Note:
    The internal request helper retries indefinitely on failure and logs errors
    via the project server logger.
"""

from __future__ import annotations

import hmac
import time
import base64
import requests
from typing import Any, Dict, Optional
from urllib.parse import urlparse
import logging

logger = logging.getLogger(__name__)


class RynnAuthClient:

    def __init__(
        self,
        http_url: str,
        product_key: str,
        device_name: str,
        device_secret: str,
        *,
        request_timeout_s: float = 5.0,
        max_attempts: Optional[int] = None,
    ) -> None:
        """Initialize the authentication client."""
        self.http_url = http_url.rstrip("/")
        self.product_key = product_key
        self.device_name = device_name
        self.device_secret = device_secret
        self.request_timeout_s = float(request_timeout_s)
        self.max_attempts = max_attempts

    def _build_auth_payload(self) -> Dict[str, Any]:
        """Build the request body for the authentication request."""
        ts = str(int(time.time() * 1000))
        params = {
            "nonce": 1,
            "authKey": self.product_key,
            "timestamp": ts,
            "productKey": self.product_key,
            "deviceName": self.device_name,
        }
        sorted_items = sorted(params.items())
        sign_content = ";".join(f"{k}={v}" for k, v in sorted_items)

        mac = hmac.new(
            self.device_secret.encode("utf-8"),
            sign_content.encode("utf-8"),
            digestmod="sha1",
        )
        signature = base64.b64encode(mac.digest()).decode("utf-8")

        return {
            "productKey": self.product_key,
            "deviceName": self.device_name,
            "auth": {
                "authKey": self.product_key,
                "nonce": 1,
                "timestamp": ts,
                "sign": signature,
            },
        }

    def _post_with_retry(self, url: str, what: str) -> Dict[str, Any]:
        """Send an authentication request to the given URL."""
        attempts = 0
        while True:
            attempts += 1
            try:
                payload = self._build_auth_payload()
                resp = requests.post(url, json=payload, timeout=self.request_timeout_s)

                if resp.status_code != 200:
                    raise RuntimeError(f"error with code {resp.status_code}")

                result = resp.json()
                if not result.get("success", False):
                    raise RuntimeError(f"{result.get('message')} check device config")

                return result["data"]
            except Exception as e:
                if self.max_attempts is not None and attempts >= self.max_attempts:
                    raise RuntimeError(
                        f"[AUTH] Get auth {what} token failed after {attempts} attempts: {e}"
                    ) from e
                logger.error("[AUTH] Get auth %s token failed: %s, retry...", what, e, exc_info=True)
                time.sleep(1)

    def get_mqtt_config(self, endpoint_mqtt: str, mqtt_port: int) -> Dict[str, Any]:
        """Get the configuration required to connect to the MQTT broker."""
        url = f"{self.http_url}{endpoint_mqtt}"
        cfg = self._post_with_retry(url, "MQTT")

        return {
            "mqtt_host": cfg["endpoint"],
            "client_id": cfg["clientId"],
            "device_ak": cfg["accessKeyId"],
            "device_sk": cfg["accessKeySecret"],
            "instance_id": cfg["instanceId"],
            "mqtt_port": mqtt_port,
        }

    def get_ws_config(self, endpoint_websocket: str) -> Dict[str, Any]:
        """Get the configuration required to establish a WebSocket connection."""
        url = f"{self.http_url}{endpoint_websocket}"
        cfg = self._post_with_retry(url, "WS")

        token = cfg["token"]
        expire = int(cfg["expire"])
        uri = cfg["uri"]
        parsed = urlparse(uri)
        host = parsed.hostname
        path = parsed.path
        if parsed.port is not None:
            port = parsed.port
        else:
            port = 443 if parsed.scheme == "wss" else 80

        return {
            "token": token,
            "expire": expire,
            "websocket_host": host,
            "websocket_path": path,
            "websocket_port": port,
        }
