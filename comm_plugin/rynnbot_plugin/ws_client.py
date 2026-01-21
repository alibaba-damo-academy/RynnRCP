# comm_plugin/rynnbot_plugin/ws_client.py

"""
WebSocket transport client for the Rynnbot plugin.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module provides :class:`~comm_plugin.rynnbot_plugin.ws_client.RynnWebsocketClient`,
a wrapper around ``websocket-client`` that connects to the Rynnbot WebSocket gateway
using an access token obtained from :class:`~comm_plugin.rynnbot_plugin.auth.RynnAuthClient`.

The client runs the WebSocket event loop in a background thread, forwards inbound
binary frames to an optional callback, and supports basic reconnection behavior.
When the server closes the connection with the token-expired code, the token is
refreshed before reconnecting.
"""

from __future__ import annotations

from typing import Optional
import ssl
import threading
import time

import websocket

from .auth import RynnAuthClient
from rcp_core.common.utils.logger import server_logger

logger = server_logger()


class RynnWebsocketClient:
    K_TOKEN_EXPIRED_CODE = 4001

    def __init__(
        self,
        auth: RynnAuthClient,
        endpoint_websocket: str,
        on_message_callback=None,
    ) -> None:
        """Initialize the WebSocket client wrapper."""
        self.auth = auth
        self.endpoint_websocket = endpoint_websocket

        self._token = ""
        self._expire = 0
        self._ws_host = ""
        self._ws_path = ""
        self._ws_port = 443

        self._ws: Optional[websocket.WebSocketApp] = None
        self._conn_flag = "INIT"  # INIT / CONNECTED / DISCONNECTED / ERROR

        self._ws_thread: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._lock = threading.Lock()
        self._on_message_callback = on_message_callback

    def set_message_callback(self, cb) -> None:
        """Set the callback function to be invoked when a WebSocket message is received."""
        self._on_message_callback = cb

    def _load_ws_config(self) -> None:
        """Use the auth client to load/refresh WebSocket connection configuration and token."""
        cfg = self.auth.get_ws_config(self.endpoint_websocket)
        self._token = cfg["token"]
        self._expire = cfg["expire"]
        self._ws_host = cfg["websocket_host"]
        self._ws_path = cfg["websocket_path"]
        self._ws_port = cfg["websocket_port"]

    def _refresh_token(self) -> None:
        """Refresh the token."""
        logger.info("[WS] Refreshing token...")

        self._load_ws_config()

    def _on_open(self, ws) -> None:
        """Callback when the WebSocket connection is successfully opened."""
        logger.info("[WS] Connected")
        with self._lock:
            self._conn_flag = "CONNECTED"

    def _on_message(self, ws, message) -> None:
        """Callback when a WebSocket message is received."""
        if isinstance(message, bytes):
            payload = message
            logger.info(f"[WS] Binary message, size={len(message)}")

            if self._on_message_callback:
                self._on_message_callback(
                    {"channel": "ws", "topic": None, "payload": payload, "raw": message}
                )

    def _on_error(self, ws, error) -> None:
        """Callback when a WebSocket error occurs."""
        logger.error(f"[WS] Error: {error}")
        with self._lock:
            self._conn_flag = "ERROR"

    def _on_close(self, ws, code, msg) -> None:
        """Callback when the WebSocket connection is closed."""
        logger.info(f"[WS] Closed: code={code}, msg={msg}")
        with self._lock:
            self._conn_flag = "DISCONNECTED"
        if code == self.K_TOKEN_EXPIRED_CODE:
            logger.info("[WS] Token expired")
            self._refresh_token()

    def _build_url(self) -> str:
        """Construct the WebSocket connection URL."""
        scheme = "wss" if self._ws_port == 443 else "ws"
        return f"{scheme}://{self._ws_host}:{self._ws_port}{self._ws_path}"

    def _create_ws_app(self) -> websocket.WebSocketApp:
        """Create a WebSocketApp instance and set request headers and callbacks."""
        headers = {"tunnel-access-token": self._token}
        return websocket.WebSocketApp(
            self._build_url(),
            header=[f"{k}: {v}" for k, v in headers.items()],
            on_open=self._on_open,
            on_message=self._on_message,
            on_error=self._on_error,
            on_close=self._on_close,
        )

    def _ws_run_forever(self) -> None:
        """Run the WebSocket loop in a separate thread and handle automatic reconnection."""
        self._running.set()
        while self._running.is_set():
            try:
                self._ws = self._create_ws_app()
                logger.info(f"[WS] Connecting to {self._build_url()} ...")
                self._ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})
            except Exception as e:
                logger.error(f"[WS] run_forever error: {e}")

            if not self._running.is_set():
                break

            with self._lock:
                flag = self._conn_flag
            if flag in ("ERROR", "DISCONNECTED"):
                logger.info("[WS] Reconnect in 1s...")
                time.sleep(1)

    def start(self) -> None:
        """Start the WebSocket client."""
        self._load_ws_config()
        self._ws_thread = threading.Thread(target=self._ws_run_forever, daemon=True)
        self._ws_thread.start()

    def send_bytes(self, data: bytes) -> None:
        """Send binary data over the WebSocket connection."""
        if not self._ws:
            logger.warning("[WS] not connected, drop message")
            return
        try:
            self._ws.send(data, opcode=websocket.ABNF.OPCODE_BINARY)
        except Exception as e:
            logger.warning("[WS] send_bytes error:", e)

    def close(self) -> None:
        """Close the WebSocket client."""
        self._running.clear()
        if self._ws:
            try:
                self._ws.close()
            except Exception:
                pass
        if self._ws_thread and self._ws_thread.is_alive():
            self._ws_thread.join(timeout=2)
