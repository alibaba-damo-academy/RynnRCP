"""LCM connector for RCP protocol JSON messages."""

from __future__ import annotations

import json
import logging
import threading
import time
from typing import Any, Callable, Dict

from .base import BaseConnector

logger = logging.getLogger(__name__)
_THREAD_JOIN_TIMEOUT_S = 1.0

try:
    import lcm as _lcm_lib

    LCM_AVAILABLE = True
except ImportError:
    LCM_AVAILABLE = False


class LCMConnector(BaseConnector):
    """LCM protocol connector."""

    def __init__(self, url: str | None = None) -> None:
        if not LCM_AVAILABLE:
            raise RuntimeError(
                "lcm is not installed – cannot create LCMConnector"
            )

        self.url = url
        self.lcm_instance = _lcm_lib.LCM(url) if url else _lcm_lib.LCM()
        self._running = True
        self._last_loop_error_log_at = 0.0
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while self._running:
            try:
                self.lcm_instance.handle_timeout(10)
            except Exception as e:
                now = time.monotonic()
                if now - self._last_loop_error_log_at >= 2.0:
                    logger.error(
                        "LCMConnector loop error (url=%s): %s",
                        self.url or "default",
                        e,
                        exc_info=True,
                    )
                    self._last_loop_error_log_at = now
                time.sleep(0.01)

    def pub(self, params: Dict[str, Any], msg: Any) -> None:
        topic = params.get("topic")
        if not topic:
            raise ValueError("params.topic is required")
        if not isinstance(msg, (dict, list, tuple)):
            raise TypeError("LCMConnector pub expects protocol JSON data")
        payload = json.dumps(msg, ensure_ascii=False).encode("utf-8")
        self.lcm_instance.publish(topic, payload)

    def sub(self, params: Dict[str, Any], callback: Callable) -> None:
        topic = params.get("topic")
        if not topic:
            raise ValueError("params.topic is required")
        msg_type = str(params.get("msg_type") or "json").strip().lower()
        if msg_type != "json":
            raise ValueError("LCMConnector only supports msg_type=json")

        self.lcm_instance.subscribe(
            topic,
            lambda channel, data: callback(json.loads(data.decode("utf-8"))),
        )
        logger.info("LCMConnector subscribed to %s (json)", topic)

    def stop(self) -> None:
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=_THREAD_JOIN_TIMEOUT_S)
            if self._thread.is_alive():
                logger.warning(
                    "LCMConnector thread did not stop within %.1fs",
                    _THREAD_JOIN_TIMEOUT_S,
                )
        logger.info("LCMConnector stopped")
