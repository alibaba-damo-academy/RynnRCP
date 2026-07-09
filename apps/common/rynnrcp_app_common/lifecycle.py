"""Small lifecycle helper for RynnRCP apps."""

from __future__ import annotations

import logging
import time
from typing import Any, Dict, Optional


class AppLifecycle:
    """Minimal lifecycle state shared by app packages."""

    VERSION = "0.0.0"
    DESCRIPTION = ""
    AUTHOR = ""

    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None) -> None:
        self.name = name
        self.config = config or {}
        self._running = False
        self._start_time: Optional[float] = None
        self._logger = logging.getLogger(f"rynnrcp.app.{name}")

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def logger(self) -> logging.Logger:
        return self._logger

    def health_check(self) -> Dict[str, Any]:
        uptime = None
        if self._running and self._start_time is not None:
            uptime = time.time() - self._start_time
        return {
            "name": self.name,
            "running": self._running,
            "uptime_s": uptime,
        }

    def _mark_started(self) -> None:
        self._running = True
        self._start_time = time.time()

    def _mark_stopped(self) -> None:
        self._running = False
        self._start_time = None
