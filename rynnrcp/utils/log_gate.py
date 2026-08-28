"""Rate-limited logging for high-frequency failure sources.

Loops that touch hardware, sockets, or IPC can fail hundreds of times per
second. ``LogGate`` implements the standard pattern for those call sites:

- first failure: logged immediately at the configured level
- ongoing failures: one periodic summary per ``interval_s`` with a count
- recovery: one ``info`` line with the outage duration and total failures
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any


class LogGate:
    """Deduplicate and rate-limit repeated failure logs for one source.

    One gate represents one independently recoverable source.
    """

    def __init__(
        self,
        logger: logging.Logger,
        source: str,
        *,
        interval_s: float = 5.0,
        level: int = logging.ERROR,
    ) -> None:
        self._logger = logger
        self._source = str(source)
        self._interval_s = float(interval_s)
        self._level = int(level)
        self._failing_since: float | None = None
        self._failure_count = 0
        self._last_emit_at = 0.0
        self._lock = threading.Lock()

    @property
    def failing(self) -> bool:
        with self._lock:
            return self._failing_since is not None

    @property
    def failure_count(self) -> int:
        with self._lock:
            return self._failure_count

    def failure(self, message: str, *args: Any, exc_info: Any = None) -> None:
        """Record one failure; emits immediately on first, then summarizes."""
        now = time.monotonic()
        emit_first = False
        summary: tuple[float, int] | None = None
        with self._lock:
            self._failure_count += 1
            if self._failing_since is None:
                self._failing_since = now
                self._last_emit_at = now
                emit_first = True
            elif now - self._last_emit_at >= self._interval_s:
                self._last_emit_at = now
                summary = (now - self._failing_since, self._failure_count)

        if emit_first:
            self._logger.log(
                self._level,
                f"[{self._source}] {message}",
                *args,
                exc_info=exc_info,
            )
            return
        if summary is not None:
            self._logger.log(
                self._level,
                f"[{self._source}] still failing after %.1fs (%d failures): {message}",
                summary[0],
                summary[1],
                *args,
            )

    def success(self) -> None:
        """Record one success; emits a recovery notice if it ends an outage."""
        with self._lock:
            if self._failing_since is None:
                return
            duration = time.monotonic() - self._failing_since
            count = self._failure_count
            self._failing_since = None
            self._failure_count = 0
        self._logger.info(
            "[%s] recovered after %.1fs (%d failures)",
            self._source,
            duration,
            count,
        )
