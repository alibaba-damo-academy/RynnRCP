# Copyright 2026 RynnRCP Authors. All rights reserved.
# Platform abstraction: lightweight auto-reset event notification.

from __future__ import annotations

import threading
from enum import Enum, auto


class WaitResult(Enum):
    """Result of an event wait operation."""

    SIGNALED = auto()
    TIMEOUT = auto()
    ERROR = auto()


class Event:
    """Intra-process auto-reset event based on ``threading.Event``."""

    def __init__(self) -> None:
        self._event = threading.Event()
        self._closed = False

    def is_valid(self) -> bool:
        return not self._closed

    def signal(self) -> bool:
        if self._closed:
            return False
        self._event.set()
        return True

    def wait(self, timeout_ms: int = -1) -> WaitResult:
        if self._closed:
            return WaitResult.ERROR

        timeout = None if timeout_ms < 0 else timeout_ms / 1000.0
        if timeout_ms == 0:
            signaled = self._event.is_set()
        else:
            signaled = self._event.wait(timeout=timeout)

        if signaled:
            self._event.clear()
            return WaitResult.SIGNALED
        return WaitResult.TIMEOUT

    def reset(self) -> None:
        self._event.clear()

    def close(self) -> None:
        self._closed = True
        self._event.set()
