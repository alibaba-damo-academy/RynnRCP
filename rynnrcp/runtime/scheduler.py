# Copyright 2026 RynnRCP Authors. All rights reserved.
# Core layer: Scheduler - periodic component execution.

from __future__ import annotations

import threading
import time
from typing import Callable, Dict, List, Optional

import logging

logger = logging.getLogger(__name__)
ERROR_LOG_INTERVAL_S = 5.0


class Component:
    """A schedulable unit with a periodic callback.

    Args:
        name: Unique name for this component.
        period_ms: Desired execution period in milliseconds.
        callback: Function to call at each period.
        priority: Higher value = higher priority (default 0).
    """

    def __init__(self, name: str, period_ms: float, callback: Callable[[], None],
                 priority: int = 0) -> None:
        assert period_ms > 0, f"period_ms must be > 0, got {period_ms}"
        self.name = name
        self.period_ms = period_ms
        self.callback = callback
        self.priority = priority
        self._enabled = True

    @property
    def enabled(self) -> bool:
        return self._enabled

    @enabled.setter
    def enabled(self, value: bool) -> None:
        self._enabled = value


class _ComponentRunner:
    """Runs a single component in its own thread at fixed frequency."""

    def __init__(self, component: Component) -> None:
        self._comp = component
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_error_key: tuple[str, str] | None = None
        self._last_error_logged_at = 0.0
        self._suppressed_error_count = 0

    def start(self) -> None:
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run_loop,
            name=f"sched_{self._comp.name}",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> bool:
        self._stop_event.set()
        if self._thread is not None:
            timeout_s = max(self._comp.period_ms / 1000.0 * 3, 0.1)
            self._thread.join(timeout=timeout_s)
            if self._thread.is_alive():
                logger.warning(
                    "Component '%s' did not stop within %.3fs",
                    self._comp.name,
                    timeout_s,
                )
                return False
            self._thread = None
        return True

    def _run_loop(self) -> None:
        period_sec = self._comp.period_ms / 1000.0
        next_time = time.monotonic()

        while not self._stop_event.is_set():
            next_time += period_sec

            if self._comp.enabled:
                try:
                    self._comp.callback()
                    self._report_recovery()
                except Exception as e:
                    self._report_error(e)
                actual_end = time.monotonic()
                if actual_end > next_time:
                    missed = int((actual_end - next_time) // period_sec)
                    next_time += missed * period_sec

            # Sleep until next period
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0:
                self._stop_event.wait(timeout=sleep_time)

    def _report_error(self, error: Exception) -> None:
        now = time.monotonic()
        error_key = (type(error).__name__, str(error))
        if error_key != self._last_error_key:
            self._last_error_key = error_key
            self._last_error_logged_at = now
            self._suppressed_error_count = 0
            logger.warning(
                "Component '%s' error: %s",
                self._comp.name,
                error,
                exc_info=True,
            )
            return

        self._suppressed_error_count += 1
        if now - self._last_error_logged_at < ERROR_LOG_INTERVAL_S:
            return
        logger.warning(
            "Component '%s' still failing: %s "
            "(suppressed %d repeated errors)",
            self._comp.name,
            error,
            self._suppressed_error_count,
        )
        self._last_error_logged_at = now
        self._suppressed_error_count = 0

    def _report_recovery(self) -> None:
        if self._last_error_key is None:
            return
        error_type, error_message = self._last_error_key
        logger.info(
            "Component '%s' recovered after %s: %s",
            self._comp.name,
            error_type,
            error_message,
        )
        self._last_error_key = None
        self._last_error_logged_at = 0.0
        self._suppressed_error_count = 0


class Scheduler:
    """Manages periodic execution of multiple components.

    Each component runs in its own thread at its specified frequency.
    """

    def __init__(self) -> None:
        self._components: Dict[str, Component] = {}
        self._runners: Dict[str, _ComponentRunner] = {}
        self._running = False
        self._lock = threading.Lock()

    def add_component(self, component: Component) -> None:
        """Register a component for scheduling.

        Can be called before or after start(). If already running,
        the component starts immediately.
        """
        with self._lock:
            if component.name in self._components:
                raise ValueError(f"Component '{component.name}' already registered")
            self._components[component.name] = component

            if self._running:
                runner = _ComponentRunner(component)
                self._runners[component.name] = runner
                runner.start()

    def remove_component(self, name: str) -> None:
        """Remove and stop a component."""
        runner: Optional[_ComponentRunner] = None
        with self._lock:
            if name in self._runners:
                runner = self._runners.pop(name)
            if name in self._components:
                del self._components[name]
        if runner is not None:
            runner.stop()

    def start(self) -> None:
        """Start all registered components."""
        with self._lock:
            if self._running:
                return
            self._running = True
            # Sort by priority (higher first)
            sorted_comps = sorted(
                self._components.values(),
                key=lambda c: c.priority,
                reverse=True,
            )
            for comp in sorted_comps:
                runner = _ComponentRunner(comp)
                self._runners[comp.name] = runner
                runner.start()

    def stop(self) -> None:
        """Stop all components gracefully."""
        with self._lock:
            if not self._running:
                return
            self._running = False
            runners = list(self._runners.values())
            self._runners.clear()
        for runner in runners:
            runner.stop()

    def spin_once(self) -> None:
        """Manually trigger one execution of all enabled components.

        Useful for testing or single-step debugging.
        """
        # Sort by priority
        sorted_comps = sorted(
            self._components.values(),
            key=lambda c: c.priority,
            reverse=True,
        )
        for comp in sorted_comps:
            if comp.enabled:
                try:
                    comp.callback()
                except Exception as e:
                    logger.warning("Component '%s' error: %s", comp.name, e, exc_info=True)

    def list_components(self) -> List[str]:
        """List all registered component names."""
        with self._lock:
            return list(self._components.keys())

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._running

    @property
    def component_count(self) -> int:
        with self._lock:
            return len(self._components)
