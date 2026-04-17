# rcp_core/common/bus/progress.py

"""
Generic progress tracking utilities for long-running tool calls.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module provides :class:`ProgressStage` and :class:`ProgressTracker`,
a lightweight multi-stage progress calculator that is completely decoupled
from any specific business logic.

Usage pattern
-------------
1. The *tool* (e.g. DataServer) defines its own stage list and creates a
   ``ProgressTracker`` with an optional callback supplied by the caller.
2. The *caller* (e.g. McpPlugin, TeleopPlugin) supplies a callable
   ``(current: float, total: float, message: str) -> None`` and wires it
   to whatever transport it uses (MCP progress notification, WebSocket, …).
3. :class:`RcpBus` detects whether a handler accepts ``_progress_callback``
   via :func:`inspect.signature` and passes it through transparently—no
   hardcoded tool names anywhere.

This module has **no imports from business modules** and defines **no
business-specific stage configurations**.  Stage definitions belong in the
module that owns the business logic (e.g. ``data_server.py``).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional


@dataclass
class ProgressStage:
    """Description of one named stage in a multi-stage operation.

    :param name: Unique stage identifier used in :meth:`ProgressTracker.enter_stage`.
    :param weight: Fraction of the total operation this stage represents (0 < weight ≤ 1).
                   All weights across all stages should sum to 1.0.
    :param description: Human-readable label sent in progress messages.
    """

    name: str
    weight: float
    description: str


# Type alias for the progress callback signature expected by tools and adapters.
ProgressCallback = Callable[[float, float, str], None]


class ProgressTracker:
    """Multi-stage progress calculator with an optional callback.

    Tracks which stages have completed, computes an overall percentage in
    the range [0, 100], and fires *callback* on every update.

    :param stages: Ordered list of :class:`ProgressStage` objects that define
                   the structure of the operation.
    :param callback: Optional callable ``(current, total, message) -> None``.
                     *current* and *total* are both floats; *total* is always
                     ``100.0`` so callers can treat the pair as a percentage.
                     If ``None``, all calls are no-ops (zero overhead).

    Example::

        STAGES = [
            ProgressStage("fetch",  0.2, "Fetching data"),
            ProgressStage("encode", 0.7, "Encoding"),
            ProgressStage("upload", 0.1, "Uploading"),
        ]

        def my_tool(data_id: str, _progress_callback=None):
            tracker = ProgressTracker(STAGES, _progress_callback)

            tracker.enter_stage("fetch")
            items = fetch(data_id)

            tracker.enter_stage("encode")
            for i, item in enumerate(items):
                encode(item)
                tracker.report(i + 1, len(items), f"Encoded {item.name}")

            tracker.enter_stage("upload")
            upload(items)

            tracker.complete("Done")
    """

    def __init__(
        self,
        stages: list[ProgressStage],
        callback: Optional[ProgressCallback] = None,
    ) -> None:
        self._stages: dict[str, ProgressStage] = {s.name: s for s in stages}
        self._callback = callback
        self._current_stage: Optional[str] = None
        self._stage_progress: float = 0.0   # fraction within current stage [0, 1]
        self._completed_weight: float = 0.0 # sum of weights for finished stages

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def enter_stage(self, stage_name: str, message: Optional[str] = None) -> None:
        """Advance to *stage_name*, finalising the previous stage.

        :param stage_name: Must match a name in the stages list passed to ``__init__``.
        :param message: Optional override for the notification message; defaults to
                        the stage's ``description``.
        """
        # Finalise previous stage
        if self._current_stage and self._current_stage in self._stages:
            self._completed_weight += self._stages[self._current_stage].weight

        self._current_stage = stage_name
        self._stage_progress = 0.0

        if self._callback:
            stage = self._stages.get(stage_name)
            text = message or (stage.description if stage else stage_name)
            self._callback(self._total_progress(), 100.0, text)

    def report(
        self,
        current: float,
        total: float,
        message: Optional[str] = None,
    ) -> None:
        """Report sub-progress within the current stage.

        :param current: How many units have been completed so far.
        :param total: Total units in this stage (0 is treated as complete).
        :param message: Human-readable description of the current step.
        """
        self._stage_progress = (current / total) if total > 0 else 1.0
        self._stage_progress = min(self._stage_progress, 1.0)

        if self._callback:
            stage = self._stages.get(self._current_stage or "")
            prefix = f"[{stage.description}] " if stage else ""
            text = f"{prefix}{message}" if message else (stage.description if stage else "")
            self._callback(self._total_progress(), 100.0, text)

    def complete(self, message: str = "Done") -> None:
        """Mark the entire operation as 100 % complete.

        :param message: Final message sent to the callback.
        """
        self._completed_weight = 1.0
        self._stage_progress = 1.0
        if self._callback:
            self._callback(100.0, 100.0, message)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _total_progress(self) -> float:
        """Return overall progress as a float in [0, 100]."""
        stage_contrib = 0.0
        if self._current_stage and self._current_stage in self._stages:
            stage_contrib = (
                self._stages[self._current_stage].weight * self._stage_progress
            )
        return min((self._completed_weight + stage_contrib) * 100.0, 100.0)
