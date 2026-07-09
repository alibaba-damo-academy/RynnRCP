# rynnrcp_app_rynnbot/auth.py

"""
Bounded task submission utilities for the Rynnbot app.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rynnrcp_app_rynnbot.auth.RynnBoundedExecutor`,
a thin wrapper around :class:`concurrent.futures.ThreadPoolExecutor` that limits
how many tasks can be in-flight (running + queued) at the same time.

If the in-flight limit is reached, :meth:`RynnBoundedExecutor.submit` refuses the
submission and returns ``None`` instead of blocking. The wrapper also schedules a
background timeout watcher per task: if a task does not complete within the given
timeout, it is logged via the project logger (the underlying task is not cancelled).
"""

import concurrent.futures
import threading
from typing import Optional, Callable
import logging

logger = logging.getLogger(__name__)


class RynnBoundedExecutor:
    """
    Limit the number of in-flight tasks submitted to an executor (running + queued).
    If full, submit() returns None. Add timeout functionality for task completion.
    """

    def __init__(
        self, executor: concurrent.futures.ThreadPoolExecutor, max_in_flight: int
    ) -> None:
        self._executor = executor
        self._sem = threading.BoundedSemaphore(max_in_flight)

    def submit(
        self, fn: Callable, *args, timeout: Optional[float] = 10, **kwargs
    ) -> Optional[concurrent.futures.Future]:
        """Submits a task to the executor with a specified timeout."""
        if not self._sem.acquire(blocking=False):
            return None  # Semaphore limit reached, return None

        # Submit the task to the executor
        fut = self._executor.submit(fn, *args, **kwargs)

        # Set up a callback to handle the completion of the task
        fut.add_done_callback(
            lambda f: self._sem.release()
        )  # Release the semaphore when done
        self._wait_with_timeout(fut, timeout)  # Wait for the task with timeout

        return fut

    def _wait_with_timeout(
        self, fut: concurrent.futures.Future, timeout: float
    ) -> None:
        """Wait for the future to complete and handle the timeout in background."""

        def check_timeout():
            try:
                fut.result(
                    timeout=timeout
                )  # This will raise TimeoutError if it exceeds the timeout
            except concurrent.futures.TimeoutError as e:
                logger.error("Timeout processing task: %s", e, exc_info=True)
            except Exception as e:
                logger.error("Error processing task: %s", e, exc_info=True)

        # Submit the timeout check to the executor to run asynchronously
        self._executor.submit(check_timeout)
