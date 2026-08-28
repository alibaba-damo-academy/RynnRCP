"""
ProcessNode – Independent process node with its own GIL.

Each ProcessNode runs in a child ``multiprocessing.Process`` and contains:
- Its own ChannelManager singleton (fresh per-process)
- A set of component setup functions executed on startup
- A connection to the SharedChannelRegistry for cross-process routing
"""

from __future__ import annotations

import logging
import os
import threading
from typing import Any, Callable, List, Optional

logger = logging.getLogger(__name__)


class ProcessNode:
    """A unit of execution that runs in its own OS process.

    Component *setup functions* are registered before the node is started.
    Each ``setup_fn`` has the signature::

        def setup_fn(channel_manager, config) -> Optional[threading.Event]:
            # Initialise component, start worker threads, etc.
            # Return a stop event (or None) that will be set on shutdown.

    Args:
        name: Human-readable process name.
        config: Configuration dict passed to every setup function.
        registry_attached: Whether to attach the SharedChannelRegistry
                           on startup (default True).
    """

    def __init__(
        self, name: str, config: Optional[dict] = None, registry_attached: bool = True
    ) -> None:
        self.name = name
        self.config = config or {}
        self._components: List[Callable] = []
        self._running = False
        self._registry_attached = registry_attached

    def add_component(self, setup_fn: Callable) -> None:
        """Register a component initialisation function.

        The function will be called inside the child process with
        ``(ChannelManager, config)`` as arguments.
        """
        self._components.append(setup_fn)

    def run(self, lifecycle_queue: Any = None, shutdown_event: Any = None) -> None:
        """Child-process entry point.

        This method is intended to be the ``target`` of a
        ``multiprocessing.Process``.  It:

        1. Resets the process-local ChannelManager singleton.
        2. Optionally connects to the SharedChannelRegistry.
        3. Calls each registered component setup function.
        4. Blocks until the launcher sends a shutdown event.
        5. Sets all returned stop-events and closes channels.
        """
        mgr = None
        registry = None
        stop_events: List[threading.Event] = []
        cleanup_callbacks: List[Callable[[], None]] = []
        signal_event = threading.Event()
        parent_pid = os.getppid()
        _start_parent_watchdog(parent_pid, signal_event, self.name)

        try:
            # Deferred imports avoid circular deps and ensure fresh per-process state.
            from rynnrcp.ipc.channel import ChannelManager
            from rynnrcp.utils.logging import configure_logging

            _configure_process_logging(self.name, self.config, configure_logging)
            logger.info("Process node '%s' starting", self.name)

            ChannelManager.reset()
            mgr = ChannelManager.instance()

            if self._registry_attached:
                from rynnrcp.ipc.channel_registry import SharedChannelRegistry

                registry_name = self.config.get(
                    "channel_registry_name", "rynnrcp_channel_registry"
                )
                registry = SharedChannelRegistry(create=False, name=registry_name)
                mgr.attach_registry(registry)

            for setup_fn in self._components:
                detail = getattr(setup_fn, "__name__", repr(setup_fn))
                logger.info("Process node '%s' running setup: %s", self.name, detail)
                _send_lifecycle(lifecycle_queue, self.name, "starting", detail=detail)
                result = setup_fn(mgr, self.config)
                _collect_lifecycle_result(result, stop_events, cleanup_callbacks)
                logger.info("Process node '%s' setup completed: %s", self.name, detail)

            self._running = True
            _send_lifecycle(lifecycle_queue, self.name, "ready")
            _wait_for_shutdown(signal_event, shutdown_event)
        except KeyboardInterrupt:
            logger.info("Process node '%s' interrupted; shutting down", self.name)
            _send_lifecycle(
                lifecycle_queue, self.name, "stopped", detail="KeyboardInterrupt"
            )
            return
        except BaseException as exc:
            logger.exception("Process node '%s' failed", self.name)
            _send_lifecycle(
                lifecycle_queue,
                self.name,
                "failed",
                error=f"{type(exc).__name__}: {exc}",
            )
            raise
        finally:
            self._running = False
            _cleanup_node(mgr, registry, stop_events, cleanup_callbacks)

    @property
    def running(self) -> bool:
        return self._running


def _configure_process_logging(
    name: str, config: dict, configure_logging: Callable
) -> None:
    from rynnrcp.utils.logging import resolve_log_run_id, set_log_context

    runtime_config = config.get("runtime_config") if isinstance(config, dict) else None
    robot_id = str(getattr(runtime_config, "robot_id", "") or "")
    session_id = str(getattr(runtime_config, "log_session_id", "") or "")
    # Carry the parent run's correlation ids into this child process so all
    # per-process log files can be joined on robot_id/session_id.
    set_log_context(
        robot_id=robot_id or None,
        run_id=resolve_log_run_id(session_id or None),
        session_id=session_id or None,
        process=name,
    )
    if not robot_id:
        configure_logging()
        return

    from rynnrcp.utils import safe_name
    from rynnrcp.utils.user_paths import log_session_dir, logs_dir, robot_root

    root = robot_root(robot_id)
    log_dir = log_session_dir(root, session_id) if session_id else logs_dir(root)
    configure_logging(
        sinks=["stderr", "file"],
        file_path=str(log_dir / f"process_{safe_name(name)}.log"),
    )


def _cleanup_node(
    mgr: Any,
    registry: Any,
    stop_events: List[threading.Event],
    cleanup_callbacks: List[Callable[[], None]],
) -> None:
    """Release component, channel, and registry resources."""
    logger.info(
        "[ProcessNodeShutdown] begin stop_events=%d cleanups=%d",
        len(stop_events),
        len(cleanup_callbacks),
    )
    for ev in stop_events:
        ev.set()
    for cleanup in reversed(cleanup_callbacks):
        try:
            detail = getattr(cleanup, "__name__", repr(cleanup))
            logger.info("[ProcessNodeShutdown] running cleanup: %s", detail)
            cleanup()
            logger.info("[ProcessNodeShutdown] cleanup completed: %s", detail)
        except Exception as exc:
            logger.warning("Component cleanup error: %s", exc, exc_info=True)
    if mgr is not None:
        try:
            logger.info("[ProcessNodeShutdown] close ChannelManager")
            from rynnrcp.ipc.channel import ChannelManager

            ChannelManager.reset()
            logger.info("[ProcessNodeShutdown] ChannelManager closed")
        except Exception as exc:
            logger.warning("Channel cleanup error: %s", exc, exc_info=True)
    if registry is not None:
        try:
            logger.info("[ProcessNodeShutdown] close channel registry")
            registry.close()
            logger.info("[ProcessNodeShutdown] channel registry closed")
        except Exception as exc:
            logger.warning("Registry cleanup error: %s", exc, exc_info=True)
    logger.info("[ProcessNodeShutdown] complete")


def _wait_for_shutdown(signal_event: threading.Event, shutdown_event: Any) -> None:
    """Wait for POSIX signals or a cross-platform multiprocessing Event."""
    if shutdown_event is None:
        signal_event.wait()
        return
    while not signal_event.is_set():
        try:
            if shutdown_event.wait(timeout=0.1):
                return
        except Exception:
            signal_event.wait(timeout=0.1)
            return


def _start_parent_watchdog(
    parent_pid: int, signal_event: threading.Event, node_name: str
) -> None:
    if parent_pid <= 1:
        return

    def _watch_parent() -> None:
        while not signal_event.wait(timeout=0.5):
            current_parent = os.getppid()
            if current_parent == parent_pid:
                continue
            logger.error(
                "Process node '%s' parent process disappeared: parent_pid=%s current_parent=%s; exiting",
                node_name,
                parent_pid,
                current_parent,
            )
            os._exit(1)

    threading.Thread(
        target=_watch_parent,
        name=f"{node_name}-parent-watchdog",
        daemon=True,
    ).start()


def _send_lifecycle(
    queue: Any, name: str, status: str, error: str = "", detail: str = ""
) -> None:
    if queue is None:
        return
    try:
        queue.put({"name": name, "status": status, "error": error, "detail": detail})
    except Exception:
        logger.debug(
            "Failed to send lifecycle event for node '%s'", name, exc_info=True
        )


def _collect_lifecycle_result(
    result,
    stop_events: List[threading.Event],
    cleanup_callbacks: List[Callable[[], None]],
) -> None:
    """Normalize component setup return values.

    Existing components can still return ``threading.Event``. Newer components
    may return ``(event, cleanup)`` or an object exposing ``stop()``.
    """
    if result is None:
        return
    if isinstance(result, threading.Event):
        stop_events.append(result)
        return
    if isinstance(result, tuple):
        for item in result:
            _collect_lifecycle_result(item, stop_events, cleanup_callbacks)
        return
    if callable(result):
        cleanup_callbacks.append(result)
        return
    stop = getattr(result, "stop", None)
    if callable(stop):
        cleanup_callbacks.append(stop)
