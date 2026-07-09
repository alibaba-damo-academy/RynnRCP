"""
NodeLauncher – Orchestrates multiple ProcessNode instances.

Creates a SharedChannelRegistry, launches each node as a separate
``multiprocessing.Process``, monitors health, and performs graceful shutdown.

Usage::

    launcher = NodeLauncher()
    launcher.add_node("sensor_proc", sensor_setup_fn)
    launcher.add_node("control_proc", control_setup_fn)
    launcher.start()
    # ...
    launcher.shutdown()
"""

from __future__ import annotations

import multiprocessing
import queue
import time
import logging
from typing import Any, Callable, Dict, Optional

from rynnrcp.process.process_node import ProcessNode

logger = logging.getLogger(__name__)

_DEFAULT_SHUTDOWN_TIMEOUT_S = 8.0


class NodeLauncher:
    """Process orchestrator for multi-process rynnrcp deployments.

    Args:
        config: Global configuration dict shared with all nodes.
    """

    def __init__(
        self,
        config: Optional[dict] = None,
        *,
        start_method: str = "spawn",
        startup_timeout_s: float = 30.0,
        registry_name: str = "rynnrcp_channel_registry",
    ) -> None:
        self.config = config or {}
        self._nodes: Dict[str, ProcessNode] = {}
        self._processes: Dict[str, multiprocessing.Process] = {}
        self._shutdown_events: Dict[str, Any] = {}
        self._mp_ctx = multiprocessing.get_context(start_method)
        self._registry = None  # SharedChannelRegistry (lazy)
        self._registry_name = str(registry_name)
        self._started = False
        self._startup_timeout_s = float(startup_timeout_s)
        self._last_errors: Dict[str, str] = {}
        self._last_status: Dict[str, str] = {}

    def add_node(self, name: str, setup_fn: Callable,
                 node_config: Optional[dict] = None) -> None:
        """Register a process node with one initial component.

        Multiple components can be added to the same node by calling
        ``add_component_to_node`` afterwards.

        Args:
            name: Unique process name.
            setup_fn: Component setup callable ``(ChannelManager, config) -> Event|None``.
            node_config: Per-node config override; defaults to global config.
        """
        if name in self._nodes:
            raise ValueError(f"Node '{name}' already registered")
        node = ProcessNode(name, config=node_config or self.config)
        node.add_component(setup_fn)
        self._nodes[name] = node

    def add_component_to_node(self, node_name: str,
                               setup_fn: Callable) -> None:
        """Add another component setup function to an existing node."""
        if node_name not in self._nodes:
            raise KeyError(f"Node '{node_name}' not found")
        self._nodes[node_name].add_component(setup_fn)

    # ── lifecycle ────────────────────────────────────────────────────

    def start(self) -> None:
        """Create the shared registry and launch all registered nodes."""
        if self._started:
            raise RuntimeError("NodeLauncher already started")

        self.prepare_registry()
        logger.info(
            "[NodeStartup] launching %d process nodes timeout=%.1fs nodes=%s",
            len(self._nodes),
            self._startup_timeout_s,
            sorted(self._nodes.keys()),
        )

        lifecycle_queue = self._mp_ctx.Queue()
        pending = set(self._nodes.keys())
        try:
            for name, node in self._nodes.items():
                shutdown_event = self._mp_ctx.Event()
                p = self._mp_ctx.Process(
                    target=node.run,
                    args=(lifecycle_queue, shutdown_event),
                    name=f"rynnrcp_{name}",
                    daemon=False,
                )
                logger.info("[NodeStartup] spawning node '%s'", name)
                p.start()
                logger.info("[NodeStartup] spawned node '%s' pid=%s", name, p.pid)
                self._processes[name] = p
                self._shutdown_events[name] = shutdown_event

            self._wait_until_ready(lifecycle_queue, pending)
            self._started = True
            logger.info("[NodeStartup] all process nodes ready")
        except BaseException:
            self.shutdown(timeout=1.0)
            raise

    def prepare_registry(self):
        """Create the shared channel registry before child processes start.

        Runtime uses this to attach the main-process ChannelManager early, so
        services can create SHM channels before runner processes subscribe.
        """
        if self._registry is None:
            from rynnrcp.ipc.channel_registry import SharedChannelRegistry

            self._registry = SharedChannelRegistry(create=True, name=self._registry_name)
        return self._registry

    def shutdown(self, timeout: float = _DEFAULT_SHUTDOWN_TIMEOUT_S) -> None:
        """Gracefully terminate all child processes and clean up.

        Args:
            timeout: Max seconds to wait per process after requesting graceful shutdown.
        """
        logger.info(
            "[NodeShutdown] begin nodes=%s timeout=%.1fs",
            sorted(self._processes.keys()),
            timeout,
        )
        # Ask processes to exit cleanly first. This works on Windows too,
        # where terminate() is a hard kill and does not run Python cleanup.
        for name, p in self._processes.items():
            if p.is_alive():
                event = self._shutdown_events.get(name)
                if event is not None:
                    logger.info("[NodeShutdown] signal node '%s' pid=%s", name, p.pid)
                    event.set()

        # Wait for each process to finish gracefully.
        for name, p in self._processes.items():
            if p.is_alive():
                logger.info("[NodeShutdown] waiting node '%s' pid=%s", name, p.pid)
            p.join(timeout=timeout)
            if p.is_alive():
                logger.info("[NodeShutdown] node '%s' still running after graceful wait", name)
            else:
                logger.info(
                    "[NodeShutdown] node '%s' stopped exitcode=%s",
                    name,
                    p.exitcode,
                )

        # Terminate any survivors.
        for name, p in self._processes.items():
            if p.is_alive():
                logger.warning(
                    "Process node '%s' did not stop after %.1fs; sending terminate (pid=%s)",
                    name,
                    timeout,
                    p.pid,
                )
                p.terminate()
                logger.info("[NodeShutdown] waiting node '%s' after terminate", name)
                p.join(timeout=1.0)
                if not p.is_alive():
                    logger.info(
                        "[NodeShutdown] node '%s' stopped after terminate exitcode=%s",
                        name,
                        p.exitcode,
                    )

        # Force-kill any survivors (Windows: TerminateProcess)
        for name, p in self._processes.items():
            if p.is_alive():
                logger.error(
                    "Process node '%s' ignored terminate; sending kill (pid=%s)",
                    name,
                    p.pid,
                )
                p.kill()
                p.join(timeout=1.0)
            if p.is_alive():
                logger.error("Process node '%s' is still alive after kill (pid=%s)", name, p.pid)

        self._processes.clear()
        self._shutdown_events.clear()
        self._last_errors.clear()
        self._last_status.clear()

        # Clean up registry SHM
        if self._registry is not None:
            logger.info("[NodeShutdown] close channel registry name=%s", self._registry_name)
            self._registry.close()
            self._registry = None

        self._started = False
        logger.info("[NodeShutdown] complete")

    # ── monitoring ───────────────────────────────────────────────────

    def is_alive(self, name: str) -> bool:
        """Check whether a specific node process is alive."""
        p = self._processes.get(name)
        return p is not None and p.is_alive()

    def health_check(self) -> Dict[str, bool]:
        """Return ``{node_name: is_alive}`` for all registered nodes."""
        return {name: self.is_alive(name) for name in self._nodes}

    def detailed_health_check(self) -> Dict[str, Dict[str, Any]]:
        """Return detailed process status for diagnostics."""
        status: Dict[str, Dict[str, Any]] = {}
        for name in self._nodes:
            p = self._processes.get(name)
            status[name] = {
                "alive": p is not None and p.is_alive(),
                "pid": p.pid if p is not None else None,
                "exitcode": p.exitcode if p is not None else None,
                "error": self._last_errors.get(name, ""),
            }
        return status

    def list_nodes(self) -> list:
        """Return list of registered node names."""
        return list(self._nodes.keys())

    @property
    def started(self) -> bool:
        return self._started

    @property
    def registry(self):
        """Return the prepared SharedChannelRegistry, if any."""
        return self._registry

    def _wait_until_ready(self, lifecycle_queue, pending: set[str]) -> None:
        deadline = time.monotonic() + self._startup_timeout_s
        while pending:
            for name in list(pending):
                p = self._processes.get(name)
                if p is not None and p.exitcode is not None:
                    self._last_errors[name] = f"process exited during startup, exitcode={p.exitcode}"
                    pending.remove(name)
                    raise RuntimeError(f"Node '{name}' exited during startup, exitcode={p.exitcode}")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                missing = ", ".join(sorted(pending))
                details = {
                    name: {
                        "pid": self._processes[name].pid if name in self._processes else None,
                        "alive": self._processes[name].is_alive() if name in self._processes else False,
                        "last_status": self._last_status.get(name, "no lifecycle message received"),
                    }
                    for name in sorted(pending)
                }
                raise TimeoutError(
                    f"Timed out waiting for nodes to start: {missing}. "
                    f"Startup details: {details}"
                )
            try:
                message = lifecycle_queue.get(timeout=min(0.1, remaining))
            except queue.Empty:
                continue
            name = message.get("name")
            if name not in pending:
                continue
            status = message.get("status")
            detail = message.get("detail", "")
            if status:
                self._last_status[name] = f"{status}: {detail}" if detail else str(status)
                logger.info(
                    "[NodeStartup] node '%s' status=%s detail=%s",
                    name,
                    status,
                    detail,
                )
            if status == "ready":
                pending.remove(name)
                continue
            if status == "failed":
                error = message.get("error", "")
                self._last_errors[name] = error
                pending.remove(name)
                raise RuntimeError(f"Node '{name}' failed during startup: {error}")
