# rynnrcp/rynnrcp.py

"""
Unified RynnRCP entrypoint (core + communication plugins).
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rynnrcp.rynnrcp.RynnRCP`, a top-level orchestrator that
combines an :class:`~rcp_core.RcpCore` instance (tools + servers + protocol adapters)
with one or more external communication plugins (instances of
:class:`~comm_plugin.RcpPlugin`).

Responsibilities
----------------
- Holds a reference to the core runtime (``self.rcp_core``).
- Manages a list of plugins (``self.plugins``).
- Binds the core into each plugin via ``plugin.bind_rcp_core(...)`` so plugins can call
  ``tool_list`` / ``tool_call``.
- Runs each plugin’s ``start()`` method in a background thread and provides a unified
  shutdown path.

Lifecycle
---------
- :meth:`add_plugin`:
  Adds a plugin at runtime and immediately binds it to the current core.

- :meth:`start`:
  Starts all plugins in daemon threads and then waits until termination:
  - On Linux/macOS: blocks on ``thread.join()``, relying on native Ctrl+C behavior
    (KeyboardInterrupt) to trigger shutdown.
  - On Windows: installs a SIGINT handler and polls a stop event (improves Ctrl+C
    behavior in environments like Git Bash), then stops everything.

- :meth:`stop`:
  Stops all plugins (best-effort) and then calls ``rcp_core.stop()`` to shut down
  protocol adapters and other core resources (e.g., ModuleAdapter destroy hooks).

Notes
-----
Plugins are expected to implement at least:
- ``bind_rcp_core(rcp_core)``
- ``start()``
- ``stop()``
"""

from __future__ import annotations

from typing import Iterable, List
import threading

from rcp_core import RcpCore
from comm_plugin import RcpPlugin


class RynnRCP:
    """
    Top-level unified entry point; composes RcpCore with multiple communication plugins.
    """

    def __init__(
        self,
        rcp_core: RcpCore = None,
        plugins: Iterable[RcpPlugin] = None,
    ) -> None:
        """
        Parameters
        ----------
        rcp_core : RcpCore
        plugins : Iterable[RcpPlugin]
        """

        # 1. Core instance
        self.rcp_core = rcp_core

        # 2. Plugin list
        self.plugins: List[RcpPlugin] = list(plugins) if plugins is not None else []

        # 3. Bind core to all plugins
        for plugin in self.plugins:
            plugin.bind_rcp_core(self.rcp_core)

        # 4. Track plugin threads
        self._threads: List[threading.Thread] = []

    def add_plugin(self, plugin: RcpPlugin) -> None:
        """Add an extra plugin and bind it to the current core."""
        plugin.bind_rcp_core(self.rcp_core)
        self.plugins.append(plugin)

    def start(self) -> None:
        """
        Start all plugins in background threads.
        On Linux / macOS: use blocking join() + KeyboardInterrupt (native behavior).
        On Windows: use signal + polling to ensure Ctrl+C works in Git Bash.
        """
        import sys
        import threading
        import signal

        if sys.platform == "win32":
            # --- Windows path (including Git Bash) ---
            self._stop_event = threading.Event()

            def _sigint_handler(signum, frame):
                print("\n[RynnRCP] Received interrupt signal, stopping plugins...")
                self._stop_event.set()

            # Register handler for SIGINT (Ctrl+C)
            old_handler = signal.signal(signal.SIGINT, _sigint_handler)

            try:
                for plugin in self.plugins:
                    t = threading.Thread(target=plugin.start, daemon=True)
                    t.start()
                    self._threads.append(t)

                # Poll instead of blocking join().
                # Also break when all plugin threads have exited (e.g. web UI
                # quit button calls plugin.stop() which only sets the plugin's
                # own stop event, not RynnRCP's).
                while not self._stop_event.is_set():
                    self._stop_event.wait(timeout=0.5)
                    if all(not t.is_alive() for t in self._threads):
                        break
            except KeyboardInterrupt:
                # Fallback (unlikely but safe)
                print("\n[RynnRCP] KeyboardInterrupt caught on Windows")
            finally:
                signal.signal(signal.SIGINT, old_handler)  # restore
                self.stop()

        else:
            # --- Linux / macOS: original behavior ---
            try:
                for plugin in self.plugins:
                    t = threading.Thread(target=plugin.start, daemon=True)
                    t.start()
                    self._threads.append(t)

                for t in self._threads:
                    t.join()  # blocking, but Ctrl+C raises KeyboardInterrupt reliably
            except KeyboardInterrupt:
                print("[RynnRCP] Caught KeyboardInterrupt, stopping plugins...")
                self.stop()

    def stop(self) -> None:
        """Stop all plugins (to be called on stop / Ctrl+C)."""
        for plugin in self.plugins:
            try:
                plugin.stop()
            except Exception as e:
                print(f"[RynnRCP] Error stopping plugin {plugin}: {e}")

        # ProtocolFactory → ModuleAdapter → destroy
        if self.rcp_core is not None and hasattr(self.rcp_core, "stop"):
            try:
                self.rcp_core.stop()
            except Exception as e:
                print(f"[RynnRCP] Error shutting down RcpCore: {e}")
