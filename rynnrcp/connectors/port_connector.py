"""
Port connector – device driver integration.

Ported from RynnRCP PortAdapter. Manages hardware device drivers that follow
the start/read/stop lifecycle (e.g. cameras, sensors).

Subscription: instantiate driver class, call start(), poll read() in a
background thread, forward results to callback.

Publishing: not supported (raises NotImplementedError).
"""

from __future__ import annotations

import importlib
import inspect
import logging
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple

from rynnrcp.utils.imports import apply_sys_path, normalize_sys_path
from rynnrcp.utils import camel_to_snake

from .base import BaseConnector

logger = logging.getLogger(__name__)

_MIN_POLL_INTERVAL_S = 0.001
_THREAD_JOIN_TIMEOUT_S = 1.0


class PortConnector(BaseConnector):
    """Device driver connector using start/read/stop lifecycle.

    Params:
      port_type  : str  – fully-qualified driver class path, or class name
                  searched from ``search_packages``
      init_args  : dict – kwargs for the constructor
      object_name    : str  – optional disambiguation key
      search_packages : list[str] – packages to search for the driver class
    """

    def __init__(self) -> None:
        self._running: Dict[str, bool] = {}
        self._threads: Dict[str, threading.Thread] = {}
        self._instances: Dict[str, Any] = {}
        self._class_cache: Dict[tuple[str, tuple[str, ...]], Tuple[str, type]] = {}
        self._cache_lock = threading.Lock()
        self._scheduled_components: Dict[str, tuple[Any, str]] = {}
        self._last_error_log_at: Dict[str, float] = {}

    # ── class resolution ─────────────────────────────────────────────

    def _discover_subpackages(self, base_package: str) -> list:
        """Discover subpackages of *base_package*."""
        try:
            import pkgutil
            pkg = importlib.import_module(base_package)
            subs = []
            for m in pkgutil.iter_modules(pkg.__path__, pkg.__name__ + "."):
                if m.ispkg:
                    subs.append(m.name)
            subs.sort()
            return subs
        except (ImportError, AttributeError):
            return []

    @staticmethod
    def _class_cache_key(port_type: str, search_packages: list | None = None) -> tuple[str, tuple[str, ...]]:
        return port_type, normalize_sys_path(search_packages)

    def _resolve_class(
        self, port_type: str, search_packages: list | None = None
    ) -> Tuple[str, type]:
        """Resolve *port_type* class by searching packages."""
        cache_key = self._class_cache_key(port_type, search_packages)
        with self._cache_lock:
            if cache_key in self._class_cache:
                return self._class_cache[cache_key]

        if "." in port_type:
            candidates = [port_type]
            if not port_type.startswith("rynnkit."):
                candidates.append(f"rynnkit.{port_type}")
            last_err: Optional[Exception] = None
            for candidate in candidates:
                mod_name, class_name = candidate.rsplit(".", 1)
                try:
                    mod = importlib.import_module(mod_name)
                    cls = getattr(mod, class_name)
                except Exception as exc:
                    last_err = exc
                    continue
                if not inspect.isclass(cls):
                    raise TypeError(f"{candidate} is not a class")
                with self._cache_lock:
                    self._class_cache[cache_key] = (mod_name, cls)
                return mod_name, cls
            raise ValueError(f"Cannot resolve port_type='{port_type}'. Last error: {last_err}")

        mod_short = camel_to_snake(port_type)

        # Build search candidates
        candidates = []
        for pkg in normalize_sys_path(search_packages):
            subs = self._discover_subpackages(pkg)
            candidates.extend(subs)
            # Also try the package itself
            candidates.append(pkg)

        if not candidates:
            # Fallback: try direct import
            candidates = [mod_short]

        last_err: Optional[Exception] = None
        for pkg in candidates:
            mod_name = f"{pkg}.{mod_short}" if "." in pkg else mod_short
            try:
                mod = importlib.import_module(mod_name)
            except Exception as e:
                last_err = e
                continue
            cls = getattr(mod, port_type, None)
            if cls is not None and inspect.isclass(cls):
                with self._cache_lock:
                    self._class_cache[cache_key] = (mod_name, cls)
                return mod_name, cls

        raise ValueError(
            f"Cannot resolve port_type='{port_type}'. Last error: {last_err}"
        )

    # ── sub ──────────────────────────────────────────────────────────

    def sub(self, params: Dict[str, Any], callback: Callable) -> None:
        port_type = params.get("port_type")
        if not port_type:
            raise ValueError("params.port_type is required")

        init_args = params.get("init_args") or {}
        object_name = params.get("object_name", "")
        method_name = str(params.get("method_name") or "read")
        search_pkgs = params.get("search_packages")
        apply_sys_path(normalize_sys_path(params.get("sys_path")))

        mod_name, cls = self._resolve_class(str(port_type), search_pkgs)
        logger.info("PortConnector resolved %s from %s", port_type, mod_name)

        inst = cls(**init_args)

        # Verify start/read/stop interface
        for method in ("start", method_name, "stop"):
            if not callable(getattr(inst, method, None)):
                raise TypeError(f"{port_type} has no callable {method}()")

        device_id = init_args.get("device_id", "")
        key = f"{mod_name}::{device_id}::{object_name}"

        inst.start()

        self._instances[key] = inst
        self._running[key] = True

        interval = params.get("interval")
        if interval is None:
            fps = init_args.get("fps", 0)
            interval = 1.0 / fps if fps and fps > 0 else 0.0
        interval = max(float(interval), _MIN_POLL_INTERVAL_S)
        scheduler = params.get("_scheduler")
        component_name = params.get("_component_name") or f"port:{key}"

        if scheduler is not None:
            self._start_scheduler_subscription(
                scheduler=scheduler,
                component_name=str(component_name),
                key=key,
                instance=inst,
                method_name=method_name,
                interval=interval,
                callback=callback,
            )
            self._scheduled_components[key] = (scheduler, str(component_name))
            return

        self._start_thread_subscription(
            key=key,
            instance=inst,
            method_name=method_name,
            interval=interval,
            callback=callback,
            log_message=(
                "PortConnector read error "
                f"(port_type={port_type}, method={method_name}, device_id={device_id})"
            ),
        )

    # ── pub ──────────────────────────────────────────────────────────

    def pub(self, params: Dict[str, Any], msg: Any) -> None:
        raise NotImplementedError("PortConnector does not support pub()")

    # ── stop ─────────────────────────────────────────────────────────

    def stop(self) -> None:
        for k in list(self._running):
            self._running[k] = False

        for _key, (scheduler, component_name) in list(self._scheduled_components.items()):
            try:
                scheduler.remove_component(component_name)
            except Exception as exc:
                logger.debug("PortConnector scheduler cleanup ignored (%s): %s", component_name, exc, exc_info=True)
        self._scheduled_components.clear()

        for key, thread in list(self._threads.items()):
            if thread.is_alive():
                thread.join(timeout=_THREAD_JOIN_TIMEOUT_S)
                if thread.is_alive():
                    logger.warning(
                        "PortConnector thread did not stop within %.1fs: %s",
                        _THREAD_JOIN_TIMEOUT_S,
                        key,
                    )
        self._threads.clear()

        for k, inst in list(self._instances.items()):
            try:
                inst.stop()
            except Exception as e:
                logger.error("PortConnector stop error (%s): %s", k, e, exc_info=True)

        self._running.clear()
        self._instances.clear()
        self._scheduled_components.clear()
        self._last_error_log_at.clear()

        logger.info("PortConnector stopped all instances")

    def _log_loop_error(self, key: str, message: str, exc: Exception) -> None:
        now = time.monotonic()
        last = self._last_error_log_at.get(key, 0.0)
        if now - last < 2.0:
            return
        self._last_error_log_at[key] = now
        logger.error("%s instance=%s: %s", message, key, exc, exc_info=True)

    def _start_scheduler_subscription(
        self,
        *,
        scheduler: Any,
        component_name: str,
        key: str,
        instance: Any,
        method_name: str,
        interval: float,
        callback: Callable,
    ) -> None:
        from rynnrcp.runtime.scheduler import Component

        method = getattr(instance, method_name)

        def tick() -> None:
            if not self._running.get(key, False):
                return
            callback(method())

        scheduler.add_component(
            Component(
                str(component_name),
                period_ms=interval * 1000.0,
                callback=tick,
            )
        )

    def _start_thread_subscription(
        self,
        *,
        key: str,
        instance: Any,
        method_name: str,
        interval: float,
        callback: Callable,
        log_message: str,
    ) -> None:
        def loop() -> None:
            method = getattr(instance, method_name)
            while self._running.get(key, False):
                try:
                    callback(method())
                except Exception as exc:
                    self._log_loop_error(key, log_message, exc)
                    time.sleep(0.01)
                time.sleep(interval)

        th = threading.Thread(target=loop, daemon=True)
        th.start()
        self._threads[key] = th
