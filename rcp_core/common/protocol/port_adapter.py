#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# rcp_core/common/protocol/port_adapter.py

"""
Port protocol adapter (device drivers via rcp_sensor).
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.protocol.port_adapter.PortAdapter`, an
implementation of :class:`~rcp_core.common.middleware.base_protocol_adapter.BaseProtocolAdapter`
for the ``port`` protocol, intended for “generic device” drivers located under the
``rcp_sensor`` package (e.g., cameras).

Configuration / expected params
-------------------------------
- ``port_type`` (str, required): class name of the driver, e.g. ``"USBCamera"``
- ``init_args`` (dict, optional): kwargs passed to the constructor
- ``out_key`` (str, optional): used to disambiguate multiple subscriptions

Class resolution
----------------
The adapter resolves ``port_type`` by searching subpackages of ``rcp_sensor`` and trying
to import a module named after the snake_case version of the class, e.g.:

    ``USBCamera`` → ``usb_camera`` → try ``rcp_sensor.<subpkg>.usb_camera``

It selects the first module that defines a matching class object and caches the result.

Subscription behavior (fixed)
-----------------------------
:meth:`PortAdapter.sub`:
- instantiates the resolved class with ``init_args``
- requires the instance to provide callable ``start()``, ``read()``, and ``stop()``
- calls ``start()``
- spawns a daemon thread that repeatedly calls ``read()`` and forwards the return value
  to the provided callback
- optionally paces reads using ``init_args.fps`` (sleeping ``1/fps`` seconds per loop)

Publishing
----------
:meth:`PortAdapter.pub` is not supported and raises ``NotImplementedError``.

Shutdown
--------
:meth:`PortAdapter.stop` stops all polling loops and calls ``stop()`` on every created
device instance.
"""

from __future__ import annotations

import importlib
import inspect
import pkgutil
import re
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple

from .base_protocol_adapter import BaseProtocolAdapter
from rcp_core.common.utils.logger import server_logger

logger = server_logger()

PORT_AVAILABLE = True


def _camel_to_snake(name: str) -> str:
    s1 = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s1).lower()


class PortAdapter(BaseProtocolAdapter):
    """
    protocol: port

    Expected params:
      port_type: str             # class name, e.g. "USBCamera"
      init_args: dict            # kwargs for constructor
      out_key: str (optional)    # used to distinguish multiple subs

    Behavior is fixed:
      - instantiate by port_type + init_args
      - call start()
      - spawn a thread to call read() in a loop and callback(ret)
      - stop(): stop loop threads and call stop() for each instance

    Notes:
      - This adapter resolves class by searching module `rcp_sensor.*.<snake(port_type)>`
        Example: port_type=USBCamera -> try importing:
          rcp_sensor.camera.usb_camera
          rcp_sensor.xxx.usb_camera
        until it finds class `USBCamera` in that module.
      - No BaseCamera filtering: "generic devices".
    """

    def __init__(self):
        self._running: Dict[str, bool] = {}
        self._threads: Dict[str, threading.Thread] = {}
        self._instances: Dict[str, Any] = {}

        # cache: port_type -> (module_name, class)
        self._class_cache: Dict[str, Tuple[str, type]] = {}

        # lazy cache of rcp_sensor subpackages: ["rcp_sensor.camera", ...]
        self._rcp_sensor_subpkgs: Optional[list[str]] = None
        self._cache_lock = threading.Lock()

    def _list_rcp_sensor_subpackages(self) -> list[str]:
        if self._rcp_sensor_subpkgs is not None:
            return self._rcp_sensor_subpkgs

        import rcp_sensor  # must exist as a package

        subpkgs: list[str] = []
        for m in pkgutil.iter_modules(rcp_sensor.__path__, rcp_sensor.__name__ + "."):
            if m.ispkg:
                subpkgs.append(m.name)

        # deterministic order helps debugging
        subpkgs.sort()
        self._rcp_sensor_subpkgs = subpkgs
        return subpkgs

    def _resolve_class(self, port_type: str) -> Tuple[str, type]:
        # fast path: cached
        with self._cache_lock:
            if port_type in self._class_cache:
                return self._class_cache[port_type]

        mod_short = _camel_to_snake(port_type)  # e.g. "usb_camera"
        candidates = self._list_rcp_sensor_subpackages()

        last_import_err: Optional[Exception] = None

        for pkg in candidates:
            mod_name = f"{pkg}.{mod_short}"  # e.g. rcp_sensor.camera.usb_camera
            try:
                mod = importlib.import_module(mod_name)
            except Exception as e:
                last_import_err = e
                continue

            cls = getattr(mod, port_type, None)
            if cls is None or not inspect.isclass(cls):
                continue

            with self._cache_lock:
                self._class_cache[port_type] = (mod_name, cls)
            return mod_name, cls

        tried = [f"{p}.{mod_short}" for p in candidates]
        raise ValueError(
            f"[PortAdapter] Cannot resolve port_type='{port_type}'. "
            f"Tried modules: {tried}. "
            f"Last import error: {last_import_err}"
        )

    def sub(self, params: Dict[str, Any], callback: Callable) -> None:
        port_type = params.get("port_type")
        if not port_type:
            raise ValueError("[PortAdapter] params.port_type is required")

        init_args = params.get("init_args", {}) or {}
        out_key = params.get("out_key", "")

        mod_name, cls = self._resolve_class(str(port_type))
        logger.info(f"[PortAdapter] Resolved {port_type} from {mod_name}")

        inst = cls(**init_args)

        # instance key: make it stable and unique-ish
        device_id = init_args.get("device_id", "")
        inst_key = f"{port_type}::{device_id}::{out_key}"

        # start() required (fixed behavior)
        if not hasattr(inst, "start") or not callable(getattr(inst, "start")):
            raise TypeError(f"[PortAdapter] {port_type} has no callable start()")
        if not hasattr(inst, "read") or not callable(getattr(inst, "read")):
            raise TypeError(f"[PortAdapter] {port_type} has no callable read()")
        if not hasattr(inst, "stop") or not callable(getattr(inst, "stop")):
            raise TypeError(f"[PortAdapter] {port_type} has no callable stop()")

        inst.start()

        self._instances[inst_key] = inst
        self._running[inst_key] = True

        # optional fps pacing (if present)
        fps = init_args.get("fps", None)
        period = (1.0 / float(fps)) if fps else 0.0

        def loop():
            while self._running.get(inst_key, False):
                try:
                    ret = inst.read()  # fixed method
                    callback(ret)  # pass through
                except Exception as e:
                    logger.error(f"[PortAdapter] read() failed ({inst_key}): {e}")
                    time.sleep(0.01)

                if period > 0:
                    time.sleep(period)

        th = threading.Thread(target=loop, daemon=True)
        th.start()
        self._threads[inst_key] = th

    def pub(self, params: Dict[str, Any], msg: Any) -> None:
        raise NotImplementedError("[PortAdapter] pub() not supported")

    def stop(self) -> None:
        # stop loops
        for k in list(self._running.keys()):
            self._running[k] = False

        # stop instances
        for k, inst in list(self._instances.items()):
            try:
                inst.stop()
            except Exception as e:
                logger.error(f"[PortAdapter] stop() failed ({k}): {e}")

        logger.info("[PortAdapter] Stopped all port instances")
