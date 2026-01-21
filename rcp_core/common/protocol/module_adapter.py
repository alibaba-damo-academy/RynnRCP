# rcp_core/common/protocol/module_adapter.py

"""
Module protocol adapter (in-process Python integration).
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.protocol.module_adapter.ModuleAdapter`, an
implementation of :class:`~rcp_core.common.middleware.base_protocol_adapter.BaseProtocolAdapter`
that treats arbitrary Python classes/functions as a “protocol”, driven entirely by YAML
configuration.

It supports two modes:

Subscription mode (input)
-------------------------
:meth:`ModuleAdapter.sub` creates (or reuses) a Python instance and periodically calls
a configured method (default ``read``) in a background thread:

- ``params.module_name``: import path to a factory/class (e.g. ``pkg.mod.Camera``)
- ``params.init_args``: kwargs passed to the factory/constructor
- ``params.method_name`` / ``params.method_kwargs``: method to call and kwargs
- ``params.interval``: sleep between calls (seconds)

The return value of each call is forwarded to the provided callback.

Publish mode (output)
---------------------
:meth:`ModuleAdapter.pub` performs a single method call on the configured instance:

- ``params.method_name``: method to invoke
- ``params.dynamic_arg``: list mapping message dict keys (``from_key``) to function
  parameter names (``arg``)
- ``params.static_args``: list of constant kwargs to include in every call

The adapter optionally converts values based on the target method’s type annotations
(e.g. to ``np.ndarray``, ``list``, ``float``/``int``/``bool``).

Instance lifecycle and safety
-----------------------------
- Instances are keyed by ``module_name`` + ``init_args`` and reused across calls.
- Optional ``start_call`` and ``destroy_call`` sequences can be configured; if absent,
  the adapter attempts to call ``start()`` / ``destroy()`` when available.
- If any configuration sets ``instance_lock: true`` for a given instance key, calls
  into that instance are serialized with a per-instance lock (applies to both sub/pub
  and destroy).
- :meth:`ModuleAdapter.stop` stops all polling threads and runs destroy hooks.
"""

import threading
import time
import importlib
import sys
import inspect
import numpy as np
from typing import Any, Callable, Dict, Tuple, get_origin

from .base_protocol_adapter import BaseProtocolAdapter
from rcp_core.common.utils.logger import server_logger

logger = server_logger()

MODULE_AVAILABLE = True


class ModuleAdapter(BaseProtocolAdapter):
    """
    Module protocol adapter (purely configuration-driven)

    General params convention (shared by sub and pub):
      sys_path: str | List[str]     # Optional, paths to add to sys.path before import
      module_name: str              # Class path or factory function path
      init_args: dict               # Arguments passed to constructor/factory

      start_call:                  # Optional, methods to call after instance creation
        - method_name: str
          method_kwargs: dict

      destroy_call:               # Optional, methods to call before instance destruction
        - method_name: str
          method_kwargs: dict

      # Exclusive to sub:
      method_name: str              # Method name to call in a loop
      method_kwargs: dict           # kwargs for calling method_name during sub
      interval: float               # Interval (seconds) between sub calls

      # Exclusive to pub:
      #   Describe function parameters with dynamic_arg + static_args
      dynamic_arg:                  # List[ {from_key, param} ]
        - from_key: str             # OutputAdapter used from_key to get msg, here use only name
          param: str                # Function parameter name (e.g. positions for set_joint_positions)

      static_args:                  # List[ {param_name: value} ]
        - radians: true
        - mode: "smooth"

      # Optional: Control whether the same instance needs a serial lock
      instance_lock: bool           # Defaults to False; set to True to protect calls to this instance with a lock
    """

    def __init__(self):
        self._threads: Dict[str, threading.Thread] = {}
        self._running: Dict[str, bool] = {}
        self._instances: Dict[str, Any] = {}
        self._instance_params: Dict[str, Dict[str, Any]] = {}
        # One lock for each instance: inst_key -> Lock
        self._instance_locks: Dict[str, threading.Lock] = {}
        # Record whether each instance has locking enabled: inst_key -> bool
        self._instance_lock_flags: Dict[str, bool] = {}

    def _import_obj(self, path: str) -> Any:
        parts = path.split(".")
        if len(parts) < 2:
            raise ValueError(f"[ModuleAdapter] Invalid path: {path}")
        module_name = ".".join(parts[:-1])
        attr_name = parts[-1]
        mod = importlib.import_module(module_name)
        obj = getattr(mod, attr_name)
        return obj

    def _instance_key(self, module_name: str, init_args: Dict[str, Any]) -> str:
        return module_name + "::" + repr(sorted(init_args.items()))

    def _create_or_get_instance(self, params: Dict[str, Any]) -> Tuple[Any, str]:
        module_name = params.get("module_name")
        if not module_name:
            raise ValueError("[ModuleAdapter] params.module_name is not configured")

        # 1. Handle sys.path
        extra_paths = params.get("sys_path") or []
        if isinstance(extra_paths, str):
            extra_paths = [extra_paths]
        for p in extra_paths:
            if p and p not in sys.path:
                sys.path.insert(0, p)

        # 2. Instance key
        init_args = params.get("init_args", {}) or {}
        inst_key = self._instance_key(module_name, init_args)

        # 3. Instance already exists: update lock flag (OR logic), then return
        if inst_key in self._instances:
            # Does the new config require locking?
            new_lock_flag = bool(params.get("instance_lock", False))
            if new_lock_flag:
                # If any configuration specifies instance_lock: true, set the instance's lock flag to True
                self._instance_lock_flags[inst_key] = True
            # Record the last params (optional, for destroy_call, etc.)
            self._instance_params[inst_key] = params
            return self._instances[inst_key], inst_key

        # 4. Create instance
        factory = self._import_obj(module_name)
        instance = factory(**init_args)

        # 5. Start calls for one-time initialization
        start_call = params.get("start_call")
        if not start_call:
            # If start_call is not configured, try to call start() by default
            if hasattr(instance, "start") and callable(getattr(instance, "start")):
                try:
                    getattr(instance, "start")()
                except Exception as e:
                    logger.error(
                        f"[ModuleAdapter] Error calling default start() ({inst_key}): {e}"
                    )
        else:
            for call in start_call or []:
                mname = call.get("method_name")
                if not mname:
                    continue
                mkwargs = call.get("method_kwargs", {}) or {}
                m = getattr(instance, mname)
                m(**mkwargs)

        self._instances[inst_key] = instance
        self._instance_params[inst_key] = params

        # 6. Instance-level locks and their switches
        self._instance_locks[inst_key] = threading.Lock()
        lock_flag = bool(params.get("instance_lock", False))
        self._instance_lock_flags[inst_key] = lock_flag

        return instance, inst_key

    def _convert_for_param(self, func: Callable, param_name: str, value: Any) -> Any:
        """
        Convert value according to func's parameter annotations:
        - No annotation: return as is
        - Annotation is np.ndarray: np.array(value)
        - Annotation is list / Sequence: list(value)
        - Annotation is float / int / bool: cast to corresponding type
        """
        try:
            sig = inspect.signature(func)
        except (TypeError, ValueError):
            return value

        param = sig.parameters.get(param_name)
        if param is None:
            return value

        anno = param.annotation
        if anno is inspect._empty:
            return value

        # Simple handling of string annotations, e.g. 'np.ndarray'
        if isinstance(anno, str):
            t = anno.strip().lower()
            if "ndarray" in t or "numpy" in t:
                return np.array(value)
            if t == "list":
                return list(value)
            if t == "float":
                return float(value)
            if t == "int":
                return int(value)
            if t == "bool":
                return bool(value)
            return value

        origin = get_origin(anno) or anno

        from collections.abc import Sequence

        # numpy.ndarray
        if origin is np.ndarray:
            if not isinstance(value, np.ndarray):
                return np.array(value)
            return value

        # list / Sequence -> list
        if origin in (list, Sequence):
            if not isinstance(value, list):
                return list(value)
            return value

        if origin is float:
            return float(value)
        if origin is int:
            return int(value)
        if origin is bool:
            return bool(value)

        return value

    def _call_with_instance_lock(self, inst_key: str, func: Callable, **kwargs) -> Any:
        """
        If the instance is configured with instance_lock=True, use a lock for serial calls;
        otherwise, call directly.
        """
        lock_flag = self._instance_lock_flags.get(inst_key, False)
        if not lock_flag:
            # No lock needed; call directly
            return func(**kwargs)

        lock = self._instance_locks.get(inst_key)
        if lock is None:
            # Theoretically should not happen; if it does, degrade to no lock
            return func(**kwargs)

        with lock:
            return func(**kwargs)

    def _start_loop(
        self,
        inst_key: str,
        instance: Any,
        method_name: str,
        method_kwargs: Dict[str, Any],
        callback: Callable,
        interval: float = 0.0,
    ):
        self._running[inst_key] = True

        def loop():
            m = getattr(instance, method_name)
            while self._running.get(inst_key, False):
                try:
                    # If the instance needs locking, it will be serialized here; otherwise parallel
                    result = self._call_with_instance_lock(
                        inst_key, m, **(method_kwargs or {})
                    )
                    callback(result)
                except Exception as e:
                    logger.info(
                        f"[ModuleAdapter] Error calling {method_name} ({inst_key}): {e}"
                    )
                    time.sleep(0.01)
                if interval > 0:
                    time.sleep(interval)

        th = threading.Thread(target=loop, daemon=True)
        th.start()
        self._threads[inst_key] = th

    def sub(self, params: Dict[str, Any], callback: Callable) -> None:
        """
        Subscription mode (for input):
          - Periodically calls instance method by params.method_name / method_kwargs / interval
          - The result is given to the callback
        """
        method_name = params.get("method_name", "read")
        method_kwargs = params.get("method_kwargs", {}) or {}
        interval = float(params.get("interval", 0.0))

        instance, inst_key = self._create_or_get_instance(params)
        self._start_loop(
            inst_key, instance, method_name, method_kwargs, callback, interval
        )

    def pub(self, params: Dict[str, Any], msg_dict: Dict[str, Any]) -> None:
        """
        Single call mode (for output):

        Required configuration:
          dynamic_arg: List[ {from_key: str, arg: str} ]
          static_args: List[ {param_name: value} ]

        msg_dict: Complete data dictionary for the current step, e.g.:
                  {"action": frame, "observation.state": ..., ...}
        """
        method_name = params.get("method_name")
        if not method_name:
            raise ValueError("[ModuleAdapter.pub] params.method_name is not configured")

        # Create/get instance and method
        instance, inst_key = self._create_or_get_instance(params)
        m = getattr(instance, method_name)

        dyn_list = params.get("dynamic_arg") or []
        static_list = params.get("static_args") or []

        if not isinstance(dyn_list, list):
            raise TypeError("[ModuleAdapter.pub] params.dynamic_arg must be a list")
        if not isinstance(static_list, list):
            raise TypeError("[ModuleAdapter.pub] params.static_args must be a list")

        call_kwargs: Dict[str, Any] = {}

        # 1. Process dynamic_arg: from_key -> arg
        for item in dyn_list:
            if not isinstance(item, dict):
                continue
            from_key = item.get("from_key")
            param_name = item.get("arg")
            if not from_key or not param_name:
                continue
            if from_key not in msg_dict:
                continue

            raw_val = msg_dict[from_key]
            val = self._convert_for_param(m, param_name, raw_val)
            call_kwargs[param_name] = val

        # 2. Process static_args: each element is {param_name: value}
        for item in static_list:
            if not isinstance(item, dict):
                continue
            for k, v in item.items():
                val = self._convert_for_param(m, k, v)
                call_kwargs[k] = val

        # 3. Call the target method (lock based on instance lock flag)
        self._call_with_instance_lock(inst_key, m, **call_kwargs)

    def stop(self) -> None:
        for k in list(self._running.keys()):
            self._running[k] = False

        for inst_key, inst in self._instances.items():
            params = self._instance_params.get(inst_key, {}) or {}
            destroy_call = params.get("destroy_call")

            if not destroy_call:
                # If destroy_call is not configured, try to call destroy() by default
                if hasattr(inst, "destroy") and callable(getattr(inst, "destroy")):
                    try:
                        # destroy also follows the instance lock logic to avoid conflicts with parallel calls
                        self._call_with_instance_lock(
                            inst_key, getattr(inst, "destroy")
                        )
                    except Exception as e:
                        logger.error(
                            f"[ModuleAdapter] Error calling default destroy() for instance {inst_key}: {e}"
                        )
            else:
                for call in destroy_call or []:
                    mname = call.get("method_name")
                    if not mname:
                        continue
                    mkwargs = call.get("method_kwargs", {}) or {}
                    try:
                        m = getattr(inst, mname)
                        # destroy also follows the instance lock logic to avoid conflicts with parallel calls
                        self._call_with_instance_lock(inst_key, m, **mkwargs)
                    except Exception as e:
                        logger.error(
                            f"[ModuleAdapter] Error calling destroy_call.{mname} for instance {inst_key}: {e}"
                        )

        logger.info(
            "[ModuleAdapter] All module threads have been stopped and destroy_call/default destroy() executed."
        )
