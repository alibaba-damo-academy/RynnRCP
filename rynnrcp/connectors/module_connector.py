"""
Module connector – in-process Python module integration.

Ported from RynnRCP ModuleAdapter. Treats arbitrary Python classes/functions
as a connector, driven entirely by configuration.

Subscription mode:
    Periodically calls a method on a lazily-created Python instance and
    forwards the result to a callback.

Publish mode:
    Performs a single method call with dynamic + static arguments.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Dict, Tuple

from rynnrcp.utils.imports import apply_sys_path, import_object, normalize_sys_path

from .base import BaseConnector

logger = logging.getLogger(__name__)

_MIN_POLL_INTERVAL_S = 0.001
_THREAD_JOIN_TIMEOUT_S = 1.0


class ModuleConnector(BaseConnector):
    """In-process Python module connector.

    Params convention (shared by sub and pub):
      module_name   : str           – import path (e.g. ``pkg.mod.Camera``)
      init_args     : dict          – kwargs for constructor
      sys_path      : str|list[str] – extra sys.path entries
      start_call    : list[dict]    – post-init methods ``[{method_name, method_kwargs}]``
      destroy_call  : list[dict]    – pre-destroy methods

    Sub-only:
      method_name   : str   – method to poll (default ``read``)
      method_kwargs : dict  – kwargs for the method
      interval      : float – sleep between calls (seconds)

    Pub-only:
      method_name : str
      receives the protocol action value as its only argument.
    """

    def __init__(self) -> None:
        self._threads: Dict[str, threading.Thread] = {}
        self._running: Dict[str, bool] = {}
        self._instances: Dict[str, Any] = {}
        self._instance_params: Dict[str, Dict[str, Any]] = {}
        self._pub_plans: Dict[str, Callable[..., Any]] = {}
        self._scheduled_components: Dict[str, tuple[Any, str]] = {}
        self._last_error_log_at: Dict[str, float] = {}

    # ── helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _instance_key(
        module_name: str,
        init_args: Dict[str, Any],
        config_class: str | None = None,
        sys_path: tuple[str, ...] | None = None,
    ) -> str:
        return (
            module_name
            + "::"
            + str(config_class or "")
            + "::"
            + repr(tuple(sys_path or ()))
            + "::"
            + repr(sorted(init_args.items()))
        )

    def get_instance(self, params: Dict[str, Any]) -> Any:
        """Public getter – create-or-reuse an instance."""
        inst, _ = self._create_or_get_instance(params)
        return inst

    def _create_or_get_instance(
        self, params: Dict[str, Any]
    ) -> Tuple[Any, str]:
        module_name = params.get("module_name")
        if not module_name:
            raise ValueError("params.module_name is required")

        # sys.path handling
        extra = normalize_sys_path(params.get("sys_path"))
        apply_sys_path(extra)

        init_args = params.get("init_args") or {}
        config_class = params.get("config_class")
        key = self._instance_key(module_name, init_args, config_class, extra)

        # existing instance
        if key in self._instances:
            self._instance_params[key] = _merge_instance_params(
                self._instance_params.get(key) or {},
                params,
            )
            return self._instances[key], key

        # create new
        factory = import_object(module_name)
        logger.info("Creating module connector instance: module=%s init_args=%s", module_name, init_args)
        if config_class:
            config_cls = import_object(config_class)
            instance = factory(config_cls(**init_args))
        else:
            instance = factory(**init_args)

        # start_call lifecycle
        for call in params.get("start_call") or []:
            if not isinstance(call, dict):
                continue
            mname = call.get("method_name")
            if mname:
                started_at = time.monotonic()
                logger.info(
                    "Calling module lifecycle start: module=%s method=%s",
                    module_name,
                    mname,
                )
                getattr(instance, mname)(**(call.get("method_kwargs") or {}))
                logger.info(
                    "Module lifecycle start completed: module=%s method=%s duration=%.3fs",
                    module_name,
                    mname,
                    time.monotonic() - started_at,
                )

        self._instances[key] = instance
        self._instance_params[key] = params

        return instance, key

    def _start_scheduler_subscription(
        self,
        *,
        scheduler: Any,
        component_name: str,
        key: str,
        instance: Any,
        method_name: str,
        method_kwargs: Dict[str, Any],
        interval: float,
        callback: Callable,
    ) -> None:
        from rynnrcp.runtime.scheduler import Component

        method = getattr(instance, method_name)

        def tick() -> None:
            if not self._running.get(key, False):
                return
            callback(method(**method_kwargs))

        scheduler.add_component(
            Component(
                str(component_name),
                period_ms=interval * 1000.0,
                callback=tick,
            )
        )
        self._scheduled_components[key] = (scheduler, str(component_name))

    def _start_thread_subscription(
        self,
        *,
        key: str,
        instance: Any,
        method_name: str,
        method_kwargs: Dict[str, Any],
        interval: float,
        callback: Callable,
        log_message: str,
    ) -> None:
        def loop() -> None:
            method = getattr(instance, method_name)
            while self._running.get(key, False):
                try:
                    callback(method(**method_kwargs))
                except Exception as e:
                    self._log_loop_error(key, log_message, e)
                    time.sleep(0.01)
                time.sleep(interval)

        th = threading.Thread(target=loop, daemon=True)
        th.start()
        self._threads[key] = th

    # ── sub / pub ────────────────────────────────────────────────────

    def sub(self, params: Dict[str, Any], callback: Callable) -> None:
        method_name = params.get("method_name", "read")
        method_kwargs = params.get("method_kwargs") or {}
        interval = max(float(params.get("interval", 0.0)), _MIN_POLL_INTERVAL_S)

        instance, instance_key = self._create_or_get_instance(params)
        key = _subscription_key(instance_key, params, method_name, method_kwargs)
        self._running[key] = True
        scheduler = params.get("_scheduler")
        component_name = params.get("_component_name") or f"module:{key}"

        if scheduler is not None:
            self._start_scheduler_subscription(
                scheduler=scheduler,
                component_name=str(component_name),
                key=key,
                instance=instance,
                method_name=method_name,
                method_kwargs=method_kwargs,
                interval=interval,
                callback=callback,
            )
            return

        self._start_thread_subscription(
            key=key,
            instance=instance,
            method_name=method_name,
            method_kwargs=method_kwargs,
            interval=interval,
            callback=callback,
            log_message=(
                "ModuleConnector sub error "
                f"(module={params.get('module_name')}, method={method_name})"
            ),
        )

    def pub(self, params: Dict[str, Any], msg_dict: Any) -> None:
        method = self._get_pub_method(params)
        method(_single_protocol_value(msg_dict))

    def _get_pub_method(self, params: Dict[str, Any]) -> Callable[..., Any]:
        method_name = params.get("method_name")
        if not method_name:
            raise ValueError("params.method_name is required for pub")

        instance, key = self._create_or_get_instance(params)
        plan_key = f"{key}::{method_name}"
        cached = self._pub_plans.get(plan_key)
        if cached is not None:
            return cached

        method = getattr(instance, method_name)
        self._pub_plans[plan_key] = method
        return method

    # ── stop ─────────────────────────────────────────────────────────

    def stop(self) -> None:
        for k in list(self._running):
            self._running[k] = False

        for _key, (scheduler, component_name) in list(self._scheduled_components.items()):
            try:
                scheduler.remove_component(component_name)
            except Exception as exc:
                logger.debug("ModuleConnector scheduler cleanup ignored (%s): %s", component_name, exc, exc_info=True)
        self._scheduled_components.clear()
        self._pub_plans.clear()

        for key, thread in list(self._threads.items()):
            if thread.is_alive():
                thread.join(timeout=_THREAD_JOIN_TIMEOUT_S)
                if thread.is_alive():
                    logger.warning(
                        "ModuleConnector thread did not stop within %.1fs: %s",
                        _THREAD_JOIN_TIMEOUT_S,
                        key,
                    )
        self._threads.clear()

        for key, inst in list(self._instances.items()):
            params = self._instance_params.get(key) or {}
            for call in params.get("destroy_call") or []:
                if not isinstance(call, dict):
                    continue
                mname = call.get("method_name")
                if not mname:
                    continue
                try:
                    getattr(inst, mname)(**(call.get("method_kwargs") or {}))
                except Exception as exc:
                    logger.error(
                        "ModuleConnector destroy error (%s.%s): %s",
                        key,
                        mname,
                        exc,
                        exc_info=True,
                    )

        self._running.clear()
        self._instances.clear()
        self._instance_params.clear()
        self._last_error_log_at.clear()

        logger.info("ModuleConnector stopped all instances")

    def _log_loop_error(self, key: str, message: str, exc: Exception) -> None:
        now = time.monotonic()
        last = self._last_error_log_at.get(key, 0.0)
        if now - last < 2.0:
            return
        self._last_error_log_at[key] = now
        logger.warning("%s instance=%s: %s", message, key, exc, exc_info=True)


def _merge_instance_params(existing: Dict[str, Any], incoming: Dict[str, Any]) -> Dict[str, Any]:
    """Merge params for a shared module instance without losing lifecycle calls."""
    merged = dict(existing)
    merged.update(incoming)
    merged["start_call"] = _merge_call_lists(
        existing.get("start_call") or [],
        incoming.get("start_call") or [],
    )
    merged["destroy_call"] = _merge_call_lists(
        existing.get("destroy_call") or [],
        incoming.get("destroy_call") or [],
    )
    return merged


def _single_protocol_value(msg: Any) -> Any:
    if not isinstance(msg, dict):
        return msg
    action_items = [(key, value) for key, value in msg.items() if str(key) != "action_type"]
    if len(action_items) != 1:
        raise ValueError("module action message must contain exactly one protocol action value")
    return action_items[0][1]


def _subscription_key(
    instance_key: str,
    params: Dict[str, Any],
    method_name: Any,
    method_kwargs: Dict[str, Any],
) -> str:
    return (
        instance_key
        + "::sub::"
        + str(method_name)
        + "::"
        + str(params.get("object_name") or "")
        + "::"
        + str(params.get("channel") or "")
        + "::"
        + repr(sorted(method_kwargs.items()))
    )


def _merge_call_lists(existing: list[Any], incoming: list[Any]) -> list[Any]:
    merged: list[Any] = []
    seen: set[tuple[str, str]] = set()
    for call in list(existing) + list(incoming):
        if not isinstance(call, dict):
            continue
        method_name = str(call.get("method_name", ""))
        method_kwargs = call.get("method_kwargs") or {}
        key = (method_name, repr(sorted(method_kwargs.items())))
        if method_name and key not in seen:
            merged.append(call)
            seen.add(key)
    return merged
