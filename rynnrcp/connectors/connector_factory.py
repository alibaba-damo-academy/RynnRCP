"""
Connector factory and dispatcher.

Instantiates the connector types required by one Runtime/RunnerManager instance
and routes pub/sub calls through them.
"""

from __future__ import annotations

import logging
import os
import sys
from typing import Any, Callable, Dict, Iterable, List

from rynnrcp.utils.imports import import_object

from .base import BaseConnector

logger = logging.getLogger(__name__)

# Lazy availability flags – evaluated on first use
_availability_cache: Dict[str, tuple[bool, str]] = {}
_python_only_connectors = {"module", "port"}


def _check_availability(name: str) -> bool:
    """Check runtime availability of a connector type."""
    return _check_availability_detail(name)[0]


def _check_availability_detail(name: str) -> tuple[bool, str]:
    """Check runtime availability and return a human-readable diagnostic."""
    if name in _availability_cache:
        return _availability_cache[name]

    available = False
    reason = "unknown connector"
    if name == "ros2":
        available, reason = _check_ros2_availability()
    elif name == "lcm":
        available, reason = _check_lcm_availability()
    elif name in _python_only_connectors:
        available = True  # always available (pure Python)
        reason = "pure Python connector"

    _availability_cache[name] = (available, reason)
    return available, reason


def _check_ros2_availability() -> tuple[bool, str]:
    try:
        import rclpy  # noqa: F401
        return True, "rclpy import succeeded"
    except ImportError as exc:
        return (
            False,
            (
                f"Python package 'rclpy' is not importable: {exc}. "
                "Run in a ROS2 Python environment and source the ROS2 setup file "
                "(for example: source /opt/ros/<distro>/setup.bash)."
            ),
        )


def _check_lcm_availability() -> tuple[bool, str]:
    try:
        import lcm  # noqa: F401
        return True, "lcm import succeeded"
    except ImportError as exc:
        return (
            False,
            (
                f"Python package 'lcm' is not importable: {exc}. "
                "Install LCM Python bindings and ensure the active Python environment can import lcm."
            ),
        )


def _create_ros2_connector(node_name: str, init_args: Dict[str, Any]) -> BaseConnector:
    from .ros2_connector import ROS2Connector

    init_args = dict(init_args)
    init_args.setdefault("node_name", node_name)
    return ROS2Connector(**init_args)


def _create_lcm_connector(_node_name: str, init_args: Dict[str, Any]) -> BaseConnector:
    from .lcm_connector import LCMConnector

    return LCMConnector(**dict(init_args))


def _create_module_connector(_node_name: str, init_args: Dict[str, Any]) -> BaseConnector:
    from .module_connector import ModuleConnector

    return ModuleConnector(**dict(init_args))


def _create_port_connector(_node_name: str, init_args: Dict[str, Any]) -> BaseConnector:
    from .port_connector import PortConnector

    return PortConnector(**dict(init_args))


_CONNECTOR_BUILDERS = {
    "ros2": _create_ros2_connector,
    "lcm": _create_lcm_connector,
    "module": _create_module_connector,
    "port": _create_port_connector,
}


class ConnectorFactory:
    """Top-level connector factory.

    - Receives the connector names required by the current Runtime
    - Intersects those names with runtime availability
    - Instantiates only needed connectors
    - Provides unified pub / sub / stop
    """

    def __init__(
        self,
        node_name: str = "rynnrcp",
        *,
        connector_names: Iterable[str],
        connectors_config: Dict[str, Any] | None = None,
    ):
        connector_configs = dict(connectors_config or {})
        used = {str(name).lower() for name in connector_names}
        self.connectors: Dict[str, BaseConnector] = {}

        for name in sorted(used):
            connector_cfg = connector_configs.get(name, {})
            has_custom_class = isinstance(connector_cfg, dict) and bool(connector_cfg.get("class"))
            available, reason = _check_availability_detail(name)
            if not has_custom_class and not available:
                logger.warning(
                    "Connector '%s' requested but not available - skipping. %s "
                    "(python=%s, executable=%s, cwd=%s)",
                    name,
                    reason,
                    sys.version.split()[0],
                    sys.executable,
                    os.getcwd(),
                )
                continue
            self.connectors[name] = self._create_connector(name, node_name, connector_configs)

        self.available: List[str] = sorted(self.connectors.keys())

    # ------------------------------------------------------------------
    @staticmethod
    def _create_connector(name: str, node_name: str, config: Dict) -> BaseConnector:
        connector_cfg = _connector_config(config, name)

        cls_path = connector_cfg.get("class")
        init_args = dict(connector_cfg.get("init_args") or {})

        if cls_path:
            cls = import_object(cls_path)
            return cls(**init_args)
        builder = _CONNECTOR_BUILDERS.get(name)
        if builder is None:
            raise ValueError(f"Unknown connector type: {name}")
        return builder(node_name, init_args)

    # ------------------------------------------------------------------
    def get_connector(self, name: str) -> BaseConnector:
        name = name.lower()
        conn = self.connectors.get(name)
        if conn is None:
            raise ValueError(
                f"Connector '{name}' is not enabled. Available: {self.available}"
            )
        return conn

    def pub(self, connector: str, params: Dict[str, Any], msg: Any) -> None:
        self.get_connector(connector).pub(params, msg)

    def sub(self, connector: str, params: Dict[str, Any], callback: Callable) -> None:
        self.get_connector(connector).sub(params, callback)

    def stop(self) -> None:
        for name, conn in self.connectors.items():
            try:
                conn.stop()
            except Exception as e:
                logger.error("Error stopping connector '%s': %s", name, e)


def _connector_config(config: Dict, name: str) -> Dict[str, Any]:
    connector_cfg = config.get(name, {})
    if connector_cfg is True:
        connector_cfg = {}
    if not isinstance(connector_cfg, dict):
        raise TypeError(f"connectors.{name} must be a dict")
    return connector_cfg
