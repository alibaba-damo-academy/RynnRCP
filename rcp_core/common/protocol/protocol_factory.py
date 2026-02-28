# rcp_core/common/protocol/protocol_factory.py

"""
Protocol adapter factory and dispatcher.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.protocol.protocol_factory.ProtocolFactory`,
a singleton responsible for instantiating and managing middleware protocol adapters
based on what the current configuration actually uses.

Responsibilities
----------------
- Scans ``config.servers[*].inputs`` and ``config.servers[*].outputs`` to collect the set
  of requested protocols (normalized to lowercase).
- Intersects the requested protocols with runtime availability flags
  (``ROS2_AVAILABLE``, ``LCM_AVAILABLE``, ``MODULE_AVAILABLE``, ``PORT_AVAILABLE``).
- Instantiates the corresponding adapters (e.g. :class:`ROS2Adapter`, :class:`LCMAdapter`,
  :class:`ModuleAdapter`, :class:`PortAdapter`) and exposes a unified interface.

Public API
----------
- :meth:`ProtocolFactory.pub(protocol, params, msg)`:
  dispatch publish calls to the correct protocol adapter.
- :meth:`ProtocolFactory.sub(protocol, params, callback)`:
  dispatch subscribe calls to the correct protocol adapter.
- :meth:`ProtocolFactory.stop()`:
  stops all initialized adapters, logging errors if shutdown fails.

Notes
-----
- Implements a basic singleton pattern via ``__new__`` and an ``_initialized`` guard, so
  only one ProtocolFactory/adapters set is created per process.
- Raises a clear error if a caller tries to use a protocol that is not enabled or not
  available in the current environment.
"""

import threading
from typing import Any, Callable, Dict, List, Set

from .base_protocol_adapter import BaseProtocolAdapter
from .ros2_adapter import ROS2_AVAILABLE, ROS2Adapter
from .lcm_adapter import LCM_AVAILABLE, LCMAdapter
from .module_adapter import MODULE_AVAILABLE, ModuleAdapter
from .port_adapter import PORT_AVAILABLE, PortAdapter

from rcp_core.common.utils.logger import server_logger

logger = server_logger()


class ProtocolFactory:
    """
    Top-level protocol factory:
    - Parses servers[*].inputs/outputs[*].protocol from config
    - Intersects with actually available protocols
    - Initializes protocol adapters (ROS2Adapter / LCMAdapter) for the intersection
    - Provides unified pub / sub / stop interfaces
    """

    _instance = None
    _lock = threading.Lock()

    def __new__(cls, node_name: str = "none_name", config: Dict = None):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._initialized = False
        return cls._instance

    def __init__(self, node_name: str = "protocol_name", config: Dict = None):
        if self._initialized:
            return

        if config is None:
            raise ValueError("[ProtocolFactory] Missing configuration file")

        # 1. Collect all protocols used in config
        used_protocols = self._collect_protocols_from_config(config)

        # 2. Intersect with actually available protocols and initialize adapters
        self.protocol_adapters: Dict[str, BaseProtocolAdapter] = {}

        if "ros2" in used_protocols and ROS2_AVAILABLE:
            self.protocol_adapters["ros2"] = ROS2Adapter(node_name=node_name)

        if "lcm" in used_protocols and LCM_AVAILABLE:
            self.protocol_adapters["lcm"] = LCMAdapter()

        if "module" in used_protocols and MODULE_AVAILABLE:
            self.protocol_adapters["module"] = ModuleAdapter()

        if "port" in used_protocols and PORT_AVAILABLE:
            self.protocol_adapters["port"] = PortAdapter()

        self.available_protocols: List[str] = list(self.protocol_adapters.keys())

        self._initialized = True

    def _collect_protocols_from_config(self, config: Dict) -> Set[str]:
        """Collect all protocols from config.servers[*].inputs/outputs[*].protocol, normalized to lowercase."""
        protocols: Set[str] = set()

        servers = config.get("servers", [])
        for server in servers:
            # inputs
            for inp in server.get("inputs", []) or []:
                proto = inp.get("protocol")
                if proto:
                    protocols.add(str(proto).lower())

            # outputs
            for out in server.get("outputs", []) or []:
                proto = out.get("protocol")
                if proto:
                    protocols.add(str(proto).lower())

        return protocols

    def get_adapter(self, protocol: str) -> BaseProtocolAdapter:
        """According to protocol, get the protocol adapter."""
        protocol = protocol.lower()
        adapter = self.protocol_adapters.get(protocol)
        if adapter is None:
            raise ValueError(
                f"[ProtocolFactory] Protocol '{protocol}' is not enabled or available, "
                f"currently available: {list(self.protocol_adapters.keys())}"
            )
        return adapter

    def pub(self, protocol: str, params: Dict[str, Any], msg: Any) -> None:
        """Publish a message to the given protocol and params."""
        adapter = self.get_adapter(protocol)
        adapter.pub(params, msg)

    def sub(
        self,
        protocol: str,
        params: Dict[str, Any],
        callback: Callable,
    ) -> None:
        """Subscribe to a topic for the given protocol."""
        adapter = self.get_adapter(protocol)
        adapter.sub(params, callback)

    def stop(self) -> None:
        """Stop all initialized protocol adapters."""
        for proto, adapter in self.protocol_adapters.items():
            try:
                adapter.stop()
            except Exception as e:
                logger.error(
                    f"[ProtocolFactory] Error shutting down protocol '{proto}': {e}"
                )
