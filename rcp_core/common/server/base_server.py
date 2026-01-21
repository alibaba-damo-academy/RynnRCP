# rcp_core/common/server/base_server.py

"""
Base server infrastructure (input subscription + buffering + compose helpers).
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.server.base_server.BaseServer`, a common
base class for servers that:
- subscribe to configured inputs via :class:`~rcp_core.common.protocol.protocol_factory.ProtocolFactory`
- parse incoming messages using registered input adapters
- store parsed results into a shared, per-server-partitioned :class:`~rcp_core.common.utils.global_buffer.GlobalBuffer`
- optionally “compose” multiple buffered keys into derived outputs (e.g. concatenated arrays)

Key concepts
------------
Per-server buffer partitioning:
- GlobalBuffer stores data as ``{server_name: {key: Deque[(ts, value)]}}``.
- Each BaseServer instance reads/writes only within its own ``server_name`` partition
  (via ``_push_to_buffer`` and ``snapshot(server_name)``).

Lifecycle / wiring:
- :meth:`bind_bus` attaches an :class:`~rcp_core.common.bus.rcp_bus.RcpBus` for tool calls.
- :meth:`bind_adapter` attaches a :class:`~rcp_core.common.protocol.protocol_factory.ProtocolFactory`
  and subscribes to all configured inputs for this server.

Input flow
----------
For each configured input:
- a callback is created by :meth:`_create_callback_with_params`
- the callback instantiates the configured input adapter from
  :data:`~rcp_core.common.adapter.INPUT_ADAPTER_REGISTRY`
- on each received message, ``adapter.parse(msg)`` returns ``(ts, parsed_dict)``
- each ``(key, value)`` in ``parsed_dict`` is pushed into the GlobalBuffer under the
  current server’s namespace

Compose helpers
---------------
:meth:`run_compose_with_cfg` builds derived outputs from time-aligned inputs using the
server’s ``compose`` configuration. Currently supported compose type:
- ``type: "array"`` — concatenates multiple ``array('d')`` inputs into a single
  ``array('d')`` stored under ``out_key``.

Thread safety
-------------
- ``buffer_lock`` is used to serialize writes from callbacks into this server’s buffer
  partition (GlobalBuffer itself performs the actual push).
"""

import threading
from array import array
from collections import deque
from typing import Any, Callable, Dict, List, Deque, Tuple, Optional

from ..adapter import INPUT_ADAPTER_REGISTRY
from ..bus.rcp_bus import RcpBus
from ..protocol.protocol_factory import ProtocolFactory
from ..utils.global_buffer import GlobalBuffer
from rcp_core.common.utils.logger import server_logger

logger = server_logger()


class BaseServer:
    """Base server class."""

    def __init__(
        self,
        config: Dict,
        server_name: str,
        queue_len: Optional[int] = 100,
        expire_seconds: float = 0.5,
    ):
        """
        Parameters
        ----------
        queue_len : int | None
            Max queue length per key.
        expire_seconds : float
            Data expiration time in seconds.

        Global buffer is partitioned by server_name:
          GlobalBuffer stores: { server_name: { key: Deque[(ts, value)] } }
        """
        self.server_name = server_name

        # Global buffer singleton
        self._global_buffer = GlobalBuffer.get(
            queue_len=queue_len, expire_seconds=expire_seconds
        )

        # This server's own buffer view (from snapshot), read-only reference
        self.buffer: Dict[str, Deque[Tuple[float, Any]]] = self._global_buffer.snapshot(
            self.server_name
        )
        # Lock for this server only: protects access to this server's own data
        self.buffer_lock = threading.Lock()

        self.robot_type = config["robot_type"]
        self.server_config = next(
            s for s in config["servers"] if s["name"] == server_name
        )
        self.bus: Optional[RcpBus] = None
        self.protocol_factory: Optional[ProtocolFactory] = None

        # Compose config (for this server only)
        self.compose_cfg: List[Dict] = self.server_config.get("compose", [])

        self.max_queue_len: Optional[int] = queue_len
        self.expire_seconds: float = expire_seconds

    # Buffer interfaces: per-server view + global view

    def _push_to_buffer(self, key: str, ts: float, value: Any) -> None:
        """Append a record to this server's buffer[key]."""
        # Actual push logic is handled by GlobalBuffer, partitioned by server_name
        self._global_buffer.push(self.server_name, key, ts, value)

    def get_buffer(self) -> Dict[str, Deque[Tuple[float, Any]]]:
        """
        Get this server's buffer (partitioned by server_name), with expiration
        cleanup applied before returning.

        Structure: Dict[str, Deque[(ts, value)]]
        """
        # Refresh self.buffer reference to the latest internal structure
        # (snapshot returns an internal reference)
        self.buffer = self._global_buffer.snapshot(self.server_name)
        return self.buffer

    def get_buffer_global(
        self,
    ) -> Dict[str, Dict[str, Deque[Tuple[float, Any]]]]:
        """
        Get the global buffer view across all servers:
          { server_name: { key: Deque[(ts, value)] } }

        Note:
          This is a shallow snapshot view. Upper layers should treat it as
          read-only (iterate, read deque contents), and must not modify the
          structure (do not add/remove keys).
        """
        return self._global_buffer.snapshot_all()

    # Compose-related: static functions

    @staticmethod
    def run_compose_with_cfg(
        aligned: List[Tuple[str, float, Any]],
        compose_cfg: List[Dict[str, Any]],
        target_out_keys: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        """
        Apply compose_cfg to aligned results and return {out_key: composed_value}.

        Parameters
        ----------
        aligned : List[(key, ts, value)]
            Aligned input data.
        compose_cfg : List[Dict[str, Any]]
            Compose configuration from a server's config.
        target_out_keys : list[str] | None
            If None, run all entries in compose_cfg.
            If a list, only run entries whose out_key is in this list.
        """
        results: Dict[str, Any] = {}

        if not compose_cfg or not aligned:
            return results

        # If target_out_keys is specified, filter entries by out_key
        filtered_cfg = compose_cfg
        if target_out_keys:
            targets = set(target_out_keys)
            filtered_cfg = [c for c in compose_cfg if c.get("out_key") in targets]

        if not filtered_cfg:
            return results

        # Convert to key -> value for easier lookup
        aligned_dict: Dict[str, Any] = {k: v for k, ts, v in aligned}

        for comp in filtered_cfg:
            out_key = comp.get("out_key")
            ctype = comp.get("type", "array")
            from_keys = comp.get("from_keys", [])

            if not out_key or not from_keys:
                continue

            # Check whether all from_keys are present in aligned_dict
            missing = [k for k in from_keys if k not in aligned_dict]
            if missing:
                logger.info(
                    f"[Compose] Skipping out_key='{out_key}', missing in aligned: {missing}"
                )
                continue

            # Compose by type
            if ctype == "array":
                composed = array("d")
                ok = True
                for k in from_keys:
                    v = aligned_dict[k]
                    if isinstance(v, array) and v.typecode == "d":
                        composed.extend(v)
                    else:
                        logger.warning(
                            f"[Compose] Skipping out_key='{out_key}', "
                            f"from_key='{k}' type not supported: {type(v)}"
                        )
                        ok = False
                        break
                if not ok:
                    continue

                results[out_key] = composed

            else:
                logger.warning(
                    f"[Compose] Unknown compose type='{ctype}', not processing out_key='{out_key}'"
                )
                continue

        return results

    # Subscription callbacks & adapter binding

    def _create_callback_with_params(self, adapter_cfg: Dict) -> Callable:
        """
        Generic callback creation:
        - adapter_cfg: A complete input configuration containing protocol/adapter/params, etc.
        - The adapter extracts what it needs from adapter_cfg["params"]
        """
        adapter_name = adapter_cfg.get("adapter")
        msg_adapter_cls = INPUT_ADAPTER_REGISTRY.get(adapter_name)
        if msg_adapter_cls is None:
            raise ValueError(
                f"[BaseServer] Input message adapter '{adapter_name}' not found in INPUT_ADAPTER_REGISTRY"
            )

        msg_adapter = msg_adapter_cls(adapter_cfg)
        params = adapter_cfg.get("params", {})
        debug_source = params.get("topic") or params.get("module_name") or adapter_name

        def callback(msg: Any):
            try:
                ts, parsed = msg_adapter.parse(msg)
            except Exception as e:
                logger.error(
                    f"[BaseServer] Failed to parse message, source={debug_source}: {e}"
                )
                return

            if ts == -1:
                logger.error(
                    f"[BaseServer] Invalid message, skipping, source={debug_source}"
                )
                return

            with self.buffer_lock:
                for k, v in parsed.items():
                    self._push_to_buffer(k, ts, v)

        return callback

    def bind_bus(self, bus: RcpBus) -> None:
        """Bind an RcpBus instance for tool calls."""
        self.bus = bus

    def bind_adapter(self, protocol_factory: ProtocolFactory) -> None:
        """Bind middleware adapter and subscribe to all inputs defined in server_config."""
        self.protocol_factory = protocol_factory

        for inp in self.server_config.get("inputs", []):
            protocol = inp["protocol"].lower()
            adapter_cfg = inp
            params = inp.get("params", {}) or {}

            callback = self._create_callback_with_params(adapter_cfg)

            self.protocol_factory.sub(protocol, params, callback)
