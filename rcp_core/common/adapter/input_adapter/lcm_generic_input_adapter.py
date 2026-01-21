# rcp_core/common/adapter/lcm_generic_input_adapter.py

"""
Generic LCM input adapter with configurable field mappings.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.adapter.lcm_generic_input_adapter.LcmGenericInputAdapter`,
a :class:`~rcp_core.common.adapter.base_input_adapter.BaseInputAdapter` implementation that
extracts a timestamp and selected fields from an LCM message.

The adapter reads the message timestamp from ``msg.header.stamp_sec`` and
``msg.header.stamp_nanosec`` (seconds + nanoseconds). Payload fields are extracted
according to a mapping configuration that supports:

- dotted attribute access (e.g. ``header.stamp_sec``)
- index/slice access (e.g. ``position[0]``, ``position[0:6]``)
- the special selector ``"*"`` to forward the entire message

Extracted numeric values are normalized into ``array('d')`` for downstream
processing, while non-numeric values are passed through unchanged.
"""

from typing import Any, Dict, List, Tuple
from array import array

from .base_input_adapter import BaseInputAdapter
from rcp_core.common.utils.logger import server_logger

logger = server_logger()


class LcmGenericInputAdapter(BaseInputAdapter):
    """
    Generic LCM input adapter:
      - Extracts timestamp ts (float seconds) from msg.header.stamp_sec / stamp_nanosec
      - Supports mappings syntax:
          - "position"          -> msg.position
          - "position[0]"       -> msg.position[0]
          - "header.stamp_sec"  -> msg.header.stamp_sec
          - "*"                 -> msg
      - parse returns: (ts, parsed_dict)
    """

    def __init__(self, adapter_cfg: Dict):
        super().__init__(adapter_cfg)
        params = adapter_cfg.get("params", adapter_cfg)
        self.mappings: List[Dict[str, str]] = params.get("mappings", [])

    @staticmethod
    def _get_field_value(msg: Any, field_expr: str):
        """Get value from nested object/sequence according to field expression."""
        if field_expr == "*":
            return msg

        cur = msg
        # Convert index notation a[0].b into tokens: ["a", "[0]", "b"]
        tokens = field_expr.replace("[", ".[").split(".")
        for tok in tokens:
            if not tok:
                continue
            if tok.startswith("[") and tok.endswith("]"):
                idx_str = tok[1:-1]
                # Support slicing [start:stop]
                if ":" in idx_str:
                    start_str, stop_str = idx_str.split(":", 1)
                    start = int(start_str) if start_str else None
                    stop = int(stop_str) if stop_str else None
                    cur = cur[start:stop]
                else:
                    idx = int(idx_str)
                    cur = cur[idx]
            else:
                cur = getattr(cur, tok)
        return cur

    @staticmethod
    def _extract_timestamp_from_header(msg: Any) -> float:
        """Extract timestamp (seconds) from msg.header.stamp_sec / msg.header.stamp_nanosec."""
        try:
            header = getattr(msg, "header")
            sec = getattr(header, "stamp_sec")
            nanosec = getattr(header, "stamp_nanosec")
        except AttributeError as e:
            raise ValueError(
                f"[LcmGenericInputAdapter] Missing header.stamp_sec / stamp_nanosec in message: {e}"
            )

        return float(sec) + float(nanosec) * 1e-9

    def parse(self, msg: Any) -> Tuple[float, Dict[str, Any]]:
        """Return (ts, out_dict) where fields are extracted per mappings."""
        ts = self._extract_timestamp_from_header(msg)

        out: Dict[str, Any] = {}
        for m in self.mappings:
            field_name, out_key = m["field"], m["out_key"]
            try:
                val = self._get_field_value(msg, field_name)
            except Exception as e:
                logger.warning(
                    f"[LcmGenericInputAdapter] Failed to parse field='{field_name}': {e}"
                )
                continue

            # Normalize to unified data types
            if isinstance(val, (list, tuple)):
                # Convert list/tuple to array('d')
                val = array("d", [float(x) for x in val])
            elif isinstance(val, (int, float)):
                # Convert scalar into length-1 array
                val = array("d", [float(val)])
            # Other types (e.g. '*' → whole msg) are kept as-is

            out[out_key] = val

        return ts, out
