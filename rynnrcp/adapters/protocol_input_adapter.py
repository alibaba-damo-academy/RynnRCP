"""
Protocol input adapter.

Normalizes connector messages into ``(timestamp, {object_name: protocol_value})``.
Supports dotted field access, indexing, slicing, and wildcard extraction.
"""

from __future__ import annotations

import time
from typing import Any, Dict, List, Tuple

from .base_input_adapter import BaseInputAdapter


class ProtocolInputAdapter(BaseInputAdapter):
    """Adapter that extracts numeric fields from structured messages.

    Configuration (via *params* passed to ``__init__``):
      mappings : list of ``{field, object_name}`` dicts
        - *field*: dotted access path (e.g. ``position``, ``header.stamp.sec``,
          ``data[0:6]``, ``data[*]``)
        - *object_name*: key in the returned dict
      object_name  : protocol object name used when no mappings are configured
    """

    def __init__(self, params: Dict[str, Any] | None = None) -> None:
        if params is None:
            raise ValueError("ProtocolInputAdapter requires params")
        self._mappings: List[Dict[str, str]] = []
        self._object_name: str = str(params["object_name"])
        self._payload_mode: str | None = params.get("payload_mode")
        self._mappings = params.get("mappings") or []

    # ── field access ─────────────────────────────────────────────────

    @staticmethod
    def _get_field(obj: Any, path: str) -> Any:
        """Traverse *obj* following a dotted/indexed path.

        Supports:
        - ``name``            → getattr / dict key
        - ``name.sub``        → nested access
        - ``name[0]``         → index
        - ``name[0:6]``       → slice
        - ``name[*]``         → all elements (list())
        """
        for part in path.split("."):
            # Check for bracket access
            if "[" in part:
                attr, bracket = part.split("[", 1)
                bracket = bracket.rstrip("]")

                # Get the attribute first
                if attr:
                    obj = getattr(obj, attr) if hasattr(obj, attr) else obj[attr]

                if bracket == "*":
                    obj = list(obj)
                elif ":" in bracket:
                    parts = bracket.split(":")
                    start = int(parts[0]) if parts[0] else None
                    stop = int(parts[1]) if parts[1] else None
                    obj = obj[start:stop]
                else:
                    obj = obj[int(bracket)]
            else:
                if isinstance(obj, dict):
                    obj = obj[part]
                else:
                    obj = getattr(obj, part)
        return obj

    # ── timestamp extraction ─────────────────────────────────────────

    @staticmethod
    def _extract_timestamp(obj: Any) -> float:
        """Try to extract a timestamp from the message; fall back to wall clock."""
        # ROS2 style: header.stamp.sec + header.stamp.nanosec
        try:
            header = getattr(obj, "header", None)
            if header:
                stamp = getattr(header, "stamp", None)
                if stamp:
                    sec = getattr(stamp, "sec", 0)
                    nsec = getattr(stamp, "nanosec", 0)
                    return float(sec) + float(nsec) * 1e-9
        except Exception:
            pass

        # LCM style: header.stamp_sec + header.stamp_nanosec
        try:
            header = getattr(obj, "header", None)
            if header:
                sec = getattr(header, "stamp_sec", None)
                nsec = getattr(header, "stamp_nanosec", None)
                if sec is not None:
                    return float(sec) + float(nsec or 0) * 1e-9
        except Exception:
            pass

        # dict style
        if isinstance(obj, dict):
            if "timestamp" in obj:
                return float(obj["timestamp"])

        return time.time()

    # ── parse ────────────────────────────────────────────────────────

    def parse(self, msg: Any) -> Tuple[float, Dict[str, Any]]:
        ts = self._extract_timestamp(msg)
        result: Dict[str, Any] = {}

        if self._mappings:
            for m in self._mappings:
                field = m["field"]
                object_name = m["object_name"]
                val = self._get_field(msg, field)
                if isinstance(val, (list, tuple)):
                    val = [float(v) for v in val]
                elif hasattr(val, "__iter__") and not isinstance(val, (str, bytes, dict)):
                    val = [float(v) for v in val]
                result[object_name] = val
        else:
            if self._payload_mode == "protocol_json":
                if not isinstance(msg, dict) or not isinstance(msg.get("name"), str) or "value" not in msg:
                    raise ValueError("protocol_json input must be an object with name and value")
                result[str(msg["name"])] = msg["value"]
                return ts, result
            # No mappings → store entire msg
            if isinstance(msg, (list, tuple)):
                result[self._object_name] = [float(v) for v in msg]
            elif hasattr(msg, "__iter__") and not isinstance(msg, (str, bytes, dict)):
                result[self._object_name] = [float(v) for v in msg]
            else:
                result[self._object_name] = msg

        return ts, result
