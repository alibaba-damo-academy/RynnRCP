"""Small shared utilities used by RynnRCP runtime modules."""

from __future__ import annotations

import math
import re
from typing import Any


def camel_to_snake(name: str) -> str:
    s1 = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", str(name))
    return re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s1).lower()


def safe_name(value: str) -> str:
    value = str(value).strip().replace("/", "_")
    return re.sub(r"[^0-9a-zA-Z._-]+", "_", value).strip("_") or "stream"


def coerce_timestamp(value: Any, *, context: str) -> float:
    if value is None or value == "":
        raise ValueError(f"invalid timestamp for {context}: empty")
    try:
        ts = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid timestamp for {context}: {value!r}") from exc
    if not math.isfinite(ts):
        raise ValueError(f"invalid timestamp for {context}: non-finite {value!r}")
    return ts
