"""Stable, privacy-preserving identifiers derived from the host MAC address."""

from __future__ import annotations

import hashlib
import re
import uuid
from typing import Any


_GENERATED_SUFFIX = re.compile(r"_[0-9a-fA-F]{8}$")


def machine_mac_suffix() -> str:
    """Return an eight-character host suffix without exposing the raw MAC."""
    mac = f"{uuid.getnode() & ((1 << 48) - 1):012x}"
    return hashlib.sha256(mac.encode("ascii")).hexdigest()[:8]


def with_machine_suffix(value: Any, default_base: str, suffix: str) -> str:
    """Add or replace an eight-character generated suffix on an identifier."""
    base = str(value or default_base).strip() or str(default_base)
    base = _GENERATED_SUFFIX.sub("", base)
    return f"{base}_{suffix}"
