"""Redaction and payload summarization helpers for safe logging.

Use ``redact()`` before logging config/request dicts that may hold
credentials, and ``describe_payload()`` instead of logging raw frames,
images, or long arrays.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

REDACTED = "***"

# Substring match against lower-cased key names.
SENSITIVE_KEY_MARKERS = (
    "token",
    "password",
    "passwd",
    "authorization",
    "cookie",
    "credential",
    "secret",
    "auth",
    "api_key",
    "apikey",
    "access_key",
    "private_key",
)

_MAX_INLINE_ITEMS = 8


def is_sensitive_key(key: Any) -> bool:
    name = str(key).lower()
    return any(marker in name for marker in SENSITIVE_KEY_MARKERS)


def redact(value: Any, *, max_depth: int = 6) -> Any:
    """Return a copy of *value* with credential-like fields masked."""
    if max_depth <= 0:
        return "..."
    if isinstance(value, Mapping):
        return {
            str(key): REDACTED if is_sensitive_key(key) else redact(item, max_depth=max_depth - 1)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [redact(item, max_depth=max_depth - 1) for item in value]
    return value


def describe_payload(value: Any) -> str:
    """Return a compact, log-safe description of an arbitrary payload.

    Never embeds raw bytes, text payloads, full images, or long arrays — only
    type, size/shape, and short numeric sequences.
    """
    if value is None:
        return "None"
    if isinstance(value, (bytes, bytearray, memoryview)):
        return f"<{type(value).__name__} len={len(value)}>"
    shape = getattr(value, "shape", None)
    dtype = getattr(value, "dtype", None)
    if shape is not None and dtype is not None:  # numpy-like array
        return f"<array shape={tuple(shape)} dtype={dtype}>"
    if isinstance(value, str):
        return f"<str len={len(value)}>"
    if isinstance(value, Mapping):
        keys = list(value.keys())
        head = ", ".join(str(key) for key in keys[:_MAX_INLINE_ITEMS])
        suffix = ", ..." if len(keys) > _MAX_INLINE_ITEMS else ""
        return f"<dict len={len(keys)} keys=[{head}{suffix}]>"
    if isinstance(value, Sequence):
        if len(value) <= _MAX_INLINE_ITEMS and all(
            isinstance(item, (int, float, bool)) for item in value
        ):
            return repr(list(value))
        return f"<{type(value).__name__} len={len(value)}>"
    return f"<{type(value).__name__}>"
