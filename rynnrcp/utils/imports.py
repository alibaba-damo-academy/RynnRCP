"""Dynamic import helpers."""

from __future__ import annotations

import importlib
import sys
from functools import lru_cache
from typing import Any


def import_object(path: str) -> Any:
    """Import and return an object from ``package.module.Object``."""
    module_name, attr_name = _split_dotted_path(path)
    module = importlib.import_module(module_name)
    return getattr(module, attr_name)


def import_colon_object(path: str) -> Any:
    """Import and return an object from ``package.module:object.attr``."""
    module_name, sep, attr_path = path.partition(":")
    if not sep or not module_name or not attr_path:
        raise ValueError(f"import path must be 'module:object', got {path!r}")
    obj: Any = importlib.import_module(module_name)
    for attr in attr_path.split("."):
        obj = getattr(obj, attr)
    return obj


def normalize_sys_path(extra: Any) -> tuple[str, ...]:
    if not extra:
        return ()
    if isinstance(extra, str):
        extra = [extra]
    normalized: list[str] = []
    seen: set[str] = set()
    for path in extra:
        item = str(path).strip()
        if not item or item in seen:
            continue
        normalized.append(item)
        seen.add(item)
    return tuple(normalized)


def apply_sys_path(extra: tuple[str, ...]) -> None:
    for path in extra:
        if path and path not in sys.path:
            sys.path.insert(0, path)


@lru_cache(maxsize=128)
def get_message_class(type_str: str) -> Any:
    """Import and return a message class from its fully-qualified path."""
    return import_object(type_str)


def _split_dotted_path(path: str) -> tuple[str, str]:
    module_name, sep, attr_name = path.rpartition(".")
    if not sep or not module_name or not attr_name:
        raise ValueError(f"Invalid import path: {path}")
    return module_name, attr_name
