# Copyright 2026 RynnRCP Authors. All rights reserved.
# Native acceleration auto-detection.
# This module provides a transparent fallback mechanism:
#   - If rynnrcp_core_native (C++ extension) is available, use it
#   - Otherwise, fall back to pure Python implementations

from __future__ import annotations

import logging

_logger = logging.getLogger(__name__)

_native_available = False
_native_module = None

try:
    import rynnrcp_core_native as _native_module  # type: ignore
    _native_available = True
    _logger.info("Native C++ acceleration available (rynnrcp_core_native)")
    # Bridge C++ logs to Python logging system
    _native_module.set_python_log_sink(logging.getLogger("rynnrcp.native"))
except ImportError:
    _native_available = False
    _logger.debug("Native C++ acceleration not available, using pure Python")


def is_native_available() -> bool:
    """Check if the native C++ acceleration module is available."""
    return _native_available


def get_native_module():
    """Get the native module, or None if not available."""
    return _native_module
