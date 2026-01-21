# rcp_core/common/adapter/base_input_adapter.py

"""
Base interface for input adapters.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.adapter.base_input_adapter.BaseInputAdapter`,
the minimal contract used to normalize incoming middleware/protocol messages.

An input adapter converts a raw message into:
- a timestamp (float, seconds)
- a standardized ``Dict[str, Any]`` payload for downstream buffering and processing

Concrete adapters should subclass :class:`BaseInputAdapter` and implement
:meth:`BaseInputAdapter.parse`.
"""

from typing import Any, Dict, Tuple


class BaseInputAdapter:
    """Base class for all Input Adapters. Must implement parse(msg) -> (ts, Dict[str, Any])."""

    def __init__(self, adapter_cfg: Dict):
        self.adapter_cfg = adapter_cfg

    def parse(self, msg: Any) -> Tuple[float, Dict[str, Any]]:
        """Parse an input message into a timestamp and a standardized data dict."""
        raise NotImplementedError
