# rcp_core/common/adapter/module_generic_input_adapter.py

"""
Generic input adapter for Python module return values.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.adapter.module_generic_input_adapter.ModuleGenericInputAdapter`,
a :class:`~rcp_core.common.adapter.base_input_adapter.BaseInputAdapter` implementation that
turns the return value of a Python callable into a buffered observation entry.

The adapter:
- uses the current wall-clock time as the timestamp
- expects the message to be a 1-D numeric sequence (``numpy.ndarray``, ``list``, or ``tuple``)
- normalizes the data into ``array('d')``
- stores it under the configured ``params.out_key`` (e.g. ``observation.state``)
"""

import time
import numpy as np

from typing import Any, Dict, Tuple
from array import array

from .base_input_adapter import BaseInputAdapter


class ModuleGenericInputAdapter(BaseInputAdapter):
    """
    Generic module input adapter for constructing observation.state, etc.,
    from return values of Python methods.

    Conventions:
      - Method return value is a 1D array of joint data: np.ndarray / list / tuple
      - adapter_cfg['params']['out_key'] specifies the key to write to the buffer
    """

    def parse(self, msg: Any) -> Tuple[float, Dict[str, Any]]:
        """Return (ts, out_dict) where fields are extracted per mappings."""
        params = self.adapter_cfg.get("params", {})
        out_key = params.get("out_key")
        if not out_key:
            raise ValueError(
                "[ModuleGenericInputAdapter] params.out_key is not configured, "
                "please provide params.out_key in the YAML"
            )

        ts = time.time()

        # Convert to array('d')
        if isinstance(msg, np.ndarray):
            data = array("d", [float(x) for x in msg.reshape(-1)])
        elif isinstance(msg, (list, tuple)):
            data = array("d", [float(x) for x in msg])
        else:
            raise TypeError(
                f"[ModuleGenericInputAdapter] Unsupported msg type: {type(msg)}, "
                "expected np.ndarray/list/tuple"
            )

        return ts, {out_key: data}
