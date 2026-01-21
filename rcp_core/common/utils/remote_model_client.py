# rcp_core/common/utils/remote_model_client.py

"""
Remote model HTTP client with NumPy-aware msgpack encoding.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines utilities for sending observations to a remote inference service
over HTTP using a compact serialization format:

    Python objects (incl. NumPy) → msgpack → base64 → JSON string → HTTP POST

It provides:
- :func:`pack_array` / :func:`unpack_array`: custom msgpack hooks that serialize
  ``np.ndarray`` and NumPy scalar types in a safe, reversible way.
- :class:`RemoteModelClient`: a small client that performs one-shot inference calls.

Serialization format
--------------------
- ``np.ndarray`` is encoded as a dict containing:
  ``__ndarray__``, raw byte buffer (``tobytes()``), dtype string, and shape.
- NumPy scalars (``np.generic``) are encoded similarly with ``__npgeneric__`` and ``item()``.
- Object/void/complex-like dtypes are rejected (dtype.kind in ``"V", "O", "c"``).

Network protocol
----------------
:meth:`RemoteModelClient.infer`:
- msgpacks the observation dict (NumPy-aware), base64-encodes it, wraps it as JSON,
  and POSTs to ``http://{host}:{port}``.
- Optionally attaches an ``Authorization`` header if ``token`` is provided.
- Expects a 200 response whose JSON body is a base64 string representing msgpack data.
- Decodes base64, unpacks msgpack (restoring NumPy arrays/scalars), and returns the result dict.
"""

import base64
import functools
import json
from typing import Any, Dict

import msgpack
import numpy as np
import requests


def pack_array(obj):
    """Pack numpy arrays/scalars into a msgpack-serializable dict."""
    if (isinstance(obj, (np.ndarray, np.generic))) and obj.dtype.kind in (
        "V",
        "O",
        "c",
    ):
        raise ValueError(f"Unsupported dtype: {obj.dtype}")
    if isinstance(obj, np.ndarray):
        return {
            b"__ndarray__": True,
            b"data": obj.tobytes(),
            b"dtype": obj.dtype.str,
            b"shape": obj.shape,
        }
    if isinstance(obj, np.generic):
        return {
            b"__npgeneric__": True,
            b"data": obj.item(),
            b"dtype": obj.dtype.str,
        }
    return obj


def unpack_array(obj):
    """Restore packed numpy dicts back to arrays or scalars."""
    if b"__ndarray__" in obj:
        return np.ndarray(
            buffer=obj[b"data"],
            dtype=np.dtype(obj[b"dtype"]),
            shape=obj[b"shape"],
        )
    if b"__npgeneric__" in obj:
        return np.dtype(obj[b"dtype"]).type(obj[b"data"])
    return obj


packb = functools.partial(msgpack.packb, default=pack_array)
unpackb = functools.partial(msgpack.unpackb, object_hook=unpack_array)


class RemoteModelClient:
    """Simple HTTP client for models using base64 + msgpack(+numpy) protocol."""

    def __init__(self, host: str, port: int, token: str = ""):
        """Configure target server address and optional auth token."""
        self.base_url = f"http://{host}:{port}"
        self.token = token.strip()
        self.session = requests.Session()

    def infer(self, obs: Dict[str, Any], timeout: float = 10.0) -> Dict[str, Any]:
        """Send a single inference request and return the decoded response."""
        packed = packb(obs)
        packed_b64 = base64.b64encode(packed).decode("utf-8")
        data = json.dumps(packed_b64)

        headers = {"Content-Type": "application/json"}
        if self.token:
            headers["Authorization"] = self.token

        resp = self.session.post(
            self.base_url, headers=headers, data=data, timeout=timeout
        )
        if resp.status_code != 200:
            raise RuntimeError(
                f"RemoteModelClient infer failed: {resp.status_code}, {resp.text}"
            )

        resp_b64 = resp.json()
        resp_raw = base64.b64decode(resp_b64 + "=" * (4 - len(resp_b64) % 4))
        resp_obj = unpackb(resp_raw)
        return resp_obj
