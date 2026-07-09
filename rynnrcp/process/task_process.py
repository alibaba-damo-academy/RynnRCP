"""Short-lived child process helpers.

``ProcessNode`` and ``NodeLauncher`` are for long-running RynnRCP nodes. This
module covers one-shot jobs such as dataset encoding, where the caller needs a
structured result instead of a persistent ChannelManager process.
"""

from __future__ import annotations

import multiprocessing
import traceback
from multiprocessing.connection import Connection
from typing import Any, Dict, Optional, Sequence

from rynnrcp.utils.imports import import_colon_object


class ProcessTaskError(RuntimeError):
    """Raised when a short-lived child process fails."""


def run_python_function_task(
    function_path: str,
    args: Optional[Sequence[Any]] = None,
    kwargs: Optional[Dict[str, Any]] = None,
    timeout_s: Optional[float] = None,
) -> Any:
    """Run an importable Python callable in a child process.

    Args:
        function_path: Import path in ``"package.module:function"`` form.
        args: Positional arguments. Values must be pickleable.
        kwargs: Keyword arguments. Values must be pickleable.
        timeout_s: Optional wall-clock timeout.

    Returns:
        The callable's return value, transferred through a multiprocessing
        queue. The value must be pickleable.
    """

    ctx = multiprocessing.get_context("spawn")
    parent_conn, child_conn = ctx.Pipe(duplex=False)
    process = ctx.Process(
        target=_run_function_child,
        args=(function_path, list(args or []), dict(kwargs or {}), child_conn),
        name=f"rynnrcp_task:{function_path}",
        daemon=False,
    )
    process.start()
    child_conn.close()
    process.join(timeout=timeout_s)
    if process.is_alive():
        process.terminate()
        process.join(timeout=2.0)
        if process.is_alive():
            process.kill()
            process.join(timeout=1.0)
        raise ProcessTaskError(f"task timed out: {function_path}")

    message = parent_conn.recv() if parent_conn.poll(0.1) else None
    parent_conn.close()
    if not message:
        raise ProcessTaskError(f"task exited without result: {function_path}, exitcode={process.exitcode}")
    if message.get("success"):
        return message.get("result")
    detail = message.get("traceback") or message.get("error")
    raise ProcessTaskError(detail or f"task failed: {function_path}")


def _run_function_child(
    function_path: str,
    args: Sequence[Any],
    kwargs: Dict[str, Any],
    conn: Connection,
) -> None:
    try:
        fn = import_colon_object(function_path)
        result = fn(*args, **kwargs)
        conn.send({"success": True, "result": result})
    except Exception as exc:
        conn.send({
            "success": False,
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        })
    finally:
        conn.close()
