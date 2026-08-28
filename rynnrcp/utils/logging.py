"""Unified logging configuration for RynnRCP runtime processes."""

from __future__ import annotations

import atexit
import logging
import os
import queue
import sys
import threading
from logging.handlers import QueueHandler, QueueListener, RotatingFileHandler
from typing import Any, List, Mapping, Optional

from rynnrcp.config.loader import load_config
from rynnrcp.utils.user_paths import log_file_from_config, new_log_session_id


LOG_TAGS = (
    ("rynnrcp.action_bridge", "BRIDGE"),
    ("rynnrcp.interface", "INTERFACE"),
    ("rynnrcp.runtime", "RUNTIME"),
    ("rynnrcp.services", "SERVICE"),
    ("rynnrcp.connectors", "CONNECTOR"),
    ("rynnrcp.config", "CONFIG"),
    ("rynnrcp.ipc", "IPC"),
    ("rynnrcp.process", "PROCESS"),
    ("rynnrcp.native", "NATIVE"),
    ("rynnkit.cameras", "CAMERA"),
    ("rynnrcp", "SERVER"),
)

LOG_RUN_ID_ENV = "RYNNRCP_RUN_ID"
LOG_LEVEL_ENV = "RYNNRCP_LOG_LEVEL"
LOG_MAX_BYTES_ENV = "RYNNRCP_LOG_MAX_BYTES"
LOG_MAX_FILES_ENV = "RYNNRCP_LOG_MAX_FILES"
LOG_TO_FILE_ENV = "RYNNRCP_LOG_TO_FILE"

_ASYNC_FILE_LOGGERS: list["_AsyncFileQueueHandler"] = []
_ASYNC_FILE_LOGGERS_LOCK = threading.Lock()

# Process-wide run correlation fields (robot_id, session_id, process, ...).
# Injected into every formatted record so logs from the server, runner
# processes, and apps can be joined on the same run/collection.
_LOG_CONTEXT: dict[str, str] = {}
_LOG_CONTEXT_LOCK = threading.Lock()


def resolve_log_run_id(default: str | None = None) -> str:
    """Return the correlation id shared by cooperating RynnRCP processes.

    Set ``RYNNRCP_RUN_ID`` when independently launched processes (for example
    a server, an app, and an action bridge) should be searchable as one run.
    """
    configured = os.environ.get(LOG_RUN_ID_ENV, "").strip()
    if configured:
        return configured
    resolved = str(default or new_log_session_id())
    # Persist the generated id so later configuration calls and spawned child
    # processes inherit the same correlation id.
    os.environ.setdefault(LOG_RUN_ID_ENV, resolved)
    return os.environ[LOG_RUN_ID_ENV]


def log_file_enabled(default: bool = True) -> bool:
    """Return whether rotating file output is enabled for this process."""
    return _env_flag(LOG_TO_FILE_ENV, default)


def _normalize_context_value(value: Any) -> str:
    """Keep one context field on one physical log line."""
    return str(value).replace("\r", "\\r").replace("\n", "\\n").strip()


def set_log_context(**fields: Any) -> None:
    """Attach run-correlation fields to all subsequent log lines.

    Passing ``None`` removes a field. Typical fields: ``robot_id``,
    ``run_id``, ``session_id``, ``process``, ``app_id``.
    """
    with _LOG_CONTEXT_LOCK:
        for key, value in fields.items():
            if value is None:
                _LOG_CONTEXT.pop(str(key), None)
            else:
                _LOG_CONTEXT[str(key)] = _normalize_context_value(value)


def get_log_context() -> dict[str, str]:
    with _LOG_CONTEXT_LOCK:
        return dict(_LOG_CONTEXT)


def clear_log_context() -> None:
    with _LOG_CONTEXT_LOCK:
        _LOG_CONTEXT.clear()


def log_tag(logger_name: str) -> str:
    if logger_name.startswith(("rynnrcp.app", "rynnrcp_app_")):
        return "APP"
    if logger_name.startswith("rynnrcp_robot_"):
        return "ROBOT"
    for prefix, tag in LOG_TAGS:
        if logger_name == prefix or logger_name.startswith(f"{prefix}."):
            return tag
    return "GENERAL"


class LogContextFilter(logging.Filter):
    """Snapshot process context before a record enters an async queue."""

    def filter(self, record: logging.LogRecord) -> bool:
        record.rynnrcp_context = get_log_context()
        return True


class RoboFormatter(logging.Formatter):
    """Formatter with timestamp, process id, thread id, logger, file, and line.

    Exception tracebacks (``logger.exception()`` / ``exc_info=True``) and
    ``stack_info`` are appended after the message line, matching the standard
    :class:`logging.Formatter` semantics.
    """

    def format(self, record: logging.LogRecord) -> str:
        timestamp = self.formatTime(record, "%Y-%m-%d %H:%M:%S")
        timestamp = f"{timestamp}.{int(record.msecs):03d}"
        tid = int(getattr(record, "thread", 0) or 0)
        tag = str(getattr(record, "tag", "") or log_tag(record.name))
        record_context = getattr(record, "rynnrcp_context", None)
        context = (
            dict(record_context)
            if isinstance(record_context, Mapping)
            else get_log_context()
        )
        context_block = (
            " ["
            + " ".join(f"{key}={value}" for key, value in sorted(context.items()))
            + "]"
            if context
            else ""
        )
        message = (
            f"[{timestamp}] [PID:{record.process}] [TID:{tid}] "
            f"[TAG:{tag}]{context_block} [{record.levelname:<5}] [{record.name}] "
            f"{record.filename}:{record.lineno} {record.getMessage()}"
        )
        if record.exc_info and not record.exc_text:
            record.exc_text = self.formatException(record.exc_info)
        if record.exc_text:
            message = f"{message}\n{record.exc_text}"
        if record.stack_info:
            message = f"{message}\n{self.formatStack(record.stack_info)}"
        return message


class ColorStreamHandler(logging.StreamHandler):
    """Stream handler with optional ANSI colors for TTY output."""

    COLORS = {
        logging.DEBUG: "\033[90m",
        logging.INFO: "\033[32m",
        logging.WARNING: "\033[33m",
        logging.ERROR: "\033[31m",
        logging.CRITICAL: "\033[35m",
    }
    RESET = "\033[0m"

    def __init__(self, stream=None, colored: bool = True):
        super().__init__(stream or sys.stderr)
        self.colored = colored
        self.setFormatter(RoboFormatter())

    def emit(self, record: logging.LogRecord) -> None:
        if self.colored and hasattr(self.stream, "isatty") and self.stream.isatty():
            color = self.COLORS.get(record.levelno, "")
            try:
                msg = self.format(record)
                self.stream.write(f"{color}{msg}{self.RESET}\n")
                self.flush()
            except Exception:
                self.handleError(record)
            return
        super().emit(record)


class ShmLogHandler(logging.Handler):
    """Write formatted log records to a shared-memory ring buffer."""

    def __init__(
        self,
        channel_name: str = "rynnrcp_log",
        slot_size: int = 2048,
        slot_count: int = 256,
    ) -> None:
        super().__init__()
        self.channel_name = channel_name
        self.slot_size = int(slot_size)
        self.slot_count = int(slot_count)
        self._ring_buffer = None
        self._init_lock = threading.Lock()
        self.setFormatter(RoboFormatter())

    def _ensure_buffer(self) -> bool:
        if self._ring_buffer is not None:
            return True
        with self._init_lock:
            if self._ring_buffer is not None:
                return True
            try:
                from rynnrcp.ipc.ring_buffer import RingBuffer

                try:
                    self._ring_buffer = RingBuffer(
                        self.channel_name,
                        slot_size=self.slot_size,
                        slot_count=self.slot_count,
                        create=False,
                    )
                except RuntimeError:
                    self._ring_buffer = RingBuffer(
                        self.channel_name,
                        slot_size=self.slot_size,
                        slot_count=self.slot_count,
                        create=True,
                    )
                return True
            except Exception:
                return False

    def emit(self, record: logging.LogRecord) -> None:
        if not self._ensure_buffer():
            return
        try:
            msg = self.format(record).encode("utf-8", errors="replace")
            self._ring_buffer.write(msg[: self.slot_size])
        except Exception:
            self.handleError(record)

    def close(self) -> None:
        try:
            if self._ring_buffer is not None:
                self._ring_buffer.close()
        finally:
            self._ring_buffer = None
            super().close()


def _stop_async_file_loggers() -> None:
    with _ASYNC_FILE_LOGGERS_LOCK:
        items = list(_ASYNC_FILE_LOGGERS)
    for handler in items:
        handler.stop()


atexit.register(_stop_async_file_loggers)


class _AsyncFileQueueHandler(QueueHandler):
    """Queue handler that owns its rotating file sink and listener thread.

    Each instance is self-contained: closing it stops only its own listener,
    so reconfiguring one logger never tears down file logging that another
    component set up in the same process.
    """

    def __init__(
        self,
        path: str,
        *,
        level: int,
        file_max_bytes: int,
        file_max_files: int,
    ) -> None:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        file_handler = RotatingFileHandler(
            path,
            maxBytes=file_max_bytes,
            backupCount=file_max_files,
            encoding="utf-8",
        )
        file_handler.setLevel(level)
        file_handler.setFormatter(RoboFormatter())

        log_queue: queue.SimpleQueue[logging.LogRecord] = queue.SimpleQueue()
        super().__init__(log_queue)
        self.setLevel(level)
        self._file_handler = file_handler
        self._listener = QueueListener(
            log_queue, file_handler, respect_handler_level=True
        )
        self._listener.start()
        self._stopped = False
        with _ASYNC_FILE_LOGGERS_LOCK:
            _ASYNC_FILE_LOGGERS.append(self)

    def stop(self) -> None:
        """Drain queued records, stop the listener, and close the file."""
        if self._stopped:
            return
        self._stopped = True
        with _ASYNC_FILE_LOGGERS_LOCK:
            if self in _ASYNC_FILE_LOGGERS:
                _ASYNC_FILE_LOGGERS.remove(self)
        try:
            self._listener.stop()
        finally:
            self._file_handler.close()

    def close(self) -> None:
        self.stop()
        super().close()


def _async_file_handler(
    path: str,
    *,
    level: int,
    file_max_bytes: int,
    file_max_files: int,
) -> QueueHandler:
    return _AsyncFileQueueHandler(
        path,
        level=level,
        file_max_bytes=file_max_bytes,
        file_max_files=file_max_files,
    )


def configure_logging(
    level: int = logging.INFO,
    sinks: Optional[List[str]] = None,
    *,
    colored: bool = True,
    file_path: Optional[str] = None,
    file_max_bytes: int = 50 * 1024 * 1024,
    file_max_files: int = 5,
    shm_channel: str = "rynnrcp_log",
    shm_level: Optional[int] = None,
    logger_name: Optional[str] = None,
) -> logging.Logger:
    """Configure logging for a RynnRCP process.

    By default this configures the Python root logger so existing module
    loggers are covered without forcing every package to use one namespace.
    """
    if sinks is None:
        sinks = ["stderr"]
    else:
        sinks = list(sinks)
    level = _env_log_level(level)
    if not log_file_enabled():
        sinks = [sink for sink in sinks if sink != "file"]
    file_max_bytes = _env_positive_int(LOG_MAX_BYTES_ENV, file_max_bytes)
    file_max_files = _env_positive_int(LOG_MAX_FILES_ENV, file_max_files)
    if "file" in sinks:
        if file_max_bytes <= 0:
            raise ValueError("file_max_bytes must be greater than zero")
        if file_max_files <= 0:
            raise ValueError("file_max_files must be greater than zero")

    logger = logging.getLogger(logger_name) if logger_name else logging.getLogger()
    logger.setLevel(level)
    # Close only this logger's own handlers. A global teardown here would
    # silently kill file listeners configured by other components in the
    # same process (e.g. an app embedding a server).
    for handler in list(logger.handlers):
        logger.removeHandler(handler)
        try:
            handler.close()
        except Exception:
            # Handler teardown must never break logging reconfiguration.
            pass

    file_sink_errors: list[tuple[str, OSError]] = []
    for sink_type in sinks:
        if sink_type == "stderr":
            handler = ColorStreamHandler(colored=colored)
        elif sink_type == "file":
            path = file_path or f"logs/rynnrcp_{os.getpid()}.log"
            try:
                handler = _async_file_handler(
                    path,
                    level=level,
                    file_max_bytes=file_max_bytes,
                    file_max_files=file_max_files,
                )
            except OSError as exc:
                # Logging must not prevent a robot process from starting when
                # its configured log directory is temporarily unavailable.
                file_sink_errors.append((path, exc))
                continue
        elif sink_type == "shm":
            handler = ShmLogHandler(channel_name=shm_channel)
            handler.setLevel(shm_level or logging.WARNING)
            handler.addFilter(LogContextFilter())
            logger.addHandler(handler)
            continue
        else:
            raise ValueError(f"Unknown log sink: {sink_type}")

        handler.setLevel(level)
        handler.addFilter(LogContextFilter())
        logger.addHandler(handler)

    if file_sink_errors:
        if not any(
            isinstance(handler, ColorStreamHandler) for handler in logger.handlers
        ):
            fallback = ColorStreamHandler(colored=colored)
            fallback.setLevel(level)
            fallback.addFilter(LogContextFilter())
            logger.addHandler(fallback)
        for path, exc in file_sink_errors:
            logger.log(
                max(level, logging.WARNING),
                "[Logging][FILE_SINK_UNAVAILABLE] path=%s error=%s; "
                "continuing with stderr logging",
                path,
                exc,
            )

    logger.propagate = False if logger_name else True
    return logger


def _env_log_level(default: int) -> int:
    value = os.environ.get(LOG_LEVEL_ENV, "").strip().upper()
    if not value:
        return default
    resolved = logging.getLevelName(value)
    return resolved if isinstance(resolved, int) else default


def _env_positive_int(name: str, default: int) -> int:
    try:
        value = int(os.environ.get(name, ""))
    except (TypeError, ValueError):
        return default
    return value if value > 0 else default


def _env_flag(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    return default


def configure_server_logging(
    config: str | Mapping[str, Any],
    *,
    level: int = logging.INFO,
    session_id: str | None = None,
    process_name: str | None = None,
) -> logging.Logger:
    """Configure stderr + rotating-file logging for one robot server run.

    Log files land under ``$RYNNRCP_HOME/<robot_id>/logs/<session_id>/`` and
    every line carries ``robot_id``/``run_id``/``session_id`` so multi-process
    runs can be correlated. Level and rotation are overridable via the environment:
    ``RYNNRCP_LOG_LEVEL``, ``RYNNRCP_LOG_MAX_BYTES``, ``RYNNRCP_LOG_MAX_FILES``,
    and ``RYNNRCP_LOG_TO_FILE=0`` disables the file sink.
    """
    loaded = load_config(config) if isinstance(config, str) else dict(config)
    log_session_id = str(session_id or new_log_session_id())

    manifest = (
        loaded.get("manifest") if isinstance(loaded.get("manifest"), Mapping) else {}
    )
    set_log_context(
        robot_id=str(manifest.get("robot_id") or "") or None,
        run_id=resolve_log_run_id(log_session_id),
        session_id=log_session_id,
        process=process_name,
    )

    sinks = ["stderr"]
    if log_file_enabled():
        sinks.append("file")
    return configure_logging(
        level=_env_log_level(level),
        sinks=sinks,
        file_path=str(
            log_file_from_config(loaded, "server.log", session_id=log_session_id)
        ),
        file_max_bytes=_env_positive_int(LOG_MAX_BYTES_ENV, 50 * 1024 * 1024),
        file_max_files=_env_positive_int(LOG_MAX_FILES_ENV, 5),
    )
