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
from rynnrcp.utils.user_paths import ensure_robot_dirs, log_file_from_config, new_log_session_id, robot_root_from_config


LOG_TAGS = (
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

_ASYNC_FILE_LOGGERS: list[tuple[QueueListener, RotatingFileHandler]] = []
_ASYNC_FILE_LOGGERS_LOCK = threading.Lock()


def log_tag(logger_name: str) -> str:
    if logger_name.startswith("rynnrcp.app"):
        return "APP"
    for prefix, tag in LOG_TAGS:
        if logger_name == prefix or logger_name.startswith(f"{prefix}."):
            return tag
    return "GENERAL"


class RoboFormatter(logging.Formatter):
    """Formatter with timestamp, process id, thread id, logger, file, and line."""

    def format(self, record: logging.LogRecord) -> str:
        timestamp = self.formatTime(record, "%Y-%m-%d %H:%M:%S")
        timestamp = f"{timestamp}.{int(record.msecs):03d}"
        tid = threading.current_thread().ident or 0
        tag = str(getattr(record, "tag", "") or log_tag(record.name))
        return (
            f"[{timestamp}] [PID:{os.getpid()}] [TID:{tid}] "
            f"[TAG:{tag}] [{record.levelname:<5}] [{record.name}] "
            f"{record.filename}:{record.lineno} {record.getMessage()}"
        )


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
        _ASYNC_FILE_LOGGERS.clear()
    for listener, handler in items:
        try:
            listener.stop()
        finally:
            handler.close()


atexit.register(_stop_async_file_loggers)


def _async_file_handler(
    path: str,
    *,
    level: int,
    file_max_bytes: int,
    file_max_files: int,
) -> QueueHandler:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    file_handler = RotatingFileHandler(
        path,
        maxBytes=file_max_bytes,
        backupCount=file_max_files,
    )
    file_handler.setLevel(level)
    file_handler.setFormatter(RoboFormatter())

    log_queue: queue.SimpleQueue[logging.LogRecord] = queue.SimpleQueue()
    queue_handler = QueueHandler(log_queue)
    queue_handler.setLevel(level)
    listener = QueueListener(log_queue, file_handler, respect_handler_level=True)
    listener.start()
    with _ASYNC_FILE_LOGGERS_LOCK:
        _ASYNC_FILE_LOGGERS.append((listener, file_handler))
    return queue_handler


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

    logger = logging.getLogger(logger_name) if logger_name else logging.getLogger()
    logger.setLevel(level)
    _stop_async_file_loggers()
    logger.handlers.clear()

    for sink_type in sinks:
        if sink_type == "stderr":
            handler = ColorStreamHandler(colored=colored)
        elif sink_type == "file":
            path = file_path or f"logs/rynnrcp_{os.getpid()}.log"
            handler = _async_file_handler(
                path,
                level=level,
                file_max_bytes=file_max_bytes,
                file_max_files=file_max_files,
            )
        elif sink_type == "shm":
            handler = ShmLogHandler(channel_name=shm_channel)
            handler.setLevel(shm_level or logging.WARNING)
            logger.addHandler(handler)
            continue
        else:
            raise ValueError(f"Unknown log sink: {sink_type}")

        handler.setLevel(level)
        logger.addHandler(handler)

    logger.propagate = False if logger_name else True
    return logger


def configure_server_logging(
    config: str | Mapping[str, Any],
    *,
    level: int = logging.INFO,
    session_id: str | None = None,
) -> logging.Logger:
    loaded = load_config(config) if isinstance(config, str) else dict(config)
    ensure_robot_dirs(robot_root_from_config(loaded))
    log_session_id = str(session_id or new_log_session_id())
    return configure_logging(
        level=level,
        sinks=["stderr", "file"],
        file_path=str(log_file_from_config(loaded, "server.log", session_id=log_session_id)),
    )
