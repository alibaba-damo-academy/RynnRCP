# rcp_core/common/utils/logger.py

"""
Process-wide logging utility (GLog-style).
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module implements an asynchronous, per-logger file logging system with optional
stderr mirroring, configured via a YAML file (``common/config/glog_config.yaml``).

Configuration
-------------
:class:`LogConfig` loads settings from YAML (if PyYAML is installed) and provides defaults:
- ``log_dir``: base output directory (default ``~/RynnRcplog``)
- per-logger subdirectories (e.g. ``server_node`` and ``robot_motion``)
- rotation settings (max size in MB, backup count)
- buffering/flush cadence (``log_buf_secs``)
- minimum log level and stderr threshold (GLog-like 0/1/2/3 mapping)

Process identity (robot_name + APPID + timestamp)
-------------------------------------------------
The module maintains process-level identity used to construct log filenames:
- ``robot_name`` is derived from the robot config YAML (prefers
  ``device_monitor_server.robot_info.robot_name``; falls back to top-level ``robot_type``).
- ``APPID`` is chosen uniquely across concurrently-started processes using atomic lock files
  under ``{log_dir}/.appid_lock``.
- a timestamp ``YYYYmmdd_HHMMSS`` is captured at initialization.

Call :func:`init_process_logging` once early in process startup to set these values; if you
don’t, defaults are used.

Async logging architecture
--------------------------
- Each logger created by :meth:`GLog.get_logger` is configured with a single
  :class:`logging.handlers.QueueHandler` that pushes records into a shared queue.
- A single :class:`logging.handlers.QueueListener` consumes records in a background thread and
  forwards them to one custom :class:`_DispatchHandler`.
- The dispatch handler routes each record to:
  - a per-logger-name rotating file handler (created on-demand)
  - a shared stderr handler (created on-demand)

File naming / rotation
----------------------
If ``timestamped_filename`` is true, logs are written as:

  ``{log_dir}/{logger_name}/{robot_name}_{APPID}_{timestamp}.log``

with size-based rotation via :class:`_TimestampedRotatingFileHandler`. The chosen active log
file path (and rollover events) are printed to stderr.

Public helpers
--------------
- :func:`get_logger(name)`: get a named logger managed by GLog
- :func:`server_logger()`: logger for ``server_node_log_name``
- :func:`motion_logger()`: logger for ``robot_motion_log_name``

Shutdown
--------
:class:`GLog` registers an ``atexit`` hook to stop the listener and close all handlers.
"""

from __future__ import annotations

import atexit
import logging
import logging.handlers
import os
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

try:
    import yaml  # pip install pyyaml
except Exception:
    yaml = None


def _default_config_path() -> str:
    """
    Resolve config path relative to this file (not CWD).

    RynnRCP/
      common/config/glog_config.yaml
      rcp_core/common/utils/logger.py
    """
    here = Path(__file__).resolve()
    project_root = here.parents[
        4
    ]  # logger.py -> utils -> common -> rcp_core -> RynnRCP
    return str(project_root / "common" / "config" / "glog_config.yaml")


DEFAULT_CONFIG_PATH = _default_config_path()

# 0=INFO, 1=WARN, 2=ERROR, 3=FATAL
_LEVEL_NO = {0: logging.INFO, 1: logging.WARNING, 2: logging.ERROR, 3: logging.CRITICAL}


@dataclass(frozen=True)
class LogConfig:
    log_dir: str = "~/RynnRcplog"
    server_node_log_name: str = "server_node"
    robot_motion_log_name: str = "robot_motion"
    stderr_threshold: int = 0
    max_log_size: int = 10  # MB
    log_buf_secs: float = 1.0  # seconds
    min_log_level: int = 0  # 0=INFO
    backup_count: int = 10
    timestamped_filename: bool = True
    fmt: str = (
        "%(asctime)s %(levelname)s [%(name)s] %(filename)s:%(lineno)d | %(message)s"
    )
    datefmt: str = "%Y-%m-%d %H:%M:%S"

    @classmethod
    def load(cls, path: str = DEFAULT_CONFIG_PATH) -> "LogConfig":
        if yaml is None:
            return cls()
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            if not isinstance(data, dict):
                return cls()
        except Exception:
            return cls()

        fields = cls().__dict__.keys()
        kwargs = {k: data[k] for k in data.keys() if k in fields}
        return cls(**kwargs)


class _MinLevelFilter(logging.Filter):
    def __init__(self, min_levelno: int):
        super().__init__()
        self.min_levelno = int(min_levelno)

    def filter(self, record: logging.LogRecord) -> bool:
        return record.levelno >= self.min_levelno


# ---------------- process-level identity ----------------

_PROCESS_ROBOT_NAME: str = "robot"
_PROCESS_APPID: int = 0
_PROCESS_TS: Optional[str] = None
_PROCESS_INIT_DONE = False
_PROCESS_INIT_LOCK = threading.Lock()


def _read_robot_name(robot_config_path: str) -> str:
    """Read robot_name from the robot config YAML.

    Priority:
      1) servers[].name == "device_monitor_server" -> robot_info.robot_name
      2) top-level robot_type
    """
    if yaml is None:
        return "robot"
    try:
        with open(robot_config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
    except Exception:
        return "robot"

    robot_name = str(cfg.get("robot_type", "robot"))
    for s in cfg.get("servers") or []:
        if s.get("name") == "device_monitor_server":
            robot_info = s.get("robot_info") or {}
            robot_name = str(robot_info.get("robot_name", robot_name))
            break
    return robot_name


def _atomic_create(path: str) -> bool:
    """Atomically create a file. Returns False if it already exists."""
    flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY
    try:
        fd = os.open(path, flags, 0o644)
        os.close(fd)
        return True
    except FileExistsError:
        return False


def init_process_logging(
    *, robot_config_path: Optional[str] = None, appid_default: int = 0
) -> None:
    """
    Initialize process-level logging identity (call once per process, before get_logger()).

    - robot_name is read from robot_config_path (see _read_robot_name).
    - APPID starts from appid_default and increments until a unique one is acquired.
      This is concurrency-safe across multiple processes launched at the same time.
    """
    global _PROCESS_ROBOT_NAME, _PROCESS_APPID, _PROCESS_TS, _PROCESS_INIT_DONE

    with _PROCESS_INIT_LOCK:
        if _PROCESS_INIT_DONE:
            return

        cfg = LogConfig.load(DEFAULT_CONFIG_PATH)
        base_dir = os.path.expanduser(cfg.log_dir)
        os.makedirs(base_dir, exist_ok=True)

        robot_name = (
            _read_robot_name(robot_config_path) if robot_config_path else "robot"
        )
        ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())

        # Use lock files to allocate unique APPIDs across concurrent processes.
        lock_dir = os.path.join(base_dir, ".appid_lock")
        os.makedirs(lock_dir, exist_ok=True)

        appid = int(appid_default)
        while True:
            lock_file = os.path.join(lock_dir, f"{robot_name}_{ts}_{appid}.lock")
            if _atomic_create(lock_file):
                break
            appid += 1

        _PROCESS_ROBOT_NAME = robot_name
        _PROCESS_APPID = appid
        _PROCESS_TS = ts
        _PROCESS_INIT_DONE = True


# ---------------- file handler ----------------


class _TimestampedRotatingFileHandler(logging.handlers.RotatingFileHandler):
    """{base_dir}/{logger_name}/{robot_name}_{APPID}_YYYYmmdd_HHMMSS.log(.1/.2/...)"""

    def __init__(self, base_dir: str, name: str, max_bytes: int, backup_count: int):
        d = os.path.join(base_dir, name)
        os.makedirs(d, exist_ok=True)

        robot_name = _PROCESS_ROBOT_NAME
        appid = _PROCESS_APPID
        ts = _PROCESS_TS or time.strftime("%Y%m%d_%H%M%S", time.localtime())

        base = os.path.join(d, f"{robot_name}_{appid}_{ts}.log")

        # Fallback to avoid overwriting if a handler is unexpectedly re-created.
        filename = base
        if os.path.exists(filename):
            i = 2
            while True:
                filename = base.replace(".log", f"_{i}.log")
                if not os.path.exists(filename):
                    break
                i += 1

        super().__init__(
            filename, maxBytes=max_bytes, backupCount=backup_count, encoding="utf-8"
        )

        # Print the selected log file path once the handler is created.
        # Use stderr to avoid going through logging again (which could recurse).
        print(f"[GLog] log file: {self.baseFilename}", file=os.sys.stderr)

    def doRollover(self) -> None:
        """Rotate log files when maxBytes is reached, then print the new active log file."""
        super().doRollover()
        # After rollover, RotatingFileHandler continues writing to baseFilename
        print(
            f"[GLog] log rollover, new active file: {self.baseFilename}",
            file=os.sys.stderr,
        )


class _DispatchHandler(logging.Handler):
    """
    A single handler attached to the QueueListener.
    It dispatches records to:
      - per-logger-name file handler
      - shared stderr handler
    """

    def __init__(self, mgr: "GLog"):
        super().__init__(level=logging.NOTSET)
        self._mgr = mgr

    def emit(self, record: logging.LogRecord) -> None:
        self._mgr._emit_to_sinks(record)


class GLog:
    _inst_lock = threading.Lock()
    _inst: Optional["GLog"] = None

    def __init__(self, cfg: LogConfig):
        self.cfg = cfg
        self.base_dir = os.path.expanduser(cfg.log_dir)
        os.makedirs(self.base_dir, exist_ok=True)

        self._q: "queue.Queue[logging.LogRecord]" = queue.Queue(maxsize=10000)

        self._sink_lock = threading.Lock()
        self._file_handlers: Dict[str, logging.Handler] = {}
        self._stderr_handler: Optional[logging.Handler] = None

        # One listener with one dispatch handler (no dynamic append to listener.handlers)
        self._dispatch = _DispatchHandler(self)
        self._listener = logging.handlers.QueueListener(
            self._q, self._dispatch, respect_handler_level=False
        )
        self._listener.start()

        if cfg.log_buf_secs and cfg.log_buf_secs > 0:
            threading.Thread(target=self._flush_loop, daemon=True).start()

        atexit.register(self.close)

    @classmethod
    def instance(cls) -> "GLog":
        with cls._inst_lock:
            if cls._inst is None:
                cls._inst = cls(LogConfig.load(DEFAULT_CONFIG_PATH))
        return cls._inst

    def get_logger(self, name: str) -> logging.Logger:
        log = logging.getLogger(name)
        log.propagate = False
        log.setLevel(_LEVEL_NO.get(self.cfg.min_log_level, logging.INFO))

        # Each logger only needs a QueueHandler (never add file handlers directly here)
        if not any(isinstance(h, logging.handlers.QueueHandler) for h in log.handlers):
            log.addHandler(logging.handlers.QueueHandler(self._q))

        # Pre-create sinks so the first record doesn't race on creation
        self._ensure_file_handler(name)
        self._ensure_stderr_handler()
        return log

    def get_server_logger(self) -> logging.Logger:
        return self.get_logger(self.cfg.server_node_log_name)

    def get_motion_logger(self) -> logging.Logger:
        return self.get_logger(self.cfg.robot_motion_log_name)

    def close(self) -> None:
        try:
            self._listener.stop()
        except Exception:
            pass

        with self._sink_lock:
            for h in self._file_handlers.values():
                try:
                    h.close()
                except Exception:
                    pass
            self._file_handlers.clear()

            if self._stderr_handler is not None:
                try:
                    self._stderr_handler.close()
                except Exception:
                    pass
                self._stderr_handler = None

    # ---------- internal sinks ----------

    def _ensure_file_handler(self, name: str) -> None:
        with self._sink_lock:
            if name in self._file_handlers:
                return
            self._file_handlers[name] = self._make_file_handler(name)

    def _ensure_stderr_handler(self) -> None:
        with self._sink_lock:
            if self._stderr_handler is not None:
                return
            self._stderr_handler = self._make_stderr_handler()

    def _emit_to_sinks(self, record: logging.LogRecord) -> None:
        # Called inside QueueListener thread
        name = record.name
        with self._sink_lock:
            fh = self._file_handlers.get(name)
            sh = self._stderr_handler

        if fh:
            try:
                fh.handle(record)
            except Exception:
                pass

        if sh:
            try:
                sh.handle(record)
            except Exception:
                pass

    def _flush_loop(self) -> None:
        while True:
            time.sleep(max(float(self.cfg.log_buf_secs), 0.1))
            with self._sink_lock:
                handlers = list(self._file_handlers.values())
                if self._stderr_handler:
                    handlers.append(self._stderr_handler)
            for h in handlers:
                try:
                    h.flush()
                except Exception:
                    pass

    def _make_file_handler(self, name: str) -> logging.Handler:
        max_bytes = int(self.cfg.max_log_size * 1024 * 1024)
        backup = int(self.cfg.backup_count)
        min_levelno = _LEVEL_NO.get(self.cfg.min_log_level, logging.INFO)

        if self.cfg.timestamped_filename:
            h: logging.Handler = _TimestampedRotatingFileHandler(
                self.base_dir, name, max_bytes, backup
            )
        else:
            d = os.path.join(self.base_dir, name)
            os.makedirs(d, exist_ok=True)
            filename = os.path.join(d, f"{name}.log")
            h = logging.handlers.RotatingFileHandler(
                filename, maxBytes=max_bytes, backupCount=backup, encoding="utf-8"
            )

        h.setLevel(min_levelno)
        h.addFilter(_MinLevelFilter(min_levelno))
        h.setFormatter(logging.Formatter(self.cfg.fmt, self.cfg.datefmt))
        return h

    def _make_stderr_handler(self) -> logging.Handler:
        levelno = _LEVEL_NO.get(self.cfg.stderr_threshold, logging.INFO)
        h = logging.StreamHandler()
        h.setLevel(levelno)
        h.addFilter(_MinLevelFilter(levelno))
        h.setFormatter(logging.Formatter(self.cfg.fmt, self.cfg.datefmt))
        return h


def get_logger(name: str) -> logging.Logger:
    return GLog.instance().get_logger(name)


def server_logger() -> logging.Logger:
    return GLog.instance().get_server_logger()


def motion_logger() -> logging.Logger:
    return GLog.instance().get_motion_logger()
