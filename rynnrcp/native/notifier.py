"""Cross-process named notification primitive."""

from __future__ import annotations

import ctypes
import errno
import hashlib
import logging
import os
import threading
import time
from abc import ABC, abstractmethod
from ctypes import wintypes

from rynnrcp.native.event import WaitResult

logger = logging.getLogger(__name__)
_DEFAULT_FALLBACK_POLL_INTERVAL_MS = 5.0
_MIN_FALLBACK_POLL_INTERVAL_MS = 0.5


class NotifierUnavailable(RuntimeError):
    """Raised when the current platform cannot create/open a named notifier."""


class IpcNotifier(ABC):
    """Named cross-process notifier."""

    @abstractmethod
    def notify(self) -> bool:
        """Wake waiters."""

    @abstractmethod
    def wait(self, timeout_ms: int = -1) -> WaitResult:
        """Wait for a wakeup token."""

    def drain(self) -> None:
        """Drain queued wake tokens for counting backends."""

    @abstractmethod
    def close(self) -> None:
        """Close local handles."""

    @abstractmethod
    def unlink(self) -> None:
        """Remove the named object when the platform requires it."""


def create_notifier(name: str) -> IpcNotifier:
    """Create a named notifier, replacing a stale one when possible."""
    if os.name == "nt":
        return _WinSemaphoreNotifier(name, create=True)
    return _PosixSemaphoreNotifier(name, create=True)


def open_notifier(name: str) -> IpcNotifier:
    """Open an existing named notifier."""
    if os.name == "nt":
        return _WinSemaphoreNotifier(name, create=False)
    return _PosixSemaphoreNotifier(name, create=False)


def open_or_create_notifier(name: str) -> IpcNotifier:
    try:
        return open_notifier(name)
    except NotifierUnavailable:
        return create_notifier(name)


def _short_name(name: str, prefix: str, max_len: int) -> str:
    clean = str(name).lstrip("/").replace("/", "_")
    encoded = clean.encode("utf-8")
    base = f"{prefix}{clean}"
    if len(base.encode("utf-8")) <= max_len:
        return base
    digest = hashlib.sha1(encoded).hexdigest()[:16]
    prefix_budget = max_len - len(prefix.encode("utf-8")) - len(digest) - 1
    short = encoded[:max(0, prefix_budget)].decode("utf-8", errors="ignore")
    return f"{prefix}{short}_{digest}"[:max_len]


def _resolve_fallback_poll_interval_s() -> float:
    raw = os.environ.get("RYNNRCP_NOTIFIER_FALLBACK_POLL_MS")
    if raw is None:
        return _DEFAULT_FALLBACK_POLL_INTERVAL_MS / 1000.0
    try:
        value_ms = float(raw)
    except (TypeError, ValueError):
        value_ms = _DEFAULT_FALLBACK_POLL_INTERVAL_MS
    return max(_MIN_FALLBACK_POLL_INTERVAL_MS, value_ms) / 1000.0


class _Timespec(ctypes.Structure):
    _fields_ = [
        ("tv_sec", ctypes.c_long),
        ("tv_nsec", ctypes.c_long),
    ]


class _PosixSemaphoreNotifier(IpcNotifier):
    """POSIX named semaphore backend for Linux/macOS."""

    _MAX_NAME_LEN = 30

    def __init__(self, name: str, *, create: bool) -> None:
        self._libc = ctypes.CDLL(None, use_errno=True)
        self._name = "/" + _short_name(name, "rn_", self._MAX_NAME_LEN - 1).lstrip("/")
        self._name_b = self._name.encode("utf-8")
        self._sem = None
        self._closed = False
        self._warned_timedwait_fallback = False
        self._fallback_poll_interval_s = _resolve_fallback_poll_interval_s()
        self._waiter_condition = threading.Condition()
        self._waiter_pending = 0
        self._waiter_error = False
        self._waiter_thread: threading.Thread | None = None
        self._configure_signatures()

        if create:
            try:
                self._sem_unlink(self._name_b)
            except Exception:
                pass
            sem = self._sem_open(self._name_b, os.O_CREAT, ctypes.c_uint(0o600), ctypes.c_uint(0))
        else:
            sem = self._sem_open(self._name_b, 0)
        if self._is_failed_sem(sem):
            err = ctypes.get_errno()
            raise NotifierUnavailable(f"sem_open({self._name}) failed: {os.strerror(err)}")
        self._sem = sem

    def _configure_signatures(self) -> None:
        self._sem_open = self._libc.sem_open
        self._sem_open.restype = ctypes.c_void_p
        self._sem_open.argtypes = [ctypes.c_char_p, ctypes.c_int]

        self._sem_close = self._libc.sem_close
        self._sem_close.argtypes = [ctypes.c_void_p]

        self._sem_unlink = self._libc.sem_unlink
        self._sem_unlink.argtypes = [ctypes.c_char_p]

        self._sem_post = self._libc.sem_post
        self._sem_post.argtypes = [ctypes.c_void_p]

        self._sem_wait = self._libc.sem_wait
        self._sem_wait.argtypes = [ctypes.c_void_p]

        self._sem_trywait = self._libc.sem_trywait
        self._sem_trywait.argtypes = [ctypes.c_void_p]

        self._sem_timedwait = getattr(self._libc, "sem_timedwait", None)
        if self._sem_timedwait is not None:
            self._sem_timedwait.argtypes = [ctypes.c_void_p, ctypes.POINTER(_Timespec)]

    @staticmethod
    def _is_failed_sem(value) -> bool:
        return value in (None, 0, ctypes.c_void_p(-1).value)

    def notify(self) -> bool:
        if self._closed or self._sem is None:
            return False
        return self._sem_post(self._sem) == 0

    def wait(self, timeout_ms: int = -1) -> WaitResult:
        if self._closed or self._sem is None:
            return WaitResult.ERROR
        if timeout_ms == 0:
            if self._sem_timedwait is None:
                with self._waiter_condition:
                    if self._waiter_pending > 0:
                        self._waiter_pending -= 1
                        return WaitResult.SIGNALED
            return WaitResult.SIGNALED if self._try_wait_once() else WaitResult.TIMEOUT
        if timeout_ms < 0:
            if self._sem_timedwait is None:
                return self._wait_with_waiter_thread(timeout_ms)
            return self._wait_forever()
        if self._sem_timedwait is None:
            if not self._warned_timedwait_fallback:
                logger.debug("sem_timedwait unavailable; notifier timed wait uses waiter thread")
                self._warned_timedwait_fallback = True
            return self._wait_with_waiter_thread(timeout_ms)
        return self._timed_wait(timeout_ms)

    def _wait_forever(self) -> WaitResult:
        while self._sem_wait(self._sem) != 0:
            if ctypes.get_errno() != errno.EINTR:
                return WaitResult.ERROR
        return WaitResult.SIGNALED

    def _timed_wait(self, timeout_ms: int) -> WaitResult:
        deadline = time.time() + timeout_ms / 1000.0
        while True:
            sec = int(deadline)
            nsec = int((deadline - sec) * 1_000_000_000)
            ts = _Timespec(sec, nsec)
            if self._sem_timedwait(self._sem, ctypes.byref(ts)) == 0:
                return WaitResult.SIGNALED
            err = ctypes.get_errno()
            if err == errno.EINTR:
                continue
            if err == errno.ETIMEDOUT:
                return WaitResult.TIMEOUT
            return WaitResult.ERROR

    def _fallback_timed_wait(self, timeout_ms: int) -> WaitResult:
        deadline = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < deadline:
            if self._try_wait_once():
                return WaitResult.SIGNALED
            time.sleep(min(self._fallback_poll_interval_s, max(0.0, deadline - time.monotonic())))
        return WaitResult.TIMEOUT

    def _wait_with_waiter_thread(self, timeout_ms: int) -> WaitResult:
        if not self._ensure_waiter_thread():
            return self._fallback_timed_wait(timeout_ms)
        deadline = None if timeout_ms < 0 else time.monotonic() + timeout_ms / 1000.0
        with self._waiter_condition:
            while True:
                if self._waiter_pending > 0:
                    self._waiter_pending -= 1
                    return WaitResult.SIGNALED
                if self._waiter_error:
                    return WaitResult.ERROR
                if self._closed:
                    return WaitResult.ERROR
                if deadline is None:
                    self._waiter_condition.wait()
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return WaitResult.TIMEOUT
                self._waiter_condition.wait(remaining)

    def _ensure_waiter_thread(self) -> bool:
        with self._waiter_condition:
            if self._waiter_thread is not None:
                return True
            if self._closed or self._sem is None:
                return False
            self._waiter_thread = threading.Thread(
                target=self._waiter_loop,
                name=f"notifier_waiter_{self._name.lstrip('/')}",
                daemon=True,
            )
            self._waiter_thread.start()
            return True

    def _waiter_loop(self) -> None:
        while True:
            while self._sem_wait(self._sem) != 0:
                if self._closed:
                    return
                if ctypes.get_errno() == errno.EINTR:
                    continue
                with self._waiter_condition:
                    self._waiter_error = True
                    self._waiter_condition.notify_all()
                return
            with self._waiter_condition:
                if self._closed:
                    self._waiter_condition.notify_all()
                    return
                self._waiter_pending += 1
                self._waiter_condition.notify_all()

    def _try_wait_once(self) -> bool:
        return self._sem_trywait(self._sem) == 0

    def drain(self) -> None:
        if self._closed or self._sem is None:
            return
        if self._sem_timedwait is None:
            with self._waiter_condition:
                self._waiter_pending = 0
        while self._try_wait_once():
            pass

    def close(self) -> None:
        waiter_thread = self._waiter_thread
        if not self._closed:
            self._closed = True
            if waiter_thread is not None and self._sem is not None:
                self._sem_post(self._sem)
            with self._waiter_condition:
                self._waiter_condition.notify_all()
        if waiter_thread is not None and waiter_thread is not threading.current_thread():
            waiter_thread.join(timeout=1.0)
        if self._sem is not None:
            self._sem_close(self._sem)
            self._sem = None

    def unlink(self) -> None:
        try:
            self._sem_unlink(self._name_b)
        except Exception:
            pass


class _WinSemaphoreNotifier(IpcNotifier):
    """Windows named semaphore backend."""

    def __init__(self, name: str, *, create: bool) -> None:
        self._kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        namespace = os.environ.get("RYNNRCP_WIN_OBJECT_NAMESPACE", "Local").strip() or "Local"
        self._name = _short_name(name, f"{namespace}\\rn_", 240)
        self._handle = None
        self._closed = False
        self._configure_signatures()
        if create:
            handle = self._kernel32.CreateSemaphoreW(None, 0, 2**31 - 1, self._name)
        else:
            SEMAPHORE_MODIFY_STATE = 0x0002
            SYNCHRONIZE = 0x00100000
            handle = self._kernel32.OpenSemaphoreW(SEMAPHORE_MODIFY_STATE | SYNCHRONIZE, False, self._name)
        if not handle:
            err = ctypes.get_last_error()
            raise NotifierUnavailable(f"Windows semaphore open failed: {err}")
        self._handle = handle

    def _configure_signatures(self) -> None:
        self._kernel32.CreateSemaphoreW.restype = wintypes.HANDLE
        self._kernel32.CreateSemaphoreW.argtypes = [wintypes.LPVOID, wintypes.LONG, wintypes.LONG, wintypes.LPCWSTR]
        self._kernel32.OpenSemaphoreW.restype = wintypes.HANDLE
        self._kernel32.OpenSemaphoreW.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.LPCWSTR]
        self._kernel32.ReleaseSemaphore.argtypes = [wintypes.HANDLE, wintypes.LONG, wintypes.LPVOID]
        self._kernel32.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        self._kernel32.CloseHandle.argtypes = [wintypes.HANDLE]

    def notify(self) -> bool:
        if self._closed or not self._handle:
            return False
        return bool(self._kernel32.ReleaseSemaphore(self._handle, 1, None))

    def wait(self, timeout_ms: int = -1) -> WaitResult:
        if self._closed or not self._handle:
            return WaitResult.ERROR
        INFINITE = 0xFFFFFFFF
        WAIT_OBJECT_0 = 0
        WAIT_TIMEOUT = 0x102
        timeout = INFINITE if timeout_ms < 0 else int(timeout_ms)
        result = self._kernel32.WaitForSingleObject(self._handle, timeout)
        if result == WAIT_OBJECT_0:
            return WaitResult.SIGNALED
        if result == WAIT_TIMEOUT:
            return WaitResult.TIMEOUT
        return WaitResult.ERROR

    def drain(self) -> None:
        while self.wait(0) == WaitResult.SIGNALED:
            pass

    def close(self) -> None:
        if not self._closed and self._handle:
            self._kernel32.CloseHandle(self._handle)
            self._handle = None
        self._closed = True

    def unlink(self) -> None:
        return None
