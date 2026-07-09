"""RynnRCP native primitives.

This package contains the OS/native building blocks used by the core runtime:
shared memory, events, cache/page alignment helpers, and optional native
acceleration detection.
"""

from rynnrcp.native.config import (
    CACHE_LINE_SIZE,
    DEFAULT_RING_SLOTS,
    MAX_CHANNEL_NAME_LEN,
    PAGE_SIZE,
    PLATFORM_LINUX,
    PLATFORM_MACOS,
    PLATFORM_WINDOWS,
    POINTER_SIZE,
    align_to_cache_line,
    align_to_page,
    align_up,
    get_page_size,
)
from rynnrcp.native.event import Event, WaitResult
from rynnrcp.native.notifier import (
    IpcNotifier,
    NotifierUnavailable,
    create_notifier,
    open_notifier,
    open_or_create_notifier,
)
from rynnrcp.native.shm import ShmRegion, shm_close, shm_create, shm_open, shm_unlink

__all__ = [
    "CACHE_LINE_SIZE",
    "DEFAULT_RING_SLOTS",
    "Event",
    "IpcNotifier",
    "MAX_CHANNEL_NAME_LEN",
    "NotifierUnavailable",
    "PAGE_SIZE",
    "PLATFORM_LINUX",
    "PLATFORM_MACOS",
    "PLATFORM_WINDOWS",
    "POINTER_SIZE",
    "ShmRegion",
    "WaitResult",
    "align_to_cache_line",
    "align_to_page",
    "align_up",
    "create_notifier",
    "get_page_size",
    "open_notifier",
    "open_or_create_notifier",
    "shm_close",
    "shm_create",
    "shm_open",
    "shm_unlink",
]
