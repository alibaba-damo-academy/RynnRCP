# Copyright 2026 RynnRCP Authors. All rights reserved.
# Platform abstraction: compile-time and runtime platform constants.

import os
import sys
import struct

# ---------------------------------------------------------------------------
# Platform detection
# ---------------------------------------------------------------------------
PLATFORM_WINDOWS = sys.platform == "win32"
PLATFORM_LINUX = sys.platform == "linux"
PLATFORM_MACOS = sys.platform == "darwin"

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Cache line size (bytes). Conservative default for x86-64 / ARM Cortex-A.
CACHE_LINE_SIZE: int = 64

# Maximum length for a channel name (including null terminator)
MAX_CHANNEL_NAME_LEN: int = 64

# Default SHM ring buffer slot count (must be power of 2)
DEFAULT_RING_SLOTS: int = 16

# Pointer size for this platform
POINTER_SIZE: int = struct.calcsize("P")

# ---------------------------------------------------------------------------
# Runtime platform queries
# ---------------------------------------------------------------------------

def get_page_size() -> int:
    """Get the system page size in bytes."""
    if PLATFORM_WINDOWS:
        return _get_windows_page_size()
    return os.sysconf("SC_PAGESIZE")


def _get_windows_page_size() -> int:
    import ctypes

    class SYSTEM_INFO(ctypes.Structure):
        _fields_ = [
            ("wProcessorArchitecture", ctypes.c_ushort),
            ("wReserved", ctypes.c_ushort),
            ("dwPageSize", ctypes.c_ulong),
            ("lpMinimumApplicationAddress", ctypes.c_void_p),
            ("lpMaximumApplicationAddress", ctypes.c_void_p),
            ("dwActiveProcessorMask", ctypes.c_size_t),
            ("dwNumberOfProcessors", ctypes.c_ulong),
            ("dwProcessorType", ctypes.c_ulong),
            ("dwAllocationGranularity", ctypes.c_ulong),
            ("wProcessorLevel", ctypes.c_ushort),
            ("wProcessorRevision", ctypes.c_ushort),
        ]

    si = SYSTEM_INFO()
    ctypes.windll.kernel32.GetSystemInfo(ctypes.byref(si))
    return si.dwPageSize


# Cache the page size (it never changes at runtime)
PAGE_SIZE: int = get_page_size()


# ---------------------------------------------------------------------------
# Alignment helpers
# ---------------------------------------------------------------------------

def align_up(size: int, alignment: int) -> int:
    """Align `size` up to the nearest multiple of `alignment`.
    `alignment` must be a power of 2."""
    assert alignment > 0 and (alignment & (alignment - 1)) == 0, \
        f"alignment must be a power of 2, got {alignment}"
    return (size + alignment - 1) & ~(alignment - 1)


def align_to_cache_line(size: int) -> int:
    """Align `size` up to cache line boundary."""
    return align_up(size, CACHE_LINE_SIZE)


def align_to_page(size: int) -> int:
    """Align `size` up to the system page size."""
    return align_up(size, PAGE_SIZE)
