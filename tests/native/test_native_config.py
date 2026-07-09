# Copyright 2026 RynnRCP Authors. All rights reserved.
# Tests for rynnrcp.native.config

import sys
import os

# Add the python source to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "rynnrcp"))
# Actually we need the parent of rynnrcp package
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rynnrcp.native.config import (
    CACHE_LINE_SIZE, MAX_CHANNEL_NAME_LEN, DEFAULT_RING_SLOTS,
    PAGE_SIZE, POINTER_SIZE, PLATFORM_WINDOWS, PLATFORM_LINUX, PLATFORM_MACOS,
    get_page_size, align_up, align_to_cache_line, align_to_page,
)


def test_platform_detection():
    """Exactly one platform flag should be True."""
    flags = [PLATFORM_WINDOWS, PLATFORM_LINUX, PLATFORM_MACOS]
    assert sum(flags) == 1, f"Expected exactly 1 True, got {flags}"
    print(f"[PASS] test_platform_detection: win={PLATFORM_WINDOWS} linux={PLATFORM_LINUX} mac={PLATFORM_MACOS}")


def test_cache_line_size():
    assert CACHE_LINE_SIZE >= 64
    assert CACHE_LINE_SIZE <= 256
    assert (CACHE_LINE_SIZE & (CACHE_LINE_SIZE - 1)) == 0, "Must be power of 2"
    print(f"[PASS] test_cache_line_size: {CACHE_LINE_SIZE} bytes")


def test_page_size():
    ps = get_page_size()
    assert ps >= 4096, f"Page size too small: {ps}"
    assert ps <= 65536, f"Page size too large: {ps}"
    assert (ps & (ps - 1)) == 0, "Must be power of 2"
    assert ps == PAGE_SIZE, "Cached PAGE_SIZE should match get_page_size()"
    print(f"[PASS] test_page_size: {ps} bytes")


def test_pointer_size():
    assert POINTER_SIZE in (4, 8), f"Unexpected pointer size: {POINTER_SIZE}"
    print(f"[PASS] test_pointer_size: {POINTER_SIZE} bytes")


def test_align_up():
    assert align_up(64, 64) == 64
    assert align_up(128, 64) == 128
    assert align_up(1, 64) == 64
    assert align_up(63, 64) == 64
    assert align_up(65, 64) == 128
    assert align_up(0, 64) == 0
    assert align_up(4097, 4096) == 8192
    assert align_up(4096, 4096) == 4096
    print("[PASS] test_align_up")


def test_align_to_cache_line():
    assert align_to_cache_line(1) == CACHE_LINE_SIZE
    assert align_to_cache_line(CACHE_LINE_SIZE) == CACHE_LINE_SIZE
    assert align_to_cache_line(CACHE_LINE_SIZE + 1) == CACHE_LINE_SIZE * 2
    assert align_to_cache_line(0) == 0
    print("[PASS] test_align_to_cache_line")


def test_align_to_page():
    ps = PAGE_SIZE
    assert align_to_page(1) == ps
    assert align_to_page(ps) == ps
    assert align_to_page(ps + 1) == ps * 2
    assert align_to_page(0) == 0
    print("[PASS] test_align_to_page")


def test_constants():
    assert MAX_CHANNEL_NAME_LEN == 64
    assert DEFAULT_RING_SLOTS >= 2
    assert (DEFAULT_RING_SLOTS & (DEFAULT_RING_SLOTS - 1)) == 0
    print("[PASS] test_constants")


if __name__ == "__main__":
    print("=== config tests ===")
    test_platform_detection()
    test_cache_line_size()
    test_page_size()
    test_pointer_size()
    test_align_up()
    test_align_to_cache_line()
    test_align_to_page()
    test_constants()
    print("=== All config tests passed ===")
