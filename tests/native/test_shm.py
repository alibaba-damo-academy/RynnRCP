# Copyright 2026 RynnRCP Authors. All rights reserved.
# Tests for rynnrcp.native.shm

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rynnrcp.native.shm import ShmRegion, shm_create, shm_open, shm_close, shm_unlink

TEST_SHM_NAME = "rynnrcp_test_shm"


def cleanup():
    """Clean up any leftover SHM from previous failed tests."""
    shm_unlink(TEST_SHM_NAME)


def test_create_write_read_close():
    cleanup()
    region = shm_create(TEST_SHM_NAME, 4096)
    assert region.is_valid(), "Failed to create SHM"
    assert region.size >= 4096

    # Write pattern using typed access
    for i in range(64):
        region.write_u32(i * 4, i * 0xDEAD)

    # Read back
    for i in range(64):
        val = region.read_u32(i * 4)
        assert val == (i * 0xDEAD) & 0xFFFFFFFF, f"Mismatch at {i}: {val}"

    shm_close(region)
    assert not region.is_valid()
    shm_unlink(TEST_SHM_NAME)
    print("[PASS] test_create_write_read_close")


def test_open_existing():
    cleanup()
    creator = shm_create(TEST_SHM_NAME, 8192)
    assert creator.is_valid()

    # Write magic value
    creator.write_u64(0, 0xCAFEBABE12345678)

    # Open same region
    opener = shm_open(TEST_SHM_NAME, 8192)
    assert opener.is_valid()

    # Should see the same data
    val = opener.read_u64(0)
    assert val == 0xCAFEBABE12345678, f"Got {val:#x}"

    # Write from opener, read from creator
    opener.write_u64(8, 0x1111222233334444)
    assert creator.read_u64(8) == 0x1111222233334444

    shm_close(opener)
    shm_close(creator)
    shm_unlink(TEST_SHM_NAME)
    print("[PASS] test_open_existing")


def test_open_nonexistent():
    cleanup()
    region = shm_open(TEST_SHM_NAME, 4096)
    assert not region.is_valid()
    print("[PASS] test_open_nonexistent")


def test_double_create_fails():
    cleanup()
    first = shm_create(TEST_SHM_NAME, 4096)
    assert first.is_valid()

    # Second create should fail
    second = shm_create(TEST_SHM_NAME, 4096)
    assert not second.is_valid()

    shm_close(first)
    shm_unlink(TEST_SHM_NAME)
    print("[PASS] test_double_create_fails")


def test_zero_initialized():
    cleanup()
    region = shm_create(TEST_SHM_NAME, 4096, zero_init=True)
    assert region.is_valid()

    # Check zero-initialized
    data = region.read_bytes(0, 4096)
    assert data == b'\x00' * 4096, "SHM not zero-initialized"

    shm_close(region)
    shm_unlink(TEST_SHM_NAME)
    print("[PASS] test_zero_initialized")


def test_bounds_checks():
    cleanup()
    region = shm_create(TEST_SHM_NAME, 4096)
    assert region.is_valid()

    try:
        region.read_bytes(region.size - 1, 2)
        assert False, "Should have raised ValueError"
    except ValueError:
        pass

    try:
        region.write_u64(region.size - 4, 1)
        assert False, "Should have raised ValueError"
    except ValueError:
        pass

    shm_close(region)
    shm_unlink(TEST_SHM_NAME)
    print("[PASS] test_bounds_checks")


def test_read_view():
    cleanup()
    region = shm_create(TEST_SHM_NAME, 4096)
    assert region.is_valid()

    region.write_bytes(10, b"view-test")
    view = region.read_view(10, 9)
    assert bytes(view) == b"view-test"
    view.release()

    shm_close(region)
    shm_unlink(TEST_SHM_NAME)
    print("[PASS] test_read_view")


def test_open_size_mismatch():
    cleanup()
    creator = shm_create(TEST_SHM_NAME, 4096)
    assert creator.is_valid()

    opener = shm_open(TEST_SHM_NAME, creator.size + 4096)
    assert not opener.is_valid()

    shm_close(creator)
    shm_unlink(TEST_SHM_NAME)
    print("[PASS] test_open_size_mismatch")


def test_large_region():
    cleanup()
    size = 1024 * 1024  # 1MB
    region = shm_create(TEST_SHM_NAME, size)
    assert region.is_valid()
    assert region.size >= size

    # Write at end
    region.write_bytes(region.size - 1, b'\xAB')
    val = region.read_bytes(region.size - 1, 1)
    assert val == b'\xAB'

    shm_close(region)
    shm_unlink(TEST_SHM_NAME)
    print("[PASS] test_large_region")


def test_bytes_rw():
    cleanup()
    region = shm_create(TEST_SHM_NAME, 4096)
    assert region.is_valid()

    msg = b"Hello RynnRCP SHM!"
    region.write_bytes(100, msg)
    result = region.read_bytes(100, len(msg))
    assert result == msg, f"Got {result}"

    shm_close(region)
    shm_unlink(TEST_SHM_NAME)
    print("[PASS] test_bytes_rw")


if __name__ == "__main__":
    print("=== shm tests ===")
    test_create_write_read_close()
    test_open_existing()
    test_open_nonexistent()
    test_double_create_fails()
    test_zero_initialized()
    test_bounds_checks()
    test_read_view()
    test_open_size_mismatch()
    test_large_region()
    test_bytes_rw()
    print("=== All shm tests passed ===")
