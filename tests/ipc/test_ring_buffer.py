# Copyright 2026 RynnRCP Authors. All rights reserved.
# Tests for rynnrcp.ipc.ring_buffer

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rynnrcp.ipc.ring_buffer import RingBuffer

TEST_NAME = "rb_test"


def cleanup():
    try:
        rb = RingBuffer(TEST_NAME, 64, 4, create=False)
        rb.close()
        rb.unlink()
    except Exception:
        pass
    from rynnrcp.native.shm import shm_unlink
    shm_unlink(TEST_NAME)


def test_create_and_properties():
    cleanup()
    rb = RingBuffer(TEST_NAME, slot_size=128, slot_count=8, create=True)
    assert rb.is_valid()
    assert rb.slot_size == 128
    assert rb.slot_count == 8
    assert rb.name == TEST_NAME
    assert rb.write_count() == 0
    assert rb.latest_index() == -1
    rb.close()
    rb.unlink()
    print("[PASS] test_create_and_properties")


def test_write_read_single():
    cleanup()
    rb = RingBuffer(TEST_NAME, slot_size=32, slot_count=4, create=True)

    data = b"Hello RynnRCP!"
    idx = rb.write(data)
    assert idx == 0
    assert rb.write_count() == 1
    assert rb.latest_index() == 0

    result = rb.read(0)
    # Data is zero-padded to slot_size
    assert result[:len(data)] == data
    assert result[len(data):] == b'\x00' * (32 - len(data))

    rb.close()
    rb.unlink()
    print("[PASS] test_write_read_single")


def test_sequential_writes():
    cleanup()
    rb = RingBuffer(TEST_NAME, slot_size=16, slot_count=4, create=True)

    for i in range(10):
        data = f"msg_{i:04d}".encode()
        idx = rb.write(data)
        assert idx == i

    assert rb.write_count() == 10
    assert rb.latest_index() == 9

    # Only the last 4 messages should be readable (slot_count=4)
    for i in range(6, 10):
        result = rb.read(i)
        expected = f"msg_{i:04d}".encode()
        assert result[:len(expected)] == expected

    # Older messages should raise IndexError
    try:
        rb.read(5)
        assert False, "Should have raised IndexError"
    except IndexError:
        pass

    rb.close()
    rb.unlink()
    print("[PASS] test_sequential_writes")


def test_ring_wrap_around():
    cleanup()
    rb = RingBuffer(TEST_NAME, slot_size=8, slot_count=4, create=True)

    # Write exactly slot_count messages
    for i in range(4):
        rb.write(i.to_bytes(8, "little"))

    # All 4 should be readable
    for i in range(4):
        data = rb.read(i)
        val = int.from_bytes(data, "little")
        assert val == i

    # Write one more (overwrites slot 0)
    rb.write((99).to_bytes(8, "little"))

    # Index 0 is now overwritten
    assert rb.try_read(0) is None

    # Index 4 (latest) should be 99
    data = rb.read(4)
    assert int.from_bytes(data, "little") == 99

    rb.close()
    rb.unlink()
    print("[PASS] test_ring_wrap_around")


def test_try_read():
    cleanup()
    rb = RingBuffer(TEST_NAME, slot_size=8, slot_count=4, create=True)

    # try_read on unwritten index returns None
    assert rb.try_read(0) is None
    assert rb.try_read(100) is None

    rb.write(b'\x42' * 8)
    assert rb.try_read(0) is not None
    assert rb.try_read(1) is None

    rb.close()
    rb.unlink()
    print("[PASS] test_try_read")


def test_read_exact_and_oldest_index():
    cleanup()
    rb = RingBuffer(TEST_NAME, slot_size=32, slot_count=4, create=True)

    assert rb.oldest_index() == -1
    for i in range(6):
        rb.write(f"payload-{i}".encode())

    assert rb.oldest_index() == 2
    assert rb.read_exact(5, 9) == b"payload-5"
    assert rb.try_read(1) is None

    rb.close()
    rb.unlink()
    print("[PASS] test_read_exact_and_oldest_index")


def test_data_too_large():
    cleanup()
    rb = RingBuffer(TEST_NAME, slot_size=8, slot_count=4, create=True)

    try:
        rb.write(b'\x00' * 16)  # 16 > slot_size=8
        assert False, "Should have raised ValueError"
    except ValueError:
        pass

    rb.close()
    rb.unlink()
    print("[PASS] test_data_too_large")


def test_open_existing():
    cleanup()
    rb_writer = RingBuffer(TEST_NAME, slot_size=32, slot_count=4, create=True)
    rb_writer.write(b"shared_data_test")

    # Open same buffer as reader
    rb_reader = RingBuffer(TEST_NAME, slot_size=32, slot_count=4, create=False)
    assert rb_reader.is_valid()
    assert rb_reader.latest_index() == 0

    data = rb_reader.read(0)
    assert data[:16] == b"shared_data_test"

    rb_reader.close()
    rb_writer.close()
    rb_writer.unlink()
    print("[PASS] test_open_existing")


def test_close_and_unlink():
    cleanup()
    rb = RingBuffer(TEST_NAME, slot_size=16, slot_count=4, create=True)
    rb.write(b"test")
    rb.close()
    assert not rb.is_valid()

    try:
        rb.write(b"fail")
        assert False, "Should have raised"
    except RuntimeError:
        pass

    rb.unlink()
    print("[PASS] test_close_and_unlink")


def test_multithreaded_write_read():
    import threading
    cleanup()
    rb = RingBuffer(TEST_NAME, slot_size=16, slot_count=64, create=True)

    num_writes = 200
    errors = []

    def writer():
        for i in range(num_writes):
            rb.write(i.to_bytes(8, "little") + b'\x00' * 8)

    def reader():
        seen = set()
        last_checked = 0
        while last_checked < num_writes:
            li = rb.latest_index()
            if li < 0:
                continue
            # Read the latest available
            data = rb.try_read(li)
            if data is not None:
                val = int.from_bytes(data[:8], "little")
                seen.add(val)
            last_checked = li + 1
        if len(seen) == 0:
            errors.append("Reader saw no data")

    wt = threading.Thread(target=writer)
    rt = threading.Thread(target=reader)
    rt.start()
    wt.start()
    wt.join()
    rt.join()

    assert len(errors) == 0, errors
    assert rb.write_count() == num_writes
    rb.close()
    rb.unlink()
    print("[PASS] test_multithreaded_write_read")


if __name__ == "__main__":
    print("=== ring_buffer tests ===")
    test_create_and_properties()
    test_write_read_single()
    test_sequential_writes()
    test_ring_wrap_around()
    test_try_read()
    test_read_exact_and_oldest_index()
    test_data_too_large()
    test_open_existing()
    test_close_and_unlink()
    test_multithreaded_write_read()
    print("=== All ring_buffer tests passed ===")
