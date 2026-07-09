// Copyright 2026 RynnRCP Authors. All rights reserved.
// Tests for rynnrcp/native/cpp/core/ring_buffer.h

#include "ring_buffer.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>
#include <atomic>

using namespace rynnrcp::core;

static void test_create_and_properties() {
    auto rb = RingBuffer::Create("rb_cpp_test", 256, 16);
    assert(rb != nullptr);
    assert(rb->IsValid());
    assert(rb->slot_size() == 256);
    assert(rb->slot_count() == 16);
    assert(rb->name() == "rb_cpp_test");
    assert(rb->WriteCount() == 0);
    assert(rb->LatestIndex() == UINT64_MAX);
    rb->Close();
    rb->Unlink();
    printf("[PASS] test_create_and_properties\n");
}

static void test_write_read_single() {
    auto rb = RingBuffer::Create("rb_cpp_test", 64, 4);
    assert(rb != nullptr);

    const char* msg = "hello ring buffer";
    uint64_t idx = rb->Write(msg, strlen(msg));
    assert(idx == 0);
    assert(rb->WriteCount() == 1);
    assert(rb->LatestIndex() == 0);

    char buf[64] = {};
    size_t out_len = 0;
    bool ok = rb->Read(0, buf, &out_len);
    assert(ok);
    assert(out_len == 64);
    assert(memcmp(buf, msg, strlen(msg)) == 0);
    // Check zero-padding
    assert(buf[strlen(msg)] == '\0');

    rb->Close();
    rb->Unlink();
    printf("[PASS] test_write_read_single\n");
}

static void test_sequential_writes() {
    auto rb = RingBuffer::Create("rb_cpp_test", 32, 8);
    assert(rb != nullptr);

    for (int i = 0; i < 20; ++i) {
        char data[32];
        snprintf(data, sizeof(data), "msg_%d", i);
        uint64_t idx = rb->Write(data, strlen(data));
        assert(idx == (uint64_t)i);
    }
    assert(rb->WriteCount() == 20);
    assert(rb->LatestIndex() == 19);

    // Recent 8 should be readable
    for (int i = 12; i < 20; ++i) {
        char buf[32] = {};
        assert(rb->Read(i, buf, nullptr));
        char expected[32];
        snprintf(expected, sizeof(expected), "msg_%d", i);
        assert(memcmp(buf, expected, strlen(expected)) == 0);
    }

    // Old ones should fail (overwritten)
    char buf[32];
    assert(!rb->Read(0, buf, nullptr));
    assert(!rb->Read(11, buf, nullptr));

    // Future ones should fail
    assert(!rb->Read(20, buf, nullptr));

    rb->Close();
    rb->Unlink();
    printf("[PASS] test_sequential_writes\n");
}

static void test_ring_wrap_around() {
    auto rb = RingBuffer::Create("rb_cpp_test", 16, 4);
    assert(rb != nullptr);

    // Write 4 items (fills the ring)
    for (int i = 0; i < 4; ++i) {
        uint32_t val = i * 100;
        rb->Write(&val, sizeof(val));
    }
    // Read all 4
    for (int i = 0; i < 4; ++i) {
        uint32_t val = 0;
        assert(rb->Read(i, &val, nullptr));
        // val is in a 16-byte slot, but only first 4 bytes matter
        // Actually we wrote uint32_t so the first 4 bytes contain the value
        // We need to read from a 16-byte buffer
        char buf[16] = {};
        assert(rb->Read(i, buf, nullptr));
        uint32_t read_val;
        memcpy(&read_val, buf, sizeof(read_val));
        assert(read_val == (uint32_t)(i * 100));
    }

    // Write 4 more (overwrites first 4)
    for (int i = 4; i < 8; ++i) {
        uint32_t val = i * 100;
        rb->Write(&val, sizeof(val));
    }

    // First 4 should be gone
    char buf[16];
    assert(!rb->Read(0, buf, nullptr));
    assert(!rb->Read(3, buf, nullptr));

    // Last 4 should be readable
    for (int i = 4; i < 8; ++i) {
        char buf2[16] = {};
        assert(rb->Read(i, buf2, nullptr));
        uint32_t read_val;
        memcpy(&read_val, buf2, sizeof(read_val));
        assert(read_val == (uint32_t)(i * 100));
    }

    rb->Close();
    rb->Unlink();
    printf("[PASS] test_ring_wrap_around\n");
}

static void test_open_existing() {
    auto writer = RingBuffer::Create("rb_cpp_test", 128, 8);
    assert(writer != nullptr);

    const char* msg = "shared data";
    writer->Write(msg, strlen(msg));

    // Open as reader
    auto reader = RingBuffer::Open("rb_cpp_test", 128, 8);
    assert(reader != nullptr);
    assert(reader->WriteCount() == 1);

    char buf[128] = {};
    assert(reader->Read(0, buf, nullptr));
    assert(memcmp(buf, msg, strlen(msg)) == 0);

    reader->Close();
    writer->Close();
    writer->Unlink();
    printf("[PASS] test_open_existing\n");
}

static void test_data_too_large() {
    auto rb = RingBuffer::Create("rb_cpp_test", 16, 4);
    assert(rb != nullptr);

    char big_data[32] = {};
    uint64_t idx = rb->Write(big_data, 32);
    assert(idx == UINT64_MAX);  // Should fail

    rb->Close();
    rb->Unlink();
    printf("[PASS] test_data_too_large\n");
}

static void test_invalid_slot_count() {
    // Non-power-of-2
    auto rb = RingBuffer::Create("rb_cpp_test", 64, 3);
    assert(rb == nullptr);

    // Zero
    rb = RingBuffer::Create("rb_cpp_test", 64, 0);
    assert(rb == nullptr);

    printf("[PASS] test_invalid_slot_count\n");
}

static void test_close_and_unlink() {
    auto rb = RingBuffer::Create("rb_cpp_test", 32, 4);
    assert(rb != nullptr);
    assert(rb->IsValid());

    rb->Close();
    assert(!rb->IsValid());

    // Write should fail after close
    char data[32] = {};
    assert(rb->Write(data, 4) == UINT64_MAX);

    rb->Unlink();
    printf("[PASS] test_close_and_unlink\n");
}

static void test_multithreaded_write_read() {
    auto rb = RingBuffer::Create("rb_cpp_test", 64, 64);
    assert(rb != nullptr);

    const int num_writes = 1000;
    std::atomic<bool> done{false};
    std::vector<uint64_t> read_values;
    read_values.reserve(num_writes);

    // Writer thread
    std::thread writer([&]() {
        for (int i = 0; i < num_writes; ++i) {
            uint64_t val = static_cast<uint64_t>(i);
            rb->Write(&val, sizeof(val));
        }
        done.store(true);
    });

    // Reader thread
    std::thread reader([&]() {
        uint64_t next_read = 0;
        while (!done.load() || next_read < (uint64_t)num_writes) {
            char buf[64] = {};
            if (rb->TryRead(next_read, buf, nullptr)) {
                uint64_t val;
                memcpy(&val, buf, sizeof(val));
                read_values.push_back(val);
                next_read++;
            }
            if (next_read >= (uint64_t)num_writes) break;
        }
    });

    writer.join();
    reader.join();

    // Verify we read the correct values (in order)
    assert(read_values.size() == (size_t)num_writes);
    for (int i = 0; i < num_writes; ++i) {
        assert(read_values[i] == (uint64_t)i);
    }

    rb->Close();
    rb->Unlink();
    printf("[PASS] test_multithreaded_write_read\n");
}

int main() {
    printf("=== C++ RingBuffer tests ===\n");
    test_create_and_properties();
    test_write_read_single();
    test_sequential_writes();
    test_ring_wrap_around();
    test_open_existing();
    test_data_too_large();
    test_invalid_slot_count();
    test_close_and_unlink();
    test_multithreaded_write_read();
    printf("=== All C++ RingBuffer tests passed ===\n");
    return 0;
}
