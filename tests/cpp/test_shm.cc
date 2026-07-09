// Copyright 2026 RynnRCP Authors. All rights reserved.
// Tests for rynnrcp/native/cpp/shm.h

#include "shm.h"

#include <cassert>
#include <cstdio>
#include <cstring>

using namespace rynnrcp::native;

static const char* kTestShmName = "/rynnrcp_test_shm";

void test_create_write_read_close() {
    // Clean up any leftover from previous failed test
    ShmUnlink(kTestShmName);

    const size_t kSize = 4096;
    ShmRegion region = ShmCreate(kTestShmName, kSize);
    assert(region.IsValid());
    assert(region.size >= kSize);

    // Write a pattern
    uint32_t* data = static_cast<uint32_t*>(region.ptr);
    for (uint32_t i = 0; i < 256; ++i) {
        data[i] = i * 0xDEAD;
    }

    // Read back
    for (uint32_t i = 0; i < 256; ++i) {
        assert(data[i] == i * 0xDEAD);
    }

    ShmClose(region);
    assert(!region.IsValid());

    ShmUnlink(kTestShmName);
    printf("[PASS] test_create_write_read_close\n");
}

void test_open_existing() {
    ShmUnlink(kTestShmName);

    const size_t kSize = 8192;
    ShmRegion creator = ShmCreate(kTestShmName, kSize);
    assert(creator.IsValid());

    // Write magic value
    uint64_t* magic = static_cast<uint64_t*>(creator.ptr);
    *magic = 0xCAFEBABE12345678ULL;

    // Open same region
    ShmRegion opener = ShmOpen(kTestShmName, kSize);
    assert(opener.IsValid());

    // Should see the same data
    uint64_t* read_magic = static_cast<uint64_t*>(opener.ptr);
    assert(*read_magic == 0xCAFEBABE12345678ULL);

    // Write from opener, read from creator
    read_magic[1] = 0x1111222233334444ULL;
    assert(magic[1] == 0x1111222233334444ULL);

    ShmClose(opener);
    ShmClose(creator);
    ShmUnlink(kTestShmName);
    printf("[PASS] test_open_existing\n");
}

void test_open_nonexistent() {
    ShmUnlink(kTestShmName);
    ShmRegion region = ShmOpen(kTestShmName, 4096);
    assert(!region.IsValid());
    printf("[PASS] test_open_nonexistent\n");
}

void test_double_create_fails() {
    ShmUnlink(kTestShmName);

    ShmRegion first = ShmCreate(kTestShmName, 4096);
    assert(first.IsValid());

    // Second create should fail (already exists)
    ShmRegion second = ShmCreate(kTestShmName, 4096);
    assert(!second.IsValid());

    ShmClose(first);
    ShmUnlink(kTestShmName);
    printf("[PASS] test_double_create_fails\n");
}

void test_zero_initialized() {
    ShmUnlink(kTestShmName);

    ShmRegion region = ShmCreate(kTestShmName, 4096);
    assert(region.IsValid());

    // Newly created SHM should be zero-filled
    uint8_t* bytes = static_cast<uint8_t*>(region.ptr);
    for (size_t i = 0; i < 4096; ++i) {
        assert(bytes[i] == 0);
    }

    ShmClose(region);
    ShmUnlink(kTestShmName);
    printf("[PASS] test_zero_initialized\n");
}

void test_large_region() {
    ShmUnlink(kTestShmName);

    // 1 MB region
    const size_t kSize = 1024 * 1024;
    ShmRegion region = ShmCreate(kTestShmName, kSize);
    assert(region.IsValid());
    assert(region.size >= kSize);

    // Write at the end
    uint8_t* bytes = static_cast<uint8_t*>(region.ptr);
    bytes[region.size - 1] = 0xAB;
    assert(bytes[region.size - 1] == 0xAB);

    ShmClose(region);
    ShmUnlink(kTestShmName);
    printf("[PASS] test_large_region\n");
}

int main() {
    printf("=== shm tests ===\n");
    test_create_write_read_close();
    test_open_existing();
    test_open_nonexistent();
    test_double_create_fails();
    test_zero_initialized();
    test_large_region();
    printf("=== All shm tests passed ===\n");
    return 0;
}
