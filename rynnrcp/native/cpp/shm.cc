// Copyright 2026 RynnRCP Authors. All rights reserved.
// Native primitives: shared memory implementation (cross-platform).

#include "shm.h"
#include "config.h"
#include "native_log_defs.h"

#include <cstring>

#ifdef _WIN32
// ===================== Windows Implementation =====================
#include <windows.h>

namespace rynnrcp {
namespace native {

// Strip leading '/' for Windows named objects (Global\\ prefix optional)
static std::string WinShmName(const std::string& name) {
    std::string result = name;
    if (!result.empty() && result[0] == '/') {
        result = result.substr(1);
    }
    // Replace remaining '/' with '_'
    for (auto& c : result) {
        if (c == '/') c = '_';
    }
    return "Local\\" + result;
}

ShmRegion ShmCreate(const std::string& name, size_t size) {
    ShmRegion region;
    size = AlignToPage(size);

    std::string win_name = WinShmName(name);

    HANDLE hMap = CreateFileMappingA(
        INVALID_HANDLE_VALUE,   // backed by page file
        nullptr,                // default security
        PAGE_READWRITE,
        static_cast<DWORD>(size >> 32),
        static_cast<DWORD>(size & 0xFFFFFFFF),
        win_name.c_str()
    );

    if (hMap == nullptr) {
        ROBO_LOG_ERROR("CreateFileMapping failed: %lu", GetLastError());
        return region;
    }

    // Check if it already existed
    if (GetLastError() == ERROR_ALREADY_EXISTS) {
        CloseHandle(hMap);
        ROBO_LOG_WARN("SHM '%s' already exists", name.c_str());
        return region;
    }

    void* ptr = MapViewOfFile(hMap, FILE_MAP_ALL_ACCESS, 0, 0, size);
    if (ptr == nullptr) {
        ROBO_LOG_ERROR("MapViewOfFile failed: %lu", GetLastError());
        CloseHandle(hMap);
        return region;
    }

    // Zero-initialize
    memset(ptr, 0, size);

    region.ptr = ptr;
    region.size = size;
    region.handle = hMap;
    return region;
}

ShmRegion ShmOpen(const std::string& name, size_t size) {
    ShmRegion region;
    size = AlignToPage(size);

    std::string win_name = WinShmName(name);

    HANDLE hMap = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, win_name.c_str());
    if (hMap == nullptr) {
        ROBO_LOG_ERROR("OpenFileMapping failed: %lu", GetLastError());
        return region;
    }

    void* ptr = MapViewOfFile(hMap, FILE_MAP_ALL_ACCESS, 0, 0, size);
    if (ptr == nullptr) {
        ROBO_LOG_ERROR("MapViewOfFile failed: %lu", GetLastError());
        CloseHandle(hMap);
        return region;
    }

    region.ptr = ptr;
    region.size = size;
    region.handle = hMap;
    return region;
}

void ShmClose(ShmRegion& region) {
    if (region.ptr) {
        UnmapViewOfFile(region.ptr);
        region.ptr = nullptr;
    }
    if (region.handle) {
        CloseHandle(static_cast<HANDLE>(region.handle));
        region.handle = nullptr;
    }
    region.size = 0;
}

void ShmUnlink(const std::string& /*name*/) {
    // On Windows, the shared memory object is automatically destroyed
    // when all handles are closed. No explicit unlink needed.
    // This is a no-op for API compatibility.
}

}  // namespace native
}  // namespace rynnrcp

#else
// ===================== POSIX (Linux/macOS) Implementation =====================
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

namespace rynnrcp {
namespace native {

ShmRegion ShmCreate(const std::string& name, size_t size) {
    ShmRegion region;
    size = AlignToPage(size);

    // Create exclusively (O_EXCL ensures it doesn't already exist)
    int fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
    if (fd < 0) {
        ROBO_LOG_ERROR("shm_open failed: %s", strerror(errno));
        return region;
    }

    if (ftruncate(fd, static_cast<off_t>(size)) != 0) {
        ROBO_LOG_ERROR("ftruncate failed: %s", strerror(errno));
        close(fd);
        shm_unlink(name.c_str());
        return region;
    }

    void* ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) {
        ROBO_LOG_ERROR("mmap failed: %s", strerror(errno));
        close(fd);
        shm_unlink(name.c_str());
        return region;
    }

    // Zero-initialize
    memset(ptr, 0, size);

    region.ptr = ptr;
    region.size = size;
    region.fd = fd;
    return region;
}

ShmRegion ShmOpen(const std::string& name, size_t size) {
    ShmRegion region;
    size = AlignToPage(size);

    int fd = shm_open(name.c_str(), O_RDWR, 0666);
    if (fd < 0) {
        ROBO_LOG_ERROR("shm_open failed: %s", strerror(errno));
        return region;
    }

    void* ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) {
        ROBO_LOG_ERROR("mmap failed: %s", strerror(errno));
        close(fd);
        return region;
    }

    region.ptr = ptr;
    region.size = size;
    region.fd = fd;
    return region;
}

void ShmClose(ShmRegion& region) {
    if (region.ptr) {
        munmap(region.ptr, region.size);
        region.ptr = nullptr;
    }
    if (region.fd >= 0) {
        close(region.fd);
        region.fd = -1;
    }
    region.size = 0;
}

void ShmUnlink(const std::string& name) {
    shm_unlink(name.c_str());
}

}  // namespace native
}  // namespace rynnrcp

#endif
