// Copyright 2026 RynnRCP Authors. All rights reserved.
// Native primitives: event notification implementation (cross-platform).

#include "event.h"
#include "native_log_defs.h"

#ifdef _WIN32
// ===================== Windows Implementation =====================
#include <windows.h>

namespace rynnrcp {
namespace native {

EventHandle EventCreate() {
    // Auto-reset event: resets after a single waiter is released
    HANDLE h = CreateEventA(nullptr, FALSE, FALSE, nullptr);
    if (h == nullptr) {
        ROBO_LOG_ERROR("CreateEvent failed: %lu", GetLastError());
    }
    return h;
}

bool EventSignal(EventHandle handle) {
    if (handle == kInvalidEvent) return false;
    return SetEvent(static_cast<HANDLE>(handle)) != 0;
}

WaitResult EventWait(EventHandle handle, int timeout_ms) {
    if (handle == kInvalidEvent) return WaitResult::kError;

    DWORD ms = (timeout_ms < 0) ? INFINITE : static_cast<DWORD>(timeout_ms);
    DWORD ret = WaitForSingleObject(static_cast<HANDLE>(handle), ms);

    switch (ret) {
        case WAIT_OBJECT_0:  return WaitResult::kSignaled;
        case WAIT_TIMEOUT:   return WaitResult::kTimeout;
        default:
            ROBO_LOG_ERROR("WaitForSingleObject failed: %lu", GetLastError());
            return WaitResult::kError;
    }
}

void EventClose(EventHandle& handle) {
    if (handle != kInvalidEvent) {
        CloseHandle(static_cast<HANDLE>(handle));
        handle = kInvalidEvent;
    }
}

}  // namespace native
}  // namespace rynnrcp

#else
// ===================== Linux Implementation (eventfd) =====================
#include <unistd.h>
#include <poll.h>
#include <cerrno>
#include <cstring>

#ifdef __linux__
#include <sys/eventfd.h>
#endif

namespace rynnrcp {
namespace native {

EventHandle EventCreate() {
#ifdef __linux__
    int fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (fd < 0) {
        ROBO_LOG_ERROR("eventfd failed: %s", strerror(errno));
    }
    return fd;
#else
    // macOS: eventfd not available, use pipe fallback
    ROBO_LOG_ERROR("macOS eventfd not yet supported");
    return kInvalidEvent;
#endif
}

bool EventSignal(EventHandle handle) {
    if (handle == kInvalidEvent) return false;
#ifdef __linux__
    uint64_t val = 1;
    ssize_t ret = write(handle, &val, sizeof(val));
    return ret == sizeof(val);
#else
    return false;
#endif
}

WaitResult EventWait(EventHandle handle, int timeout_ms) {
    if (handle == kInvalidEvent) return WaitResult::kError;

#ifdef __linux__
    struct pollfd pfd;
    pfd.fd = handle;
    pfd.events = POLLIN;
    pfd.revents = 0;

    int ret = poll(&pfd, 1, timeout_ms);
    if (ret < 0) {
        ROBO_LOG_ERROR("poll failed: %s", strerror(errno));
        return WaitResult::kError;
    }
    if (ret == 0) {
        return WaitResult::kTimeout;
    }

    // Consume the event counter
    uint64_t val;
    ssize_t n = read(handle, &val, sizeof(val));
    if (n != sizeof(val)) {
        return WaitResult::kError;
    }
    return WaitResult::kSignaled;
#else
    return WaitResult::kError;
#endif
}

void EventClose(EventHandle& handle) {
    if (handle != kInvalidEvent) {
        close(handle);
        handle = kInvalidEvent;
    }
}

}  // namespace native
}  // namespace rynnrcp

#endif
