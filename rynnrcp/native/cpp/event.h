// Copyright 2026 RynnRCP Authors. All rights reserved.
// Native primitives: lightweight event notification (cross-platform).
// Linux: eventfd
// Windows: CreateEvent (auto-reset)

#pragma once

#include <cstdint>

namespace rynnrcp {
namespace native {

/// Opaque event handle
#ifdef _WIN32
using EventHandle = void*;  // HANDLE
constexpr EventHandle kInvalidEvent = nullptr;
#else
using EventHandle = int;     // file descriptor
constexpr EventHandle kInvalidEvent = -1;
#endif

/// Wait result
enum class WaitResult {
    kSignaled,   ///< Event was signaled
    kTimeout,    ///< Timed out before signal
    kError       ///< System error
};

/// Create a new event object.
/// @return Valid handle, or kInvalidEvent on failure.
EventHandle EventCreate();

/// Signal the event (wake one waiter).
/// @return true on success.
bool EventSignal(EventHandle handle);

/// Wait for the event to be signaled.
/// @param handle     Event handle.
/// @param timeout_ms Timeout in milliseconds. 0 = non-blocking poll, -1 = infinite wait.
/// @return WaitResult indicating outcome.
WaitResult EventWait(EventHandle handle, int timeout_ms = -1);

/// Close and release the event handle.
void EventClose(EventHandle& handle);

}  // namespace native
}  // namespace rynnrcp
