// Copyright 2026 RynnRCP Authors. All rights reserved.
// Native primitives: unified logging framework (cross-platform).
// Header-only macros + sink-based dispatch.

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <mutex>
#include <atomic>

namespace rynnrcp {
namespace native {
namespace log {

// ---------------------------------------------------------------------------
// Log levels
// ---------------------------------------------------------------------------
enum class LogLevel : int {
    kDebug = 0,
    kInfo  = 1,
    kWarn  = 2,
    kError = 3,
    kFatal = 4,
};

inline const char* LogLevelToString(LogLevel level) {
    switch (level) {
        case LogLevel::kDebug: return "DEBUG";
        case LogLevel::kInfo:  return "INFO";
        case LogLevel::kWarn:  return "WARN";
        case LogLevel::kError: return "ERROR";
        case LogLevel::kFatal: return "FATAL";
        default:               return "?????";
    }
}

// ---------------------------------------------------------------------------
// Log record
// ---------------------------------------------------------------------------
struct LogRecord {
    LogLevel    level;
    int64_t     timestamp_us;   // microseconds since epoch
    uint32_t    pid;
    uint32_t    tid;
    char        module[32];
    char        file[64];
    int         line;
    char        message[512];
};

// ---------------------------------------------------------------------------
// LogSink - abstract base class
// ---------------------------------------------------------------------------
class LogSink {
public:
    virtual ~LogSink() = default;
    virtual void Write(const LogRecord& record) = 0;
};

// ---------------------------------------------------------------------------
// StderrSink - terminal output with optional ANSI color
// ---------------------------------------------------------------------------
class StderrSink : public LogSink {
public:
    explicit StderrSink(bool colored = true) : colored_(colored) {}
    void Write(const LogRecord& record) override;
private:
    bool colored_;
};

// ---------------------------------------------------------------------------
// FileSink - per-process file output with size-based rotation
// ---------------------------------------------------------------------------
class FileSink : public LogSink {
public:
    /// @param path File path (e.g. "logs/main.log")
    /// @param max_size_bytes Max file size before rotation (default 50MB)
    /// @param max_files Number of rotated files to keep (default 5)
    FileSink(const std::string& path, size_t max_size_bytes = 50 * 1024 * 1024, int max_files = 5);
    ~FileSink() override;
    void Write(const LogRecord& record) override;
private:
    void Rotate();
    std::string path_;
    size_t max_size_bytes_;
    int max_files_;
    FILE* fp_ = nullptr;
    size_t current_size_ = 0;
    std::mutex file_mutex_;
};

// ---------------------------------------------------------------------------
// Global sink management
// ---------------------------------------------------------------------------

/// Add a sink to the global chain. Thread-safe.
void AddLogSink(std::shared_ptr<LogSink> sink);

/// Remove a sink from the global chain. Thread-safe.
void RemoveLogSink(std::shared_ptr<LogSink> sink);

/// Set global minimum log level. Messages below this are discarded.
void SetGlobalLogLevel(LogLevel min_level);

/// Get current global log level.
LogLevel GetGlobalLogLevel();

/// Dispatch a log record to all registered sinks.
void LogDispatch(const LogRecord& record);

// ---------------------------------------------------------------------------
// Utility: get current timestamp in microseconds
// ---------------------------------------------------------------------------
inline int64_t NowMicros() {
    auto now = std::chrono::system_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch());
    return us.count();
}

// ---------------------------------------------------------------------------
// Utility: get PID and TID (platform-independent declarations)
// ---------------------------------------------------------------------------
uint32_t GetPid();
uint32_t GetTid();

// ---------------------------------------------------------------------------
// Core log implementation function
// ---------------------------------------------------------------------------
inline void _robo_log_impl(LogLevel level, const char* module,
                           const char* file, int line,
                           const char* fmt, ...) {
    // Early level check
    if (static_cast<int>(level) < static_cast<int>(GetGlobalLogLevel())) {
        return;
    }

    LogRecord record;
    record.level = level;
    record.timestamp_us = NowMicros();
    record.pid = GetPid();
    record.tid = GetTid();
    record.line = line;

    // Copy module name
    strncpy(record.module, module ? module : "", sizeof(record.module) - 1);
    record.module[sizeof(record.module) - 1] = '\0';

    // Copy file name
    strncpy(record.file, file ? file : "", sizeof(record.file) - 1);
    record.file[sizeof(record.file) - 1] = '\0';

    // Format message
    va_list args;
    va_start(args, fmt);
    vsnprintf(record.message, sizeof(record.message), fmt, args);
    va_end(args);

    LogDispatch(record);
}

}  // namespace log
}  // namespace native
}  // namespace rynnrcp

// ---------------------------------------------------------------------------
// Convenience macros - require ROBO_LOG_MODULE to be defined before use
// ---------------------------------------------------------------------------

// Portable __FILE_NAME__ fallback for compilers that don't support it
#ifndef __FILE_NAME__
#ifdef _WIN32
#define __FILE_NAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#else
#define __FILE_NAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif
#endif

#define ROBO_LOG_DEBUG(fmt, ...) \
    ::rynnrcp::native::log::_robo_log_impl( \
        ::rynnrcp::native::log::LogLevel::kDebug, \
        ROBO_LOG_MODULE, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)

#define ROBO_LOG_INFO(fmt, ...) \
    ::rynnrcp::native::log::_robo_log_impl( \
        ::rynnrcp::native::log::LogLevel::kInfo, \
        ROBO_LOG_MODULE, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)

#define ROBO_LOG_WARN(fmt, ...) \
    ::rynnrcp::native::log::_robo_log_impl( \
        ::rynnrcp::native::log::LogLevel::kWarn, \
        ROBO_LOG_MODULE, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)

#define ROBO_LOG_ERROR(fmt, ...) \
    ::rynnrcp::native::log::_robo_log_impl( \
        ::rynnrcp::native::log::LogLevel::kError, \
        ROBO_LOG_MODULE, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)

#define ROBO_LOG_FATAL(fmt, ...) \
    ::rynnrcp::native::log::_robo_log_impl( \
        ::rynnrcp::native::log::LogLevel::kFatal, \
        ROBO_LOG_MODULE, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)
