// Copyright 2026 RynnRCP Authors. All rights reserved.
// Native primitives: logging implementation.

#include "log.h"

#include <ctime>
#include <algorithm>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <sys/syscall.h>
#endif

namespace rynnrcp {
namespace native {
namespace log {

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
namespace {

struct LogState {
    std::mutex mutex;
    std::vector<std::shared_ptr<LogSink>> sinks;
    std::atomic<int> min_level{static_cast<int>(LogLevel::kDebug)};
    bool initialized = false;
};

LogState& GetState() {
    static LogState state;
    return state;
}

/// Ensure at least one sink (stderr) exists by default.
void EnsureDefaultSink() {
    auto& s = GetState();
    if (!s.initialized) {
        s.sinks.push_back(std::make_shared<StderrSink>(true));
        s.initialized = true;
    }
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// Global sink management
// ---------------------------------------------------------------------------

void AddLogSink(std::shared_ptr<LogSink> sink) {
    auto& s = GetState();
    std::lock_guard<std::mutex> lock(s.mutex);
    s.initialized = true;
    s.sinks.push_back(std::move(sink));
}

void RemoveLogSink(std::shared_ptr<LogSink> sink) {
    auto& s = GetState();
    std::lock_guard<std::mutex> lock(s.mutex);
    auto it = std::find(s.sinks.begin(), s.sinks.end(), sink);
    if (it != s.sinks.end()) {
        s.sinks.erase(it);
    }
}

void SetGlobalLogLevel(LogLevel min_level) {
    GetState().min_level.store(static_cast<int>(min_level), std::memory_order_relaxed);
}

LogLevel GetGlobalLogLevel() {
    return static_cast<LogLevel>(GetState().min_level.load(std::memory_order_relaxed));
}

void LogDispatch(const LogRecord& record) {
    auto& s = GetState();
    std::lock_guard<std::mutex> lock(s.mutex);
    if (s.sinks.empty() && !s.initialized) {
        s.sinks.push_back(std::make_shared<StderrSink>(true));
        s.initialized = true;
    }
    for (auto& sink : s.sinks) {
        sink->Write(record);
    }
}

// ---------------------------------------------------------------------------
// StderrSink implementation
// ---------------------------------------------------------------------------

void StderrSink::Write(const LogRecord& record) {
    // Format timestamp
    int64_t sec = record.timestamp_us / 1000000;
    int us = static_cast<int>(record.timestamp_us % 1000000);
    time_t t = static_cast<time_t>(sec);
    struct tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &t);
#else
    localtime_r(&t, &tm_buf);
#endif

    char ts[32];
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm_buf);

    // ANSI color codes
    const char* color_start = "";
    const char* color_end = "";
    if (colored_) {
        switch (record.level) {
            case LogLevel::kDebug: color_start = "\033[90m"; break;  // gray
            case LogLevel::kInfo:  color_start = "\033[32m"; break;  // green
            case LogLevel::kWarn:  color_start = "\033[33m"; break;  // yellow
            case LogLevel::kError: color_start = "\033[31m"; break;  // red
            case LogLevel::kFatal: color_start = "\033[35m"; break;  // magenta
        }
        color_end = "\033[0m";
    }

    fprintf(stderr, "%s[%s.%03d] [PID:%u] [TID:%u] [%-5s] [%s] %s:%d %s%s\n",
            color_start,
            ts, us / 1000,
            record.pid, record.tid,
            LogLevelToString(record.level),
            record.module,
            record.file, record.line,
            record.message,
            color_end);
}

// ---------------------------------------------------------------------------
// FileSink implementation
// ---------------------------------------------------------------------------

FileSink::FileSink(const std::string& path, size_t max_size_bytes, int max_files)
    : path_(path), max_size_bytes_(max_size_bytes), max_files_(max_files) {
    fp_ = fopen(path.c_str(), "a");
    if (fp_) {
        fseek(fp_, 0, SEEK_END);
        current_size_ = static_cast<size_t>(ftell(fp_));
    }
}

FileSink::~FileSink() {
    if (fp_) fclose(fp_);
}

void FileSink::Write(const LogRecord& record) {
    std::lock_guard lock(file_mutex_);
    if (!fp_) return;

    // Format timestamp
    int64_t sec = record.timestamp_us / 1000000;
    int us = static_cast<int>(record.timestamp_us % 1000000);
    time_t t = static_cast<time_t>(sec);
    struct tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &t);
#else
    localtime_r(&t, &tm_buf);
#endif

    char ts[32];
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm_buf);

    int written = fprintf(fp_, "[%s.%03d] [PID:%u] [TID:%u] [%-5s] [%s] %s:%d %s\n",
            ts, us / 1000,
            record.pid, record.tid,
            LogLevelToString(record.level),
            record.module,
            record.file, record.line,
            record.message);

    if (written > 0) {
        current_size_ += static_cast<size_t>(written);
        fflush(fp_);
    }

    if (current_size_ >= max_size_bytes_) {
        Rotate();
    }
}

void FileSink::Rotate() {
    if (fp_) {
        fclose(fp_);
        fp_ = nullptr;
    }

    // Rotate existing files: .log.4 -> .log.5, .log.3 -> .log.4, etc.
    for (int i = max_files_ - 1; i >= 1; --i) {
        std::string src = path_ + "." + std::to_string(i);
        std::string dst = path_ + "." + std::to_string(i + 1);
        std::remove(dst.c_str());
        std::rename(src.c_str(), dst.c_str());
    }

    // Current file becomes .1
    std::string dst = path_ + ".1";
    std::remove(dst.c_str());
    std::rename(path_.c_str(), dst.c_str());

    // Open fresh file
    fp_ = fopen(path_.c_str(), "w");
    current_size_ = 0;
}

// ---------------------------------------------------------------------------
// Platform-specific PID/TID
// ---------------------------------------------------------------------------

uint32_t GetPid() {
#ifdef _WIN32
    return static_cast<uint32_t>(GetCurrentProcessId());
#else
    return static_cast<uint32_t>(getpid());
#endif
}

uint32_t GetTid() {
#ifdef _WIN32
    return static_cast<uint32_t>(GetCurrentThreadId());
#elif defined(__linux__)
    return static_cast<uint32_t>(syscall(SYS_gettid));
#else
    // macOS fallback
    uint64_t tid;
    pthread_threadid_np(nullptr, &tid);
    return static_cast<uint32_t>(tid);
#endif
}

}  // namespace log
}  // namespace native
}  // namespace rynnrcp
