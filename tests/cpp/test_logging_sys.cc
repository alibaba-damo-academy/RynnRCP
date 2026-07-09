// Copyright 2026 RynnRCP Authors. All rights reserved.
// Tests for native logging system.

#define ROBO_LOG_MODULE "test"
#include "log.h"

#include <cassert>
#include <cstring>
#include <iostream>
#include <sstream>
#include <vector>

using namespace rynnrcp::native::log;

// ---------------------------------------------------------------------------
// Test Sink - captures log records for assertion
// ---------------------------------------------------------------------------
class TestSink : public LogSink {
public:
    void Write(const LogRecord& record) override {
        records.push_back(record);
    }
    std::vector<LogRecord> records;
};

// ---------------------------------------------------------------------------
// Test helpers
// ---------------------------------------------------------------------------
static void ClearSinks() {
    // Reset by setting level to debug (sinks are managed per-test)
    SetGlobalLogLevel(LogLevel::kDebug);
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

void test_log_level_filtering() {
    auto sink = std::make_shared<TestSink>();
    AddLogSink(sink);
    SetGlobalLogLevel(LogLevel::kWarn);

    ROBO_LOG_DEBUG("This should be filtered");
    ROBO_LOG_INFO("This should be filtered");
    ROBO_LOG_WARN("This should pass");
    ROBO_LOG_ERROR("This should pass too");

    assert(sink->records.size() == 2);
    assert(sink->records[0].level == LogLevel::kWarn);
    assert(sink->records[1].level == LogLevel::kError);

    RemoveLogSink(sink);
    SetGlobalLogLevel(LogLevel::kDebug);
    std::cout << "  PASS: test_log_level_filtering" << std::endl;
}

void test_log_record_fields() {
    auto sink = std::make_shared<TestSink>();
    AddLogSink(sink);
    SetGlobalLogLevel(LogLevel::kDebug);

    ROBO_LOG_INFO("Hello %s %d", "world", 42);

    assert(sink->records.size() == 1);
    auto& rec = sink->records[0];
    assert(rec.level == LogLevel::kInfo);
    assert(strcmp(rec.module, "test") == 0);
    assert(rec.line > 0);
    assert(strstr(rec.message, "Hello world 42") != nullptr);
    assert(rec.pid > 0);
    assert(rec.tid > 0);
    assert(rec.timestamp_us > 0);

    // file should contain this test file name
    assert(strstr(rec.file, "test_log") != nullptr);

    RemoveLogSink(sink);
    std::cout << "  PASS: test_log_record_fields" << std::endl;
}

void test_multiple_sinks() {
    auto sink1 = std::make_shared<TestSink>();
    auto sink2 = std::make_shared<TestSink>();
    AddLogSink(sink1);
    AddLogSink(sink2);
    SetGlobalLogLevel(LogLevel::kDebug);

    ROBO_LOG_ERROR("Both sinks should get this");

    assert(sink1->records.size() == 1);
    assert(sink2->records.size() == 1);
    assert(strcmp(sink1->records[0].message, sink2->records[0].message) == 0);

    RemoveLogSink(sink1);
    RemoveLogSink(sink2);
    std::cout << "  PASS: test_multiple_sinks" << std::endl;
}

void test_remove_sink() {
    auto sink = std::make_shared<TestSink>();
    AddLogSink(sink);
    SetGlobalLogLevel(LogLevel::kDebug);

    ROBO_LOG_INFO("Before remove");
    assert(sink->records.size() == 1);

    RemoveLogSink(sink);

    ROBO_LOG_INFO("After remove");
    assert(sink->records.size() == 1);  // Should not get the second message

    std::cout << "  PASS: test_remove_sink" << std::endl;
}

void test_log_level_to_string() {
    assert(strcmp(LogLevelToString(LogLevel::kDebug), "DEBUG") == 0);
    assert(strcmp(LogLevelToString(LogLevel::kInfo), "INFO") == 0);
    assert(strcmp(LogLevelToString(LogLevel::kWarn), "WARN") == 0);
    assert(strcmp(LogLevelToString(LogLevel::kError), "ERROR") == 0);
    assert(strcmp(LogLevelToString(LogLevel::kFatal), "FATAL") == 0);
    std::cout << "  PASS: test_log_level_to_string" << std::endl;
}

void test_long_message_truncation() {
    auto sink = std::make_shared<TestSink>();
    AddLogSink(sink);
    SetGlobalLogLevel(LogLevel::kDebug);

    // Create a very long message (longer than buffer)
    std::string long_msg(1000, 'X');
    ROBO_LOG_INFO("%s", long_msg.c_str());

    assert(sink->records.size() == 1);
    // Message should be truncated but not crash
    size_t msg_len = strlen(sink->records[0].message);
    assert(msg_len < 512);  // Should fit in LogRecord::message buffer
    assert(msg_len > 0);

    RemoveLogSink(sink);
    std::cout << "  PASS: test_long_message_truncation" << std::endl;
}

void test_stderr_sink_no_crash() {
    // Just verify that StderrSink::Write doesn't crash
    StderrSink sink(false);  // no color

    LogRecord rec{};
    rec.level = LogLevel::kInfo;
    rec.timestamp_us = NowMicros();
    rec.pid = GetPid();
    rec.tid = GetTid();
    strncpy(rec.module, "test", sizeof(rec.module));
    strncpy(rec.file, "test_log.cc", sizeof(rec.file));
    rec.line = 42;
    strncpy(rec.message, "StderrSink test", sizeof(rec.message));

    sink.Write(rec);  // Should not crash
    std::cout << "  PASS: test_stderr_sink_no_crash" << std::endl;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main() {
    std::cout << "=== test_logging_sys ===" << std::endl;

    test_log_level_to_string();
    test_log_record_fields();
    test_log_level_filtering();
    test_multiple_sinks();
    test_remove_sink();
    test_long_message_truncation();
    test_stderr_sink_no_crash();

    std::cout << "All log tests passed!" << std::endl;
    return 0;
}
