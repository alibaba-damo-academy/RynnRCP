# Copyright 2026 RynnRCP Authors. All rights reserved.
# Tests for core logging configuration.

import io
import logging
import os
import sys
import tempfile
import threading

# Add project root to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rynnrcp.utils.logging import RoboFormatter, ColorStreamHandler, configure_logging


class TestRoboFormatter:
    """Tests for RoboFormatter."""

    def test_format_contains_required_fields(self):
        formatter = RoboFormatter()
        record = logging.LogRecord(
            name="rynnrcp.test",
            level=logging.INFO,
            pathname="test_logging.py",
            lineno=42,
            msg="Hello %s",
            args=("world",),
            exc_info=None,
        )
        output = formatter.format(record)

        # Check format contains expected fields
        assert f"PID:{os.getpid()}" in output
        assert "TID:" in output
        assert "INFO" in output
        assert "rynnrcp.test" in output
        assert "test_logging.py:42" in output
        assert "Hello world" in output
        print("  PASS: test_format_contains_required_fields")

    def test_format_timestamp(self):
        formatter = RoboFormatter()
        record = logging.LogRecord(
            name="rynnrcp.runtime",
            level=logging.WARNING,
            pathname="foo.py",
            lineno=10,
            msg="test",
            args=(),
            exc_info=None,
        )
        output = formatter.format(record)

        # Should have timestamp-like pattern
        assert "[20" in output  # Year starts with 20xx
        assert "WARN" in output
        print("  PASS: test_format_timestamp")


class TestColorStreamHandler:
    """Tests for ColorStreamHandler."""

    def test_non_tty_no_color(self):
        """When stream is not a tty, output should not contain ANSI codes."""
        buf = io.StringIO()
        handler = ColorStreamHandler(stream=buf, colored=True)
        handler.setLevel(logging.DEBUG)

        logger = logging.getLogger("test.color")
        logger.handlers.clear()
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)

        logger.info("no color here")

        output = buf.getvalue()
        # StringIO is not a tty, so no ANSI codes should be present
        assert "\033[" not in output
        assert "no color here" in output

        logger.removeHandler(handler)
        print("  PASS: test_non_tty_no_color")


class TestConfigureLogging:
    """Tests for configure_logging."""

    def test_stderr_sink(self):
        configure_logging(level=logging.DEBUG, sinks=["stderr"], colored=False)
        root = logging.getLogger()
        assert len(root.handlers) == 1
        assert root.level == logging.DEBUG
        root.handlers.clear()
        print("  PASS: test_stderr_sink")

    def test_file_sink(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            log_path = os.path.join(tmpdir, "test.log")
            configure_logging(
                level=logging.INFO,
                sinks=["file"],
                file_path=log_path,
            )
            root = logging.getLogger()
            root.info("file test message")

            # Flush and check file
            for h in root.handlers:
                h.flush()

            assert os.path.exists(log_path)
            with open(log_path, "r") as f:
                content = f.read()
            assert "file test message" in content

            # Close handlers before temp dir cleanup (Windows file locking)
            for h in root.handlers[:]:
                h.close()
                root.removeHandler(h)
        print("  PASS: test_file_sink")

    def test_multiple_sinks(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            log_path = os.path.join(tmpdir, "multi.log")
            configure_logging(
                level=logging.DEBUG,
                sinks=["stderr", "file"],
                colored=False,
                file_path=log_path,
            )
            root = logging.getLogger()
            assert len(root.handlers) == 2

            # Close handlers before temp dir cleanup (Windows file locking)
            for h in root.handlers[:]:
                h.close()
                root.removeHandler(h)
        print("  PASS: test_multiple_sinks")

    def test_reconfigure_clears_handlers(self):
        configure_logging(level=logging.INFO, sinks=["stderr"])
        root = logging.getLogger()
        assert len(root.handlers) == 1

        configure_logging(level=logging.DEBUG, sinks=["stderr"])
        assert len(root.handlers) == 1  # Old handler removed

        root.handlers.clear()
        print("  PASS: test_reconfigure_clears_handlers")


def run_tests():
    print("=== test_logging ===")

    t1 = TestRoboFormatter()
    t1.test_format_contains_required_fields()
    t1.test_format_timestamp()

    t2 = TestColorStreamHandler()
    t2.test_non_tty_no_color()

    t3 = TestConfigureLogging()
    t3.test_stderr_sink()
    t3.test_file_sink()
    t3.test_multiple_sinks()
    t3.test_reconfigure_clears_handlers()

    print("All logging tests passed!")


if __name__ == "__main__":
    run_tests()
