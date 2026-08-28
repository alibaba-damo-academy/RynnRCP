"""P0/P1 regression tests for the logging infrastructure overhaul.

Covers: exception tracebacks in RoboFormatter, listener lifecycle across
reconfiguration, UTF-8 log files, rotation, run-correlation context, the
LogGate rate limiter, and redaction helpers.
"""

from __future__ import annotations

import io
import logging
import os
import threading
from pathlib import Path

import pytest

import rynnrcp.utils.logging as logging_utils
from rynnrcp.utils.log_gate import LogGate
from rynnrcp.utils.logging import (
    RoboFormatter,
    clear_log_context,
    configure_logging,
    get_log_context,
    resolve_log_run_id,
    set_log_context,
)
from rynnrcp.utils.redaction import REDACTED, describe_payload, redact


@pytest.fixture(autouse=True)
def clean_context():
    clear_log_context()
    yield
    clear_log_context()


def _named_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.handlers.clear()
    logger.propagate = False
    logger.setLevel(logging.DEBUG)
    return logger


# ---------------------------------------------------------------------------
# RoboFormatter: tracebacks and stack info
# ---------------------------------------------------------------------------


def test_formatter_appends_exception_traceback() -> None:
    buffer = io.StringIO()
    handler = logging.StreamHandler(buffer)
    handler.setFormatter(RoboFormatter())
    logger = _named_logger("rynnrcp.test.tb")
    logger.addHandler(handler)

    try:
        raise ValueError("kaboom")
    except ValueError:
        logger.exception("operation failed")

    output = buffer.getvalue()
    assert "operation failed" in output
    assert "Traceback (most recent call last)" in output
    assert "ValueError: kaboom" in output


def test_formatter_appends_stack_info_and_caches_exc_text() -> None:
    formatter = RoboFormatter()
    try:
        raise RuntimeError("cached")
    except RuntimeError:
        import sys

        record = logging.LogRecord(
            name="rynnrcp.test",
            level=logging.ERROR,
            pathname="x.py",
            lineno=1,
            msg="msg",
            args=(),
            exc_info=sys.exc_info(),
        )
    first = formatter.format(record)
    assert "RuntimeError: cached" in first
    # Second format reuses record.exc_text without duplicating it.
    second = formatter.format(record)
    assert second.count("RuntimeError: cached") == 1

    with_stack = logging.LogRecord(
        name="rynnrcp.test",
        level=logging.INFO,
        pathname="x.py",
        lineno=1,
        msg="msg",
        args=(),
        exc_info=None,
    )
    with_stack.stack_info = "Stack (most recent call last):\n  fake"
    assert "Stack (most recent call last)" in formatter.format(with_stack)


def test_formatter_uses_the_record_origin_process_and_thread() -> None:
    record = logging.LogRecord(
        name="rynnrcp.test",
        level=logging.INFO,
        pathname="x.py",
        lineno=1,
        msg="origin",
        args=(),
        exc_info=None,
    )
    record.process = 123
    record.thread = 456
    line = RoboFormatter().format(record)
    assert "[PID:123]" in line
    assert "[TID:456]" in line


# ---------------------------------------------------------------------------
# Run-correlation context
# ---------------------------------------------------------------------------


def test_log_context_appears_in_formatted_lines() -> None:
    set_log_context(robot_id="so101", session_id="20260728_1200_1", process="server")
    record = logging.LogRecord(
        name="rynnrcp.test",
        level=logging.INFO,
        pathname="x.py",
        lineno=1,
        msg="hello",
        args=(),
        exc_info=None,
    )
    line = RoboFormatter().format(record)
    assert "robot_id=so101" in line
    assert "session_id=20260728_1200_1" in line
    assert "process=server" in line

    set_log_context(process=None)
    assert "process" not in get_log_context()
    clear_log_context()
    assert "robot_id=" not in RoboFormatter().format(record)


def test_log_context_sanitizes_newlines_and_resolves_shared_run_id(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("RYNNRCP_RUN_ID", "shared-run")
    assert resolve_log_run_id("fallback") == "shared-run"

    set_log_context(operation="capture\nstart")
    assert get_log_context()["operation"] == "capture\\nstart"


def test_generated_run_id_is_reused_and_exported(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("RYNNRCP_RUN_ID", raising=False)
    first = resolve_log_run_id("generated-run")
    second = resolve_log_run_id("different-default")
    assert first == second == "generated-run"
    assert os.environ["RYNNRCP_RUN_ID"] == "generated-run"


# ---------------------------------------------------------------------------
# Listener lifecycle and file sinks
# ---------------------------------------------------------------------------


def test_reconfiguring_one_logger_keeps_other_file_listeners(tmp_path: Path) -> None:
    log_a = tmp_path / "a.log"
    log_b = tmp_path / "b.log"

    logger_a = configure_logging(
        sinks=["file"], file_path=str(log_a), logger_name="rynnrcp.test.a"
    )
    logger_b = configure_logging(
        sinks=["file"], file_path=str(log_b), logger_name="rynnrcp.test.b"
    )

    logger_a.info("from a before reconfigure")
    # Reconfiguring B must not stop A's async listener.
    logger_b = configure_logging(
        sinks=["file"], file_path=str(log_b), logger_name="rynnrcp.test.b"
    )
    logger_a.info("from a after reconfigure")
    logger_b.info("from b after reconfigure")

    # Closing the handlers drains their queues.
    configure_logging(sinks=[], logger_name="rynnrcp.test.a")
    configure_logging(sinks=[], logger_name="rynnrcp.test.b")

    content_a = log_a.read_text(encoding="utf-8")
    assert "from a before reconfigure" in content_a
    assert "from a after reconfigure" in content_a
    assert "from b after reconfigure" in log_b.read_text(encoding="utf-8")


def test_log_files_are_utf8(tmp_path: Path) -> None:
    log_path = tmp_path / "utf8.log"
    logger = configure_logging(
        sinks=["file"], file_path=str(log_path), logger_name="rynnrcp.test.utf8"
    )
    logger.info("机器人已连接 ✅ Δθ=0.5")
    configure_logging(sinks=[], logger_name="rynnrcp.test.utf8")

    content = log_path.read_bytes().decode("utf-8")
    assert "机器人已连接 ✅ Δθ=0.5" in content


def test_file_rotation_creates_backups(tmp_path: Path) -> None:
    log_path = tmp_path / "rotate.log"
    logger = configure_logging(
        sinks=["file"],
        file_path=str(log_path),
        file_max_bytes=2048,
        file_max_files=2,
        logger_name="rynnrcp.test.rotate",
    )
    for index in range(64):
        logger.info("rotation filler line %04d %s", index, "x" * 128)
    configure_logging(sinks=[], logger_name="rynnrcp.test.rotate")

    assert log_path.exists()
    backups = sorted(tmp_path.glob("rotate.log.*"))
    assert backups, "expected at least one rotated backup file"
    assert len(backups) <= 2


def test_exception_traceback_reaches_log_file(tmp_path: Path) -> None:
    log_path = tmp_path / "exc.log"
    logger = configure_logging(
        sinks=["file"], file_path=str(log_path), logger_name="rynnrcp.test.excfile"
    )
    try:
        raise KeyError("missing-key")
    except KeyError:
        logger.exception("lookup failed")
    configure_logging(sinks=[], logger_name="rynnrcp.test.excfile")

    content = log_path.read_text(encoding="utf-8")
    assert "lookup failed" in content
    assert "KeyError: 'missing-key'" in content


def test_async_file_records_keep_the_context_from_emit_time(tmp_path: Path) -> None:
    log_path = tmp_path / "context.log"
    logger = configure_logging(
        sinks=["file"], file_path=str(log_path), logger_name="rynnrcp.test.context"
    )
    set_log_context(run_id="run-a")
    logger.info("first")
    set_log_context(run_id="run-b")
    logger.info("second")
    configure_logging(sinks=[], logger_name="rynnrcp.test.context")

    lines = log_path.read_text(encoding="utf-8").splitlines()
    first = next(line for line in lines if line.endswith("first"))
    second = next(line for line in lines if line.endswith("second"))
    assert "run_id=run-a" in first
    assert "run_id=run-b" in second


def test_logging_environment_controls_are_case_insensitive(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    log_path = tmp_path / "disabled.log"
    monkeypatch.setenv("RYNNRCP_LOG_TO_FILE", "False")
    logger = configure_logging(
        sinks=["file"], file_path=str(log_path), logger_name="rynnrcp.test.disabled"
    )
    logger.info("not written")
    assert logger.handlers == []
    assert not log_path.exists()


def test_unavailable_file_sink_falls_back_to_stderr(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    def fail_file_handler(*_args, **_kwargs):
        raise PermissionError("read-only log directory")

    monkeypatch.setattr(logging_utils, "_async_file_handler", fail_file_handler)
    logger = configure_logging(
        level=logging.ERROR,
        sinks=["file"],
        file_path="/read-only/rynnrcp.log",
        logger_name="rynnrcp.test.file-fallback",
        colored=False,
    )
    logger.error("runtime continues")
    configure_logging(sinks=[], logger_name="rynnrcp.test.file-fallback")

    stderr = capsys.readouterr().err
    assert "FILE_SINK_UNAVAILABLE" in stderr
    assert "runtime continues" in stderr


def test_configure_logging_rejects_invalid_rotation_values(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.delenv("RYNNRCP_LOG_MAX_BYTES", raising=False)
    with pytest.raises(ValueError, match="file_max_bytes"):
        configure_logging(
            sinks=["file"],
            file_path=str(tmp_path / "invalid.log"),
            file_max_bytes=0,
        )


# ---------------------------------------------------------------------------
# LogGate rate limiting
# ---------------------------------------------------------------------------


def test_log_gate_first_failure_summary_and_recovery(
    caplog: pytest.LogCaptureFixture, monkeypatch: pytest.MonkeyPatch
) -> None:
    clock = {"now": 100.0}
    monkeypatch.setattr("rynnrcp.utils.log_gate.time.monotonic", lambda: clock["now"])

    logger = logging.getLogger("rynnrcp.test.gate")
    gate = LogGate(logger, "camera:front", interval_s=5.0)

    with caplog.at_level(logging.INFO, logger="rynnrcp.test.gate"):
        gate.failure("read failed: %s", "timeout")
        assert gate.failing is True
        # Repeats inside the window stay silent.
        clock["now"] += 1.0
        gate.failure("read failed: %s", "timeout")
        clock["now"] += 1.0
        gate.failure("read failed: %s", "timeout")
        # After the window one summary is emitted.
        clock["now"] += 5.0
        gate.failure("read failed: %s", "timeout")
        # Recovery emits duration and count.
        clock["now"] += 2.0
        gate.success()
        gate.success()  # idempotent

    messages = [record.getMessage() for record in caplog.records]
    assert len(messages) == 3
    assert "read failed: timeout" in messages[0]
    assert "still failing" in messages[1] and "4 failures" in messages[1]
    assert "recovered after" in messages[2] and "4 failures" in messages[2]
    assert gate.failing is False and gate.failure_count == 0


def test_log_gate_success_without_failure_is_silent(
    caplog: pytest.LogCaptureFixture,
) -> None:
    logger = logging.getLogger("rynnrcp.test.gate2")
    gate = LogGate(logger, "src")
    with caplog.at_level(logging.DEBUG, logger="rynnrcp.test.gate2"):
        gate.success()
    assert caplog.records == []


def test_log_gate_supports_warning_level(caplog: pytest.LogCaptureFixture) -> None:
    logger = logging.getLogger("rynnrcp.test.gate-warning")
    gate = LogGate(logger, "sim-zmq", level=logging.WARNING)
    with caplog.at_level(logging.INFO, logger=logger.name):
        gate.failure("request timed out")
    assert len(caplog.records) == 1
    assert caplog.records[0].levelno == logging.WARNING


def test_log_gate_is_thread_safe(caplog: pytest.LogCaptureFixture) -> None:
    logger = logging.getLogger("rynnrcp.test.gate-threaded")
    gate = LogGate(logger, "ipc-callback", interval_s=60.0)
    workers = [
        threading.Thread(target=gate.failure, args=("callback failed",))
        for _ in range(8)
    ]
    with caplog.at_level(logging.INFO, logger=logger.name):
        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join()

    assert gate.failure_count == 8
    assert len(caplog.records) == 1


# ---------------------------------------------------------------------------
# Redaction and payload description
# ---------------------------------------------------------------------------


def test_redact_masks_credential_like_keys() -> None:
    payload = {
        "robot_id": "so101",
        "Token": "abc123",
        "auth": {"password": "p", "Authorization": "Bearer x", "user": "u"},
        "items": [{"api_key": "k"}, "plain"],
    }
    result = redact(payload)
    assert result["robot_id"] == "so101"
    assert result["Token"] == REDACTED
    assert result["auth"] == REDACTED
    assert result["items"][0]["api_key"] == REDACTED
    assert result["items"][1] == "plain"


def test_redact_limits_depth() -> None:
    deep = {"a": {"b": {"c": {"d": {"e": {"f": {"g": 1}}}}}}}
    result = redact(deep, max_depth=3)
    assert result["a"]["b"]["c"] == "..."


def test_describe_payload_never_dumps_raw_data() -> None:
    numpy = pytest.importorskip("numpy")
    assert describe_payload(None) == "None"
    assert describe_payload(b"\x00" * 100000) == "<bytes len=100000>"
    frame = numpy.zeros((480, 640, 3), dtype=numpy.uint8)
    assert describe_payload(frame) == "<array shape=(480, 640, 3) dtype=uint8>"
    assert describe_payload("Bearer secret") == "<str len=13>"
    assert describe_payload([1.0, 2.0]) == "[1.0, 2.0]"
    assert describe_payload(list(range(1000))) == "<list len=1000>"
    long_text = "x" * 5000
    described = describe_payload(long_text)
    assert "len=5000" in described and long_text not in described
    described_dict = describe_payload({f"k{i}": i for i in range(20)})
    assert "len=20" in described_dict
