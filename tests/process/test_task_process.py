"""Tests for short-lived child process task execution."""

from __future__ import annotations

import pytest

from rynnrcp.process.task_process import ProcessTaskError, run_python_function_task


def test_runs_importable_function_and_returns_result() -> None:
    assert run_python_function_task("operator:add", args=[2, 3], timeout_s=30.0) == 5


def test_passes_keyword_arguments() -> None:
    result = run_python_function_task(
        "builtins:sorted",
        args=[[3, 1, 2]],
        kwargs={"reverse": True},
        timeout_s=30.0,
    )
    assert result == [3, 2, 1]


def test_child_exception_surfaces_with_traceback() -> None:
    with pytest.raises(ProcessTaskError, match="ZeroDivisionError"):
        run_python_function_task("operator:truediv", args=[1, 0], timeout_s=30.0)


def test_bad_import_path_fails() -> None:
    with pytest.raises(ProcessTaskError):
        run_python_function_task("definitely_missing_module:fn", timeout_s=30.0)


def test_timeout_terminates_child() -> None:
    with pytest.raises(ProcessTaskError, match="timed out"):
        run_python_function_task("time:sleep", args=[30], timeout_s=0.5)
