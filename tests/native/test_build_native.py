"""Tests for the native extension build CLI (subprocess calls stubbed)."""

from __future__ import annotations

import sys
import sysconfig
import types
from pathlib import Path

import pytest

import rynnrcp.native.build_native as build_native


@pytest.fixture(autouse=True)
def fake_pybind11(monkeypatch: pytest.MonkeyPatch) -> None:
    module = types.ModuleType("pybind11")
    module.get_cmake_dir = lambda: "/fake/pybind11/cmake"
    monkeypatch.setitem(sys.modules, "pybind11", module)


@pytest.fixture
def recorded_runs(monkeypatch: pytest.MonkeyPatch) -> list[list[str]]:
    calls: list[list[str]] = []
    monkeypatch.setattr(build_native, "_run", lambda cmd, *, cwd: calls.append(cmd))
    return calls


def _ext_suffix() -> str:
    return sysconfig.get_config_var("EXT_SUFFIX") or ".so"


def _prepare_built_extension(build_dir: Path) -> Path:
    build_dir.mkdir(parents=True, exist_ok=True)
    built = build_dir / f"rynnrcp_core_native{_ext_suffix()}"
    built.write_bytes(b"binary")
    return built


def test_main_no_copy_reports_extension(
    tmp_path: Path, recorded_runs: list[list[str]], capsys: pytest.CaptureFixture
) -> None:
    build_dir = tmp_path / "build"
    built = _prepare_built_extension(build_dir)

    exit_code = build_native.main(["--build-dir", str(build_dir), "--no-copy"])
    assert exit_code == 0
    assert len(recorded_runs) == 2
    assert recorded_runs[0][0] == "cmake" and "-S" in recorded_runs[0]
    assert "--build" in recorded_runs[1]
    assert f"Built native extension: {built}" in capsys.readouterr().out


def test_main_copies_extension_and_removes_stale(
    tmp_path: Path, recorded_runs: list[list[str]]
) -> None:
    build_dir = tmp_path / "build"
    built = _prepare_built_extension(build_dir)
    target_dir = tmp_path / "target"
    target_dir.mkdir()
    stale = target_dir / "rynnrcp_core_native.stale.so"
    stale.write_bytes(b"old")

    exit_code = build_native.main(
        ["--build-dir", str(build_dir), "--target-dir", str(target_dir), "--debug"]
    )
    assert exit_code == 0
    assert (target_dir / built.name).read_bytes() == b"binary"
    assert not stale.exists()
    configure_cmd = recorded_runs[0]
    assert "-DCMAKE_BUILD_TYPE=Debug" in configure_cmd


def test_main_requires_pybind11(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, recorded_runs: list[list[str]]
) -> None:
    monkeypatch.setitem(sys.modules, "pybind11", None)
    with pytest.raises(SystemExit, match="pybind11 is required"):
        build_native.main(["--build-dir", str(tmp_path), "--no-copy"])


def test_find_extension_prefers_current_abi(tmp_path: Path) -> None:
    other = tmp_path / "rynnrcp_core_native.other.so"
    other.write_bytes(b"")
    expected = tmp_path / f"rynnrcp_core_native{_ext_suffix()}"
    expected.write_bytes(b"")
    assert build_native._find_extension(tmp_path) == expected


def test_find_extension_errors_when_missing(tmp_path: Path) -> None:
    with pytest.raises(SystemExit, match="Could not find built"):
        build_native._find_extension(tmp_path)
