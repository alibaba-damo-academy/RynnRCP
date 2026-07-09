"""Tests for small utility helpers used across runtime and applications."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from rynnrcp.utils.imports import import_colon_object, import_object
from rynnrcp.utils import safe_name
from rynnrcp.utils import coerce_timestamp
from rynnrcp.utils.user_paths import ensure_robot_dirs, path_config_block, robot_root


def test_safe_name_normalizes_user_supplied_names() -> None:
    assert safe_name(" demo/run 01 ") == "demo_run_01"
    assert safe_name("camera:left@front") == "camera_left_front"
    assert safe_name("***") == "stream"


def test_coerce_timestamp_accepts_finite_numeric_values() -> None:
    assert coerce_timestamp("1.25", context="sample") == 1.25
    assert coerce_timestamp(2, context="sample") == 2.0


@pytest.mark.parametrize("value", [None, "", "abc", math.inf, math.nan])
def test_coerce_timestamp_rejects_invalid_values(value: object) -> None:
    with pytest.raises(ValueError, match="invalid timestamp for sample"):
        coerce_timestamp(value, context="sample")


def test_import_helpers_resolve_dotted_and_colon_paths() -> None:
    assert import_object("pathlib.Path") is Path
    cwd = import_colon_object("pathlib:Path.cwd")
    assert callable(cwd)
    assert cwd.__name__ == "cwd"

    with pytest.raises(ValueError):
        import_object("Path")
    with pytest.raises(ValueError):
        import_colon_object("pathlib.Path")


def test_user_robot_paths_respect_rynnrcp_home(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setenv("RYNNRCP_HOME", str(tmp_path / "home"))

    root = robot_root("so101_follower")
    ensure_robot_dirs(root)
    storage = path_config_block(root)

    assert root == tmp_path / "home" / "so101_follower"
    assert Path(storage["root_dir"]) == root
    assert Path(storage["raw_capture_dir"]) == root / "data" / "raw_captures" / "manual"
    assert root.is_dir()
    assert not (root / "config" / "apps").exists()
    assert not (root / "logs" / "apps").exists()
