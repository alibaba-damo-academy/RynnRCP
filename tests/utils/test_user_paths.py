"""Tests for user-local storage path helpers."""

from __future__ import annotations

from pathlib import Path

import pytest

from rynnrcp.utils.user_paths import (
    app_root,
    app_root_from_config,
    cache_dir,
    collections_dir,
    local_root_from_config,
    log_file_from_config,
    log_session_dir,
    logs_dir,
    new_log_session_id,
    path_config_block,
    raw_captures_dir,
    resolve_robot_path,
    robot_root,
    robot_root_from_config,
    rynnrcp_home,
    tmp_dir,
)


@pytest.fixture(autouse=True)
def rynnrcp_home_env(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> Path:
    home = tmp_path / "rynnrcp_home"
    monkeypatch.setenv("RYNNRCP_HOME", str(home))
    return home


def test_rynnrcp_home_honors_env(rynnrcp_home_env: Path) -> None:
    assert rynnrcp_home() == rynnrcp_home_env


def test_robot_root_validation(rynnrcp_home_env: Path) -> None:
    assert robot_root("so101") == rynnrcp_home_env / "so101"
    with pytest.raises(ValueError, match="robot_id is required"):
        robot_root("  ")
    with pytest.raises(ValueError, match="may only contain"):
        robot_root("bad/../id")


def test_robot_root_from_config(rynnrcp_home_env: Path) -> None:
    config = {"manifest": {"robot_id": "so101"}}
    assert robot_root_from_config(config) == rynnrcp_home_env / "so101"
    with pytest.raises(ValueError, match="manifest.robot_id is required"):
        robot_root_from_config(None)
    with pytest.raises(ValueError, match="manifest.robot_id is required"):
        robot_root_from_config({"manifest": "oops"})


def test_app_root_validation(rynnrcp_home_env: Path) -> None:
    assert app_root("teleop") == rynnrcp_home_env / "apps" / "teleop"
    with pytest.raises(ValueError, match="app.app_id is required"):
        app_root("")
    with pytest.raises(ValueError, match="may only contain"):
        app_root("a b")


def test_app_root_from_config_prefers_top_level_app_id(rynnrcp_home_env: Path) -> None:
    nested = app_root_from_config({"app": {"app_id": "nested"}})
    assert nested == rynnrcp_home_env / "apps" / "nested"
    top = app_root_from_config({"app_id": "top", "app": {"app_id": "nested"}})
    assert top == rynnrcp_home_env / "apps" / "top"
    with pytest.raises(ValueError, match="app.app_id is required"):
        app_root_from_config(None)


def test_local_root_from_config_dispatches_by_config_type(rynnrcp_home_env: Path) -> None:
    server = {"config_type": "rynnrcp_server_config", "manifest": {"robot_id": "r1"}}
    assert local_root_from_config(server) == rynnrcp_home_env / "r1"

    app = {"config_type": "rynnrcp_app_config", "app": {"app_id": "a1"}}
    assert local_root_from_config(app) == rynnrcp_home_env / "apps" / "a1"

    with pytest.raises(ValueError, match="config_type is required"):
        local_root_from_config(None)
    with pytest.raises(ValueError, match="unsupported config_type"):
        local_root_from_config({"config_type": "other"})


def test_directory_layout_helpers(rynnrcp_home_env: Path) -> None:
    root = rynnrcp_home_env / "r"
    assert collections_dir(root, "auto") == root / "data" / "collections" / "auto"
    assert raw_captures_dir(root) == root / "data" / "raw_captures" / "manual"
    assert tmp_dir(root) == root / "tmp"
    assert cache_dir(root) == root / "cache"

    block = path_config_block(root)
    assert block["root_dir"] == str(root)
    assert block["log_dir"] == str(logs_dir(root))
    assert set(block) == {
        "root_dir",
        "config_dir",
        "data_dir",
        "collection_root",
        "collection_dir",
        "raw_capture_dir",
        "encoded_dir",
        "datasets_dir",
        "packages_dir",
        "log_dir",
        "tmp_dir",
        "cache_dir",
    }


def test_log_session_helpers(rynnrcp_home_env: Path) -> None:
    session_id = new_log_session_id()
    assert "_" in session_id

    root = rynnrcp_home_env / "r"
    assert log_session_dir(root, "abc") == root / "logs" / "abc"
    with pytest.raises(ValueError, match="session_id is required"):
        log_session_dir(root, "  ")

    config = {"config_type": "rynnrcp_server_config", "manifest": {"robot_id": "r"}}
    with_session = log_file_from_config(config, "server.log", session_id="s1")
    assert with_session == root / "logs" / "s1" / "server.log"
    without_session = log_file_from_config(config, "server.log")
    assert without_session == root / "logs" / "server.log"


def test_resolve_robot_path_variants(rynnrcp_home_env: Path, tmp_path: Path) -> None:
    root = rynnrcp_home_env / "r"
    absolute = tmp_path / "abs.yaml"
    assert resolve_robot_path(str(absolute), root) == absolute
    assert resolve_robot_path("relative/x.yaml", root) == root / "relative" / "x.yaml"

    package_path = resolve_robot_path("package://rynnrcp/config/core.yaml", root)
    assert package_path.name == "core.yaml"
    assert package_path.exists()

    with pytest.raises(ValueError, match="invalid package path uri"):
        resolve_robot_path("package://only-package", root)
