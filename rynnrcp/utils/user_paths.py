"""User-local storage paths for RynnRCP robot servers.

Package code and templates live in the repository or installed wheel. Runtime
state, user-edited configs, captures, logs, cache, and temporary files live
under the user-local RynnRCP home so normal use does not dirty source trees.
"""

from __future__ import annotations

import os
from datetime import datetime
from importlib import resources
from pathlib import Path
from typing import Any, Mapping

from rynnrcp.utils import safe_name


RYNNRCP_HOME_ENV = "RYNNRCP_HOME"


def rynnrcp_home() -> Path:
    """Return the root user-local RynnRCP directory."""
    return Path(os.environ.get(RYNNRCP_HOME_ENV, "~/.rynnrcp")).expanduser()


def robot_root(robot_id: str) -> Path:
    """Return ``~/.rynnrcp/<robot_id>``."""
    value = str(robot_id or "").strip()
    if not value:
        raise ValueError("robot_id is required")
    if safe_name(value) != value:
        raise ValueError("robot_id may only contain letters, numbers, '.', '_' and '-'")
    return rynnrcp_home() / value


def robot_root_from_config(config: Mapping[str, Any] | None) -> Path:
    """Resolve the local storage root from manifest.robot_id."""
    if not isinstance(config, Mapping):
        raise ValueError("manifest.robot_id is required to resolve local storage root")
    manifest = config.get("manifest")
    if not isinstance(manifest, Mapping):
        raise ValueError("manifest.robot_id is required to resolve local storage root")
    return robot_root(str(manifest.get("robot_id") or ""))


def app_root(app_id: str) -> Path:
    """Return ``~/.rynnrcp/apps/<app_id>``."""
    value = str(app_id or "").strip()
    if not value:
        raise ValueError("app.app_id is required")
    if safe_name(value) != value:
        raise ValueError("app.app_id may only contain letters, numbers, '.', '_' and '-'")
    return rynnrcp_home() / "apps" / value


def app_root_from_config(config: Mapping[str, Any] | None) -> Path:
    """Resolve the local storage root from rynnrcp_app_config.app.app_id."""
    if not isinstance(config, Mapping):
        raise ValueError("app.app_id is required to resolve local app storage root")
    app = config.get("app")
    app_id = config.get("app_id")
    if not app_id and isinstance(app, Mapping):
        app_id = app.get("app_id")
    return app_root(str(app_id or ""))


def local_root_from_config(config: Mapping[str, Any] | None) -> Path:
    """Resolve local storage root by config_type."""
    if not isinstance(config, Mapping):
        raise ValueError("config_type is required to resolve local storage root")
    config_type = str(config.get("config_type") or "")
    if config_type == "rynnrcp_server_config":
        return robot_root_from_config(config)
    if config_type == "rynnrcp_app_config":
        return app_root_from_config(config)
    raise ValueError(f"unsupported config_type for local storage root: {config_type!r}")


def config_dir(root: Path) -> Path:
    return root / "config"


def app_config_dir(root: Path) -> Path:
    return config_dir(root) / "apps"


def data_dir(root: Path) -> Path:
    return root / "data"


def collections_dir(root: Path, kind: str = "manual") -> Path:
    return data_dir(root) / "collections" / str(kind)


def encoded_dir(root: Path) -> Path:
    return data_dir(root) / "encoded"


def datasets_dir(root: Path) -> Path:
    return data_dir(root) / "datasets"


def packages_dir(root: Path) -> Path:
    return data_dir(root) / "packages"


def logs_dir(root: Path) -> Path:
    return root / "logs"


def new_log_session_id() -> str:
    return f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{os.getpid()}"


def log_session_dir(root: Path, session_id: str) -> Path:
    value = str(session_id or "").strip()
    if not value:
        raise ValueError("log session_id is required")
    return logs_dir(root) / safe_name(value)


def tmp_dir(root: Path) -> Path:
    return root / "tmp"


def cache_dir(root: Path) -> Path:
    return root / "cache"


def raw_captures_dir(root: Path, kind: str = "manual") -> Path:
    return data_dir(root) / "raw_captures" / str(kind)


def ensure_robot_dirs(root: Path) -> None:
    """Create the robot-local root; feature folders are created on first use."""
    root.mkdir(parents=True, exist_ok=True)


def resolve_robot_path(path: str | os.PathLike[str], root: Path) -> Path:
    """Resolve a path; relative values are interpreted under the robot root."""
    if isinstance(path, str) and path.startswith("package://"):
        rest = path[len("package://") :]
        package, sep, resource = rest.partition("/")
        if not sep or not package or not resource:
            raise ValueError(f"invalid package path uri: {path!r}")
        return Path(resources.files(package).joinpath(resource))
    value = Path(path).expanduser()
    if value.is_absolute():
        return value
    return root / value


def path_config_block(root: Path) -> dict[str, str]:
    """Return a serializable storage block for configs."""
    return {
        "root_dir": str(root),
        "config_dir": str(config_dir(root)),
        "data_dir": str(data_dir(root)),
        "collection_root": str(data_dir(root) / "collections"),
        "collection_dir": str(collections_dir(root, "manual")),
        "raw_capture_dir": str(raw_captures_dir(root, "manual")),
        "encoded_dir": str(encoded_dir(root)),
        "datasets_dir": str(datasets_dir(root)),
        "packages_dir": str(packages_dir(root)),
        "log_dir": str(logs_dir(root)),
        "tmp_dir": str(tmp_dir(root)),
        "cache_dir": str(cache_dir(root)),
    }


def log_file_from_config(config: Mapping[str, Any], filename: str, session_id: str | None = None) -> Path:
    root = local_root_from_config(config)
    if session_id:
        return log_session_dir(root, session_id) / filename
    return logs_dir(root) / filename
