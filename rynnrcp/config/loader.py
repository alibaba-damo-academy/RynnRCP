"""YAML configuration loader."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from rynnrcp.config.validator import validate_source


def load_config(path: str) -> dict[str, Any]:
    """Load and validate one YAML configuration file."""
    config_path = Path(path).expanduser()
    if config_path.suffix.lower() not in (".yaml", ".yml"):
        raise ValueError(f"Config file must be YAML: {config_path}")
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    if not isinstance(data, dict):
        raise ValueError(f"Config must be a dict, got {type(data).__name__}")

    validate_source(data)
    return data
