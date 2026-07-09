"""Config loading helpers shared by Runtime and app code."""

from __future__ import annotations

import re
from importlib import resources
from pathlib import Path
from typing import Any, Mapping

import yaml

from rynnrcp.config.validator import validate_source


_REF_PATTERN = re.compile(r"\$\{([^}]+)\}")


def read_config_uri(uri: str) -> str:
    if uri.startswith("package://"):
        rest = uri[len("package://") :]
        package, sep, resource = rest.partition("/")
        if not sep or not package or not resource:
            raise ValueError(f"invalid package config uri: {uri!r}")
        return resources.files(package).joinpath(resource).read_text(encoding="utf-8")
    return Path(uri).expanduser().read_text(encoding="utf-8")


def load_yaml_mapping_from_uri(uri: str) -> dict[str, Any]:
    raw = yaml.safe_load(read_config_uri(uri)) or {}
    if not isinstance(raw, dict):
        raise TypeError(f"{uri} must contain a YAML mapping")
    validate_source(raw)
    return raw


def load_core_config() -> dict[str, Any]:
    raw = yaml.safe_load(
        resources.files("rynnrcp.config").joinpath("core.yaml").read_text(encoding="utf-8")
    ) or {}
    if not isinstance(raw, dict):
        raise TypeError("rynnrcp/config/core.yaml must contain a YAML mapping")
    validate_source(raw)
    return raw


def load_integration_config(server_config: Mapping[str, Any]) -> dict[str, Any]:
    integration = require_mapping(server_config, "integration")
    uri = require_str(integration, "config")
    return load_yaml_mapping_from_uri(uri)


def resolve_refs(value: Any, context: Mapping[str, Any]) -> Any:
    def lookup(path: str) -> Any:
        cur: Any = context
        for part in path.split("."):
            if isinstance(cur, Mapping) and cur.get("enabled") is False:
                raise KeyError(f"Disabled config value for ${{{path}}}")
            if isinstance(cur, Mapping) and part in cur:
                cur = cur[part]
            else:
                raise KeyError(f"Missing config value for ${{{path}}}")
        if isinstance(cur, Mapping) and cur.get("enabled") is False:
            raise KeyError(f"Disabled config value for ${{{path}}}")
        return cur

    def resolve(item: Any) -> Any:
        if isinstance(item, Mapping):
            return {key: resolve(v) for key, v in item.items()}
        if isinstance(item, list):
            return [resolve(v) for v in item]
        if isinstance(item, str):
            full = _REF_PATTERN.fullmatch(item)
            if full:
                return lookup(full.group(1))
            return _REF_PATTERN.sub(lambda match: str(lookup(match.group(1))), item)
        return item

    return resolve(value)


def require_mapping(config: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = config.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f"{key} must be a dict")
    return value


def require_str(config: Mapping[str, Any], key: str) -> str:
    value = config.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{key} must be a non-empty string")
    return value


def require_number(config: Mapping[str, Any], key: str) -> float:
    value = config.get(key)
    if not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be a number")
    return float(value)
