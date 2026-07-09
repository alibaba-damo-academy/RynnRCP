"""RynnBot app configuration helpers."""

from __future__ import annotations

import os
from dataclasses import dataclass
from importlib import resources
from typing import Any, Dict, Optional

import yaml


@dataclass
class RynnBotConfig:
    product_key: str
    device_name: str
    device_secret: str
    http_url: str
    udp_port: int
    auth_timeout_s: float
    auth_max_attempts: Optional[int]
    mqtt_connect_timeout_s: Optional[float]
    mqtt_connect_attempts: int
    device_monitor_interval_s: float
    image_upload_codec: str = "jpeg"  # "npy_gzip" | "jpeg"
    image_jpeg_quality: int = 90


def load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        value = yaml.safe_load(f) or {}
    if not isinstance(value, dict):
        raise ValueError(f"YAML config must be a mapping: {path}")
    return value


def merge_default_config(config: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    merged = _load_default_config()
    if config:
        merged.update(_app_values(config))
        merged.update(_runtime_values(config))
    _apply_env_overrides(merged)
    return merged


def resolve_rynnbot_config(config: Dict[str, Any]) -> RynnBotConfig:
    missing = [key for key in ("product_key", "device_name", "device_secret") if not config.get(key)]
    if missing:
        raise ValueError(f"Missing required RynnBot config fields: {missing}")

    return RynnBotConfig(
        product_key=str(config["product_key"]),
        device_name=str(config["device_name"]),
        device_secret=str(config["device_secret"]),
        http_url=str(config["http_url"]).rstrip("/"),
        udp_port=int(config["udp_port"]),
        auth_timeout_s=float(config["auth_timeout_s"]),
        auth_max_attempts=_optional_int(config.get("auth_max_attempts")),
        mqtt_connect_timeout_s=_optional_float(config.get("mqtt_connect_timeout_s")),
        mqtt_connect_attempts=int(config["mqtt_connect_attempts"]),
        device_monitor_interval_s=float(config["device_monitor_interval_s"]),
        image_upload_codec=str(config.get("image_upload_codec") or "jpeg"),
        image_jpeg_quality=int(config.get("image_jpeg_quality") or 90),
    )


def _load_default_config() -> Dict[str, Any]:
    path = resources.files(__package__).joinpath("default_config.yaml")
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise TypeError("RynnBot default_config.yaml must contain a YAML mapping")
    values: Dict[str, Any] = {}
    for key in ("config_type", "version"):
        if key in raw:
            values[key] = raw[key]
    app = raw.get("app")
    if isinstance(app, dict):
        values.update({key: value for key, value in app.items() if key != "name"})
        values["app"] = dict(app)
    defaults = raw.get("defaults") if raw.get("config_type") == "rynnrcp_app_config" else raw
    if not isinstance(defaults, dict):
        raise TypeError("RynnBot default_config.yaml defaults must contain a YAML mapping")
    values.update(defaults)
    return values


def _app_values(config: Dict[str, Any]) -> Dict[str, Any]:
    app = config.get("app")
    if isinstance(app, dict):
        return {key: value for key, value in app.items() if key != "name"}
    return {
        key: value
        for key, value in config.items()
        if key not in ("config_type", "version", "defaults")
    }


def _runtime_values(config: Dict[str, Any]) -> Dict[str, Any]:
    return {
        key: value
        for key, value in config.items()
        if key not in ("config_type", "version", "app", "defaults")
    }


def _apply_env_overrides(config: Dict[str, Any]) -> None:
    env_keys = {
        "RYNNBOT_APP_ID": "app_id",
        "RYNNBOT_PRODUCT_KEY": "product_key",
        "RYNNBOT_DEVICE_NAME": "device_name",
        "RYNNBOT_DEVICE_SECRET": "device_secret",
        "RYNNBOT_HTTP_URL": "http_url",
        "RYNNBOT_IMAGE_UPLOAD_CODEC": "image_upload_codec",
    }
    for env_name, config_key in env_keys.items():
        value = _env_value(env_name)
        if value is not None:
            config[config_key] = value

    udp_port = _env_value("RYNNBOT_UDP_PORT")
    if udp_port is None:
        udp_port = _env_value("UDP_PORT")
    if udp_port is None:
        base_port = _env_value("PORT")
        if base_port is not None:
            udp_port = int(base_port) + 5
    if udp_port is not None:
        config["udp_port"] = int(udp_port)

    app_id = config.get("app_id")
    if app_id:
        app = config.get("app")
        if not isinstance(app, dict):
            app = {"name": "rynnbot"}
        app["app_id"] = app_id
        config["app"] = app


def _env_value(name: str) -> Optional[str]:
    value = os.getenv(name)
    if value is None:
        return None
    value = value.strip()
    return value if value else None


def _optional_int(value: Any) -> Optional[int]:
    if value is None or value == "":
        return None
    return int(value)


def _optional_float(value: Any) -> Optional[float]:
    if value is None or value == "":
        return None
    return float(value)
