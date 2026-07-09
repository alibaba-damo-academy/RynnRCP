"""RynnBot app config resolution tests."""

from __future__ import annotations

from rynnrcp_app_rynnbot.config import merge_default_config, resolve_rynnbot_config


def test_rynnbot_env_overrides_yaml_config(monkeypatch) -> None:
    monkeypatch.setenv("RYNNBOT_APP_ID", "env_rynnbot_app")
    monkeypatch.setenv("RYNNBOT_PRODUCT_KEY", "env_product")
    monkeypatch.setenv("RYNNBOT_DEVICE_NAME", "env_device")
    monkeypatch.setenv("RYNNBOT_DEVICE_SECRET", "env_secret")
    monkeypatch.setenv("RYNNBOT_HTTP_URL", "http://env.example")
    monkeypatch.setenv("RYNNBOT_IMAGE_UPLOAD_CODEC", "npy_gzip")
    monkeypatch.setenv("RYNNBOT_UDP_PORT", "19090")

    config = merge_default_config(
        {
            "app": {
                "app_id": "yaml_app",
                "product_key": "yaml_product",
                "device_name": "yaml_device",
                "device_secret": "yaml_secret",
                "http_url": "http://yaml.example",
            },
            "udp_port": 18080,
        }
    )
    resolved = resolve_rynnbot_config(config)

    assert config["app_id"] == "env_rynnbot_app"
    assert config["app"]["app_id"] == "env_rynnbot_app"
    assert resolved.product_key == "env_product"
    assert resolved.device_name == "env_device"
    assert resolved.device_secret == "env_secret"
    assert resolved.http_url == "http://env.example"
    assert resolved.image_upload_codec == "npy_gzip"
    assert resolved.udp_port == 19090


def test_rynnbot_udp_port_env_precedence(monkeypatch) -> None:
    monkeypatch.delenv("RYNNBOT_UDP_PORT", raising=False)
    monkeypatch.delenv("UDP_PORT", raising=False)
    monkeypatch.delenv("PORT", raising=False)

    config = merge_default_config(_minimal_config())
    assert resolve_rynnbot_config(config).udp_port == 7000

    monkeypatch.setenv("PORT", "8000")
    config = merge_default_config(_minimal_config())
    assert resolve_rynnbot_config(config).udp_port == 8005

    monkeypatch.setenv("UDP_PORT", "9000")
    config = merge_default_config(_minimal_config())
    assert resolve_rynnbot_config(config).udp_port == 9000

    monkeypatch.setenv("RYNNBOT_UDP_PORT", "10000")
    config = merge_default_config(_minimal_config())
    assert resolve_rynnbot_config(config).udp_port == 10000


def _minimal_config() -> dict:
    return {
        "app": {
            "app_id": "yaml_app",
            "product_key": "yaml_product",
            "device_name": "yaml_device",
            "device_secret": "yaml_secret",
            "http_url": "http://yaml.example",
        },
        "udp_port": 7000,
    }
