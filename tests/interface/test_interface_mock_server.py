from __future__ import annotations

import time
from typing import Any, Callable

import pytest

from rynnrcp.interface.client import ClientInterface
from mock_runtime_support import (
    MOCK_CONFIG_NAME,
    MOCK_ACTION_KEY,
    MOCK_IMAGE_KEY,
    MOCK_SERVER_ID,
    MOCK_STATE_KEY,
    MOCK_STATE_VALUES,
    build_mock_runtime_config,
)
from rynnrcp.ipc.channel import ChannelManager
from rynnrcp.server import RynnRCPServer


pytest.importorskip("grpc")


def test_mock_server_interface_exposes_manifest_state_image_and_action() -> None:
    ChannelManager.reset()
    config = build_mock_runtime_config(
        server_id=MOCK_SERVER_ID,
        config_name=MOCK_CONFIG_NAME,
        image_width=8,
        image_height=4,
        image_fps=50.0,
        channel_transport="memory",
    )
    config["server"]["interface"] = {
        "host": "127.0.0.1",
        "port": 0,
        "local_registry": False,
        "mdns": False,
    }
    server = RynnRCPServer(config=config)
    server.start()
    conn = ClientInterface.with_defaults(local_registry=False).connect(f"127.0.0.1:{server.bound_port}")
    try:
        manifest = conn.request("get_manifest", {}, timeout_ms=1000)
        tools = conn.request("list_tools", {}, timeout_ms=1000)
        state = _wait_for_response(
            lambda: conn.request(
                "get_observations",
                {"names": [MOCK_STATE_KEY]},
                timeout_ms=1000,
            ),
            predicate=lambda payload: bool(payload.get("observations")),
        )
        image = _wait_for_response(
            lambda: conn.request(
                "get_observations",
                {"names": [MOCK_IMAGE_KEY]},
                timeout_ms=1000,
            ),
            predicate=lambda payload: bool(payload.get("observations")),
        )
        action = conn.request(
            "run_action_chunk",
            {
                "name": MOCK_ACTION_KEY,
                "frames": [{"joint_positions": [0.11, 0.22, 0.33, 0.44, 0.55, 0.66]}],
                "frame_rate": 100,
            },
            timeout_ms=1000,
        )
    finally:
        conn.close()
        server.stop()
        ChannelManager.reset()

    assert manifest.payload["robot_id"] == MOCK_SERVER_ID
    assert manifest.payload["robot_name"] == MOCK_CONFIG_NAME
    assert manifest.payload["capabilities"]["observations"] is True
    assert manifest.payload["capabilities"]["actions"] is True
    assert "get_observations" in tools.payload
    assert "run_action_chunk" in tools.payload

    assert state.payload["observations"][0]["name"] == MOCK_STATE_KEY
    assert state.payload["observations"][0]["value"]["joint_positions"] == MOCK_STATE_VALUES

    image_payload = image.payload["observations"][0]["value"]
    assert image.payload["observations"][0]["name"] == MOCK_IMAGE_KEY
    assert image_payload["width"] == 8
    assert image_payload["height"] == 4
    assert image_payload["encoding"] == "rgb8"
    assert isinstance(image_payload["image"], bytes)

    assert action.payload["accepted_frames"] == 1


def _wait_for_response(
    request: Callable[[], Any],
    *,
    predicate: Callable[[dict[str, Any]], bool] | None = None,
    timeout_s: float = 3.0,
) -> Any:
    deadline = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < deadline:
        last = request()
        payload = last.payload
        if isinstance(payload, dict):
            if predicate is None or predicate(payload):
                return last
        time.sleep(0.02)
    return last
