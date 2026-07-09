"""Mock Runtime config and components for Interface integration testing."""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Any


MOCK_CONFIG_NAME = "mock_so101"
MOCK_SERVER_ID = "mock-runtime"
MOCK_STATE_KEY = "observation.robot.joint_state"
MOCK_IMAGE_KEY = "observation.mock_camera.image"
MOCK_ACTION_KEY = "action.robot.joint_position"
MOCK_STATE_VALUES = [1.0, 2.0, 3.0, 4.0, 5.0, 0.25]


class MockEmbodiment:
    """Small in-process embodiment used to exercise state and action paths."""

    def __init__(self, values: list[float] | None = None) -> None:
        self._default_values = list(values or MOCK_STATE_VALUES)
        self._last_action: list[float] | None = None
        self._action_count = 0
        self._running = False
        self._lock = threading.Lock()

    def start(self) -> None:
        self._running = True

    def stop(self) -> None:
        self._running = False

    def read_state(self) -> dict[str, list[float]]:
        with self._lock:
            if self._last_action is not None:
                return {"joint_positions": list(self._last_action)}
            return {"joint_positions": list(self._default_values)}

    def write_action(self, value: Any) -> None:
        if not isinstance(value, dict):
            raise TypeError("action value must be a dict")
        positions = value.get("joint_positions")
        if not isinstance(positions, (list, tuple)):
            raise TypeError("action value requires joint_positions")
        with self._lock:
            self._last_action = [float(value) for value in positions]
            self._action_count += 1

    def action_count(self) -> int:
        with self._lock:
            return self._action_count


class MockPortCamera:
    """Deterministic RGB camera source for image transport smoke tests."""

    def __init__(self, width: int = 64, height: int = 48, fps: float = 30.0) -> None:
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self._frame_index = 0

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def read(self) -> tuple[bool, int, int, str, bytes]:
        size = max(0, self.width * self.height * 3)
        offset = self._frame_index % 256
        self._frame_index += 1
        payload = bytes((offset + index) % 256 for index in range(size))
        return True, self.width, self.height, "rgb8", payload


def build_mock_runtime_config(
    *,
    server_id: str = MOCK_SERVER_ID,
    config_name: str = MOCK_CONFIG_NAME,
    state_values: list[float] | None = None,
    image_width: int = 64,
    image_height: int = 48,
    image_fps: float = 30.0,
    channel_transport: str = "memory",
    include_action: bool = True,
) -> dict[str, Any]:
    """Build a no-hardware server config that still uses real Runtime services."""

    if not include_action:
        raise ValueError("mock runtime integration currently always exposes action")
    integration_path = Path(__file__).with_name("mock_runtime_integration.yaml")
    return {
        "config_type": "rynnrcp_server_config",
        "version": 1,
        "manifest": {
            "robot_id": server_id,
            "robot_name": config_name,
            "capabilities": {
                "observations": True,
                "actions": True,
                "health": True,
                "resources": True,
                "data_collection": True,
                "policy_service": False,
            },
        },
        "server": {"interface": {}, "metadata": {"mock": True}},
        "integration": {
            "config": str(integration_path),
        },
        "runtime": {
            "runner_mode": "thread",
            "debug_status_enabled": True,
        },
        "components": {
            "robot": {
                "enabled": True,
                "state_values": list(state_values or MOCK_STATE_VALUES),
                "channel_transport": channel_transport,
                "sys_path": [str(Path(__file__).parent)],
            },
            "mock_camera": {
                "enabled": True,
                "image_width": int(image_width),
                "image_height": int(image_height),
                "image_fps": float(image_fps),
                "channel_transport": channel_transport,
                "sys_path": [str(Path(__file__).parent)],
            }
        },
    }
