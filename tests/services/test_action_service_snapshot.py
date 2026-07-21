from __future__ import annotations

import threading

from rynnrcp.services.action_service import ActionService


class Publisher:
    def __init__(self) -> None:
        self.messages: list[bytes] = []

    def publish(self, value: bytes) -> None:
        self.messages.append(value)


def test_latest_actions_tracks_last_successfully_published_frame() -> None:
    service = ActionService.__new__(ActionService)
    publisher = Publisher()
    service._publishers = {
        "action-channel": {
            "publisher": publisher,
            "msg_size": 4096,
            "shared_data_buffer_size": None,
            "shared_data_slot_count": None,
        }
    }
    service._action_stores = {}
    service._latest_actions = {}
    service._latest_actions_lock = threading.RLock()
    service._latest_action_capture_enabled = False
    service.set_latest_action_capture(True)

    service._publish_action(
        "action.robot.joint_position",
        {"joint_positions": [0.1, 0.2]},
        50.0,
        "joint_position",
        output_channels={"action-channel"},
    )

    assert publisher.messages
    latest = service.latest_actions["action.robot.joint_position"]
    assert latest["value"] == {"joint_positions": [0.1, 0.2]}
    assert latest["frame_rate"] == 50.0
    assert latest["type"] == "joint_position"

    service.set_latest_action_capture(False)
    assert service.latest_actions == {}
