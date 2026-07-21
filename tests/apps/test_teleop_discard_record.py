"""Teleop recording discard regression test."""

from __future__ import annotations

import threading
from unittest.mock import patch

from rynnrcp_app_teleop.teleop_app import TeleopApp


def test_discard_record_deletes_latest_target_collection() -> None:
    app = TeleopApp.__new__(TeleopApp)
    app._record_status_lock = threading.RLock()
    app._latest_record_status = {
        "episode_dir": None,
        "collection_resource": {"resource_id": "episode-resource"},
    }
    deleted = []
    with patch.object(
        app,
        "delete_server_collection",
        side_effect=lambda side, resource_id: deleted.append((side, resource_id)) or {"success": True},
    ):
        assert app.discard_record() == {"success": True}
    assert deleted == [("target", "episode-resource")]
    assert app._latest_record_status["collection_resource"] == {}
