"""RynnBot master-source packet/auth tests."""

from __future__ import annotations

import threading
from types import SimpleNamespace

import numpy as np

from rynnrcp_app_rynnbot.auth import RynnAuthClient
from rynnrcp_app_rynnbot.proto_codec import RynnProtoCodec
from rynnrcp_app_rynnbot.rynnbot_app import (
    OccupyType,
    RynnBotApp,
    _action_frames,
    _action_message_summary,
    _build_joint_position_action,
    _mqtt_payload_summary,
)


def test_master_source_action_matches_existing_action_parser() -> None:
    proto = RynnProtoCodec()
    multi = _build_joint_position_action(
        proto,
        [0.1, 0.2, 0.3],
        action_name="action",
        action_rate=30,
    )
    packet = proto.build_data_packet(proto.PackageType.ACTION_DATA, multi, id=7)

    pkg_type, parsed = proto.parse_inner_payload(packet)
    action = parsed.action_list[0]
    data = np.frombuffer(action.action_data.data, dtype=np.float32).reshape(list(action.action_data.shape))

    assert pkg_type == proto.PackageType.ACTION_DATA
    assert action.name == "action"
    assert action.action_rate == 30
    assert list(action.action_data.shape) == [1, 3]
    frames = _action_frames(data)
    assert len(frames) == 1
    assert np.allclose(frames[0]["joint_positions"], [0.1, 0.2, 0.3])


def test_action_message_summary_reports_cloud_chunk_shape_rate_and_frames() -> None:
    proto = RynnProtoCodec()
    multi = proto.MultiAction()
    action = multi.action_list.add()
    action.action_data.shape.extend([30, 6])
    action.action_rate = 60

    assert _action_message_summary(multi) == {
        "actions": 1,
        "frames": 30,
        "rates": [60],
        "shapes": ["30x6"],
    }


def test_mqtt_payload_summary_keeps_cloud_trigger_timing_and_rate_fields() -> None:
    summary = _mqtt_payload_summary(
        '{"id":"r1","method":"tele_data_coll:start_round",'
        '"params":{"$TIME":1234,"fps":30,"rate":60,"role":"controller"}}'
    )

    assert summary["id"] == "r1"
    assert summary["method"] == "tele_data_coll:start_round"
    assert summary["params"] == {"$TIME": 1234, "role": "controller", "fps": 30, "rate": 60}


def test_action_flow_diagnostics_count_bounded_executor_drop() -> None:
    proto = RynnProtoCodec()
    multi = _build_joint_position_action(proto, [0.1, 0.2, 0.3], action_name="action", action_rate=30)
    packet = proto.build_data_packet(proto.PackageType.ACTION_DATA, multi, id=9)
    app = object.__new__(RynnBotApp)
    app._flow_diagnostic_interval_s = 999.0
    app._ws_action_flow_lock = threading.Lock()
    app._ws_action_flow = {}

    app._record_ws_action_dispatch(multi, packet, accepted=False)

    assert app._ws_action_flow["packets"] == 1
    assert app._ws_action_flow["frames"] == 1
    assert app._ws_action_flow["dropped"] == 1
    assert app._ws_action_flow["dropped_total"] == 1


def test_follower_rejects_cloud_action_while_not_occupied() -> None:
    proto = RynnProtoCodec()
    multi = _build_joint_position_action(proto, [0.1, 0.2, 0.3], action_name="action", action_rate=30)
    packet = proto.build_data_packet(proto.PackageType.ACTION_DATA, multi, id=10)
    finishes = []
    app = object.__new__(RynnBotApp)
    app._proto = proto
    app._occupied_lock = threading.Lock()
    app._occupied = False
    app._occupy_type = OccupyType.UNKNOWN
    app._send_action_finish = lambda **kwargs: finishes.append(kwargs)

    app._dispatch_ws_packet(proto.PackageType.ACTION_DATA, multi, packet)

    assert finishes[0]["code"] == -1
    assert finishes[0]["execute_steps"] == 0
    assert finishes[0]["expect_steps"] == 1
    assert "not occupied" in finishes[0]["error_msg"]


def test_follower_rechecks_occupancy_before_action_execution() -> None:
    finishes = []

    class Client:
        def run_action_chunk(self, *args, **kwargs):
            raise AssertionError("unoccupied action must not reach the Server")

    app = object.__new__(RynnBotApp)
    app._server_client = Client()
    app._occupied_lock = threading.Lock()
    app._occupied = False
    app._occupy_type = OccupyType.UNKNOWN
    app._send_action_finish = lambda **kwargs: finishes.append(kwargs)

    app._execute_action(object(), packet=None)

    assert finishes[0]["code"] == -1
    assert "occupation ended" in finishes[0]["error_msg"]


def test_release_stops_server_action() -> None:
    class Client:
        def __init__(self) -> None:
            self.reasons = []

        def stop_action(self, reason=None):
            self.reasons.append(reason)
            return SimpleNamespace(ok=True, message="")

    client = Client()
    app = object.__new__(RynnBotApp)
    app._server_client = client
    app._manifest = None

    app._stop_cloud_action_on_release()

    assert client.reasons == ["release_device"]


def test_release_executes_manifest_stop_motion_action() -> None:
    class Client:
        def __init__(self) -> None:
            self.actions = []

        def stop_action(self, reason=None):
            return SimpleNamespace(ok=True, message="")

        def run_action_chunk(self, name, frames, frame_rate):
            self.actions.append((name, frames, frame_rate))
            return SimpleNamespace(ok=True, message="")

    client = Client()
    app = object.__new__(RynnBotApp)
    app._server_client = client
    app._manifest = SimpleNamespace(
        actions=[
            {"name": "action.robot.joint_position", "type": "joint_position"},
            {"name": "action.robot.stop_motion", "type": "prearranged"},
        ]
    )

    app._stop_cloud_action_on_release()

    assert client.actions == [("action.robot.stop_motion", [{}], 1)]


def test_master_source_acquire_activates_configured_rate(monkeypatch) -> None:
    class FakeThread:
        def __init__(self, **kwargs) -> None:
            self.kwargs = kwargs
            self.started = False

        def start(self) -> None:
            self.started = True

    monkeypatch.setattr("rynnrcp_app_rynnbot.rynnbot_app.threading.Thread", FakeThread)

    app = object.__new__(RynnBotApp)
    app._master_source_lock = threading.Lock()
    app._master_source_thread = None
    app._master_source_stop = threading.Event()
    app._master_source_seq = 99
    app._master_source_session = {}
    app._master_source_recording = False
    app._master_source_recording_fps = None
    app._master_source_recording_rate_source = "idle_config"
    app._master_source_active_hz = 30.0
    app._master_source_idle_hz = 5.0
    app._master_source_session_from_params = lambda params: {
        "taskId": "task-1",
        "subTaskId": "subtask-1",
        "occupancyId": "occupancy-1",
    }
    app._validate_master_source_manifest = lambda: None

    app._start_master_source_session({"role": "controller"})

    assert app._master_source_recording is True
    assert app._master_source_recording_fps == 30.0
    assert app._master_source_recording_rate_source == "controller.acquire_device"
    assert app._master_source_rate_hz() == 30.0
    assert app._master_source_seq == 0
    assert app._master_source_thread.started is True


def test_master_source_round_stop_does_not_change_active_rate() -> None:
    sent = []
    app = object.__new__(RynnBotApp)
    app._master_source_lock = threading.Lock()
    app._master_source_recording = True
    app._master_source_recording_fps = 30.0
    app._master_source_recording_rate_source = "controller.acquire_device"
    app._master_source_active_hz = 30.0
    app._parse_json_or_reply_error = lambda payload, topic, context: {"id": "round-stop", "params": {}}
    app._make_base_response = lambda request: {"id": request["id"], "success": True}
    app._mqtt_send = lambda topic, payload: sent.append((topic, payload))

    app._handle_master_source_round_stop(
        "sys/pk/dn/rrpc/request/0/tele_data_coll:stop_round",
        '{"id":"round-stop","params":{}}',
    )

    assert app._master_source_recording is True
    assert app._master_source_rate_hz() == 30.0
    assert sent[0][0] == "sys/pk/dn/rrpc/response/0/tele_data_coll:stop_round"


def test_auth_preserves_ws_query_and_posts_master_source_context(monkeypatch) -> None:
    posts = []

    class Response:
        status_code = 200

        def json(self):
            return {
                "success": True,
                "data": {
                    "uri": "wss://example.com/tunnel/1/master-arm-source?subTaskId=s1&x-specified-pod=10.0.0.1",
                    "token": "token",
                    "expire": 60,
                    "relayAddr": "10.0.0.1",
                },
            }

    def fake_post(url, json, timeout):
        posts.append((url, json, timeout))
        return Response()

    monkeypatch.setattr("rynnrcp_app_rynnbot.auth.requests.post", fake_post)
    auth = RynnAuthClient("https://robot.example", "pk", "dn", "secret", max_attempts=1)

    ws_cfg = auth.get_ws_config("/connect/webSocket")
    master_cfg = auth.get_master_source_ws_config(
        "/connect/webSocket/masterSource",
        task_id="t1",
        sub_task_id="s1",
    )

    assert ws_cfg["websocket_path"] == "/tunnel/1/master-arm-source?subTaskId=s1&x-specified-pod=10.0.0.1"
    assert master_cfg["uri"].endswith("x-specified-pod=10.0.0.1")
    assert posts[1][1]["taskId"] == "t1"
    assert posts[1][1]["subTaskId"] == "s1"
