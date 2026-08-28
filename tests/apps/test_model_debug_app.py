from __future__ import annotations

import io
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from rynnrcp.interface.codec import InterfaceResponse
from rynnrcp_app_rynnbot import model_debug_app as module
from rynnrcp_app_rynnbot.proto_codec import RynnProtoCodec


def _response(payload: object) -> InterfaceResponse:
    return InterfaceResponse(request_id="test", payload=payload)


class FakeClient:
    def __init__(self) -> None:
        self.executed: dict[str, object] | None = None
        self.executions: list[dict[str, object]] = []
        self.stop_reasons: list[str | None] = []
        self.requested_names = None

    def get_observations(self, names, timeout_ms=1000):
        self.requested_names = names
        return _response(
            {
                "observations": [
                    {
                        "name": "observation.robot.joint_state",
                        "timestamp": 1.0,
                        "value": {"joint_positions": [0.1, 0.2]},
                    },
                    {
                        "name": "observation.front.image",
                        "timestamp": 1.0,
                        "value": {
                            "width": 2,
                            "height": 1,
                            "encoding": "jpg",
                            "image": b"jpeg",
                        },
                    },
                ]
            }
        )

    def run_action_chunk(self, name, frames, frame_rate, timeout_ms=1000):
        self.executed = {
            "name": name,
            "frames": frames,
            "frame_rate": frame_rate,
        }
        self.executions.append(self.executed)
        return _response({"accepted_frames": len(frames)})

    def stop_action(self, reason=None, timeout_ms=1000):
        self.stop_reasons.append(reason)
        return _response({"stopped": True})


def _debug_app() -> module.ModelDebugApp:
    app = module.ModelDebugApp("server.yaml")
    app.client = FakeClient()
    app.manifest = SimpleNamespace(
        robot_id="so101",
        robot_name="SO101",
        observations=[
            {"name": "observation.robot.joint_state", "type": "joint_state"},
            {"name": "observation.front.image", "type": "image"},
        ],
        actions=[
            {"name": "action.robot.joint_position", "type": "joint_position"},
            {"name": "action.robot.home", "type": "prearranged"},
        ],
    )
    return app


def test_protobuf_is_the_default_request_protocol() -> None:
    protobuf_option = '<option value="protobuf">Protobuf（RynnBot）</option>'
    json_option = '<option value="json">JSON</option>'

    assert module.HTML.index(protobuf_option) < module.HTML.index(json_option)
    assert 'id="totalSteps"' in module.HTML
    assert 'id="startInference"' in module.HTML
    assert 'id="stopInference"' in module.HTML
    assert module.HTML.index('id="goHome"') > module.HTML.index('id="stopInference"')


def test_page_uses_three_step_workflow() -> None:
    connect = module.HTML.index("1. 连接机器人")
    configure = module.HTML.index("2. 配置推理")
    run = module.HTML.index("3. 开始推理并查看结果")

    assert connect < configure < run
    assert "4. 运行进度" not in module.HTML
    assert module.HTML.index("Observation 状态 · 最近 120 step") > run
    assert module.HTML.index("Action 历史 · 最近 120 step") > run
    assert "const actionHistory = new Map();" in module.HTML
    assert "actionHistory.get(label)" in module.HTML


def test_infer_sends_current_observations_and_normalizes_action(monkeypatch) -> None:
    app = _debug_app()
    captured = {}

    def fake_request(config, payload):
        captured.update(payload)
        return {"action": [[0.2, 0.3], [0.3, 0.4]]}

    monkeypatch.setattr(module, "_request_model", fake_request)
    result = app.infer(
        {
            "url": "http://model.test/infer",
            "prompt": "pick",
            "observation_names": [
                "observation.robot.joint_state",
                "observation.front.image",
            ],
            "action_name": "action.robot.joint_position",
            "frame_rate": 30,
        }
    )

    assert captured["prompt"] == "pick"
    assert captured["observations"]["observation.robot.joint_state"] == {
        "joint_positions": [0.1, 0.2]
    }
    assert captured["observations"]["observation.front.image"]["image"] == {
        "encoding": "base64",
        "data": "anBlZw==",
    }
    assert app.client.requested_names == [
        "observation.robot.joint_state",
        "observation.front.image",
    ]
    assert result["action"] == {
        "name": "action.robot.joint_position",
        "frames": [
            {"joint_positions": [0.2, 0.3]},
            {"joint_positions": [0.3, 0.4]},
        ],
        "frame_rate": 30,
    }


def test_execute_action_uses_selected_rcp_action() -> None:
    app = _debug_app()
    result = app.execute_action(
        "action.robot.joint_position",
        [{"joint_positions": [0.2, 0.3]}],
        30,
    )

    assert result == {"accepted_frames": 1}
    assert app.client.executed == {
        "name": "action.robot.joint_position",
        "frames": [{"joint_positions": [0.2, 0.3]}],
        "frame_rate": 30,
    }


def test_server_info_exposes_home_action_separately() -> None:
    app = _debug_app()

    result = app.server_status()

    assert result["actions"] == ["action.robot.joint_position"]
    assert result["home_action"] == "action.robot.home"


def test_go_home_stops_current_action_then_runs_home() -> None:
    app = _debug_app()

    result = app.go_home()

    assert result["action_name"] == "action.robot.home"
    assert app.client.stop_reasons == ["model_debug_go_home"]
    assert app.client.executed == {
        "name": "action.robot.home",
        "frames": [{}],
        "frame_rate": 1,
    }


def test_home_action_recognizes_go_home_name() -> None:
    manifest = SimpleNamespace(
        actions=[{"name": "action.robot.go_home", "type": "prearranged"}]
    )

    assert module.ModelDebugApp._home_action_name(manifest) == "action.robot.go_home"


def test_inference_run_repeats_and_trims_final_action_chunk(monkeypatch) -> None:
    app = _debug_app()

    def fake_infer(_config):
        return {
            "action": {
                "name": "action.robot.joint_position",
                "frames": [
                    {"joint_positions": [0.1, 0.2]},
                    {"joint_positions": [0.2, 0.3]},
                    {"joint_positions": [0.3, 0.4]},
                ],
                "frame_rate": 30,
            },
            "elapsed_ms": 12.5,
        }

    monkeypatch.setattr(app, "infer", fake_infer)
    app.start_inference(
        {
            "total_steps": 5,
            "observation_names": ["observation.robot.joint_state"],
            "action_name": "action.robot.joint_position",
        }
    )
    app.run_thread.join(timeout=1)
    status = app.inference_status()

    assert status["running"] is False
    assert status["phase"] == "completed"
    assert status["completed_steps"] == 5
    assert status["round"] == 2
    assert [len(item["frames"]) for item in app.client.executions] == [3, 2]


def test_stop_during_inference_prevents_action_execution(monkeypatch) -> None:
    app = _debug_app()

    def fake_infer(_config):
        app.run_stop_event.set()
        return {
            "action": {
                "name": "action.robot.joint_position",
                "frames": [{"joint_positions": [0.1, 0.2]}],
                "frame_rate": 30,
            },
            "elapsed_ms": 10,
        }

    monkeypatch.setattr(app, "infer", fake_infer)
    app.start_inference(
        {
            "total_steps": 20,
            "observation_names": ["observation.robot.joint_state"],
            "action_name": "action.robot.joint_position",
        }
    )
    app.run_thread.join(timeout=1)
    status = app.inference_status()

    assert status["phase"] == "stopped"
    assert status["completed_steps"] == 0
    assert app.client.executions == []


def test_stop_inference_requests_server_action_stop(monkeypatch) -> None:
    app = _debug_app()
    entered = threading.Event()
    release = threading.Event()

    def fake_infer(_config):
        entered.set()
        release.wait(timeout=1)
        return {
            "action": {
                "name": "action.robot.joint_position",
                "frames": [{"joint_positions": [0.1, 0.2]}],
                "frame_rate": 30,
            },
            "elapsed_ms": 10,
        }

    monkeypatch.setattr(app, "infer", fake_infer)
    app.start_inference(
        {
            "total_steps": 20,
            "observation_names": ["observation.robot.joint_state"],
            "action_name": "action.robot.joint_position",
        }
    )
    assert entered.wait(timeout=1)

    status = app.stop_inference()
    release.set()
    app.run_thread.join(timeout=1)

    assert status["stopping"] is True
    assert app.client.stop_reasons == ["model_debug_stop"]
    assert app.inference_status()["phase"] == "stopped"


def test_inference_error_reports_round_stage_progress_and_cause(
    monkeypatch,
) -> None:
    app = _debug_app()

    def fake_infer(_config):
        raise RuntimeError(
            "请求模型服务失败：无法连接 http://model.test/protobuf：Connection refused"
        )

    monkeypatch.setattr(app, "infer", fake_infer)
    app.start_inference(
        {
            "total_steps": 40,
            "observation_names": ["observation.robot.joint_state"],
            "action_name": "action.robot.joint_position",
        }
    )
    app.run_thread.join(timeout=1)
    status = app.inference_status()

    assert status["phase"] == "error"
    assert status["running"] is False
    assert "第 1 轮推理失败" in status["error"]
    assert "已执行 0/40 step" in status["error"]
    assert "Connection refused" in status["error"]


def test_normalize_action_rejects_invalid_values() -> None:
    with pytest.raises(ValueError, match="无效数值"):
        module._normalize_action(
            {"action": [[0.1, float("nan")]]},
            action_name="action.robot.joint_position",
            frame_rate=30,
        )


def test_infer_requires_selected_observation() -> None:
    app = _debug_app()

    with pytest.raises(ValueError, match="选择推理输入"):
        app.infer(
            {
                "url": "http://model.test/infer",
                "prompt": "pick",
                "observation_names": [],
                "action_name": "action.robot.joint_position",
                "frame_rate": 30,
            }
        )


def test_infer_payload_only_contains_selected_observations(monkeypatch) -> None:
    app = _debug_app()
    captured = {}

    def fake_request(config, payload):
        captured.update(payload)
        return {"action": [[0.2, 0.3]]}

    monkeypatch.setattr(module, "_request_model", fake_request)
    app.infer(
        {
            "url": "http://model.test/infer",
            "prompt": "pick",
            "observation_names": ["observation.robot.joint_state"],
            "action_name": "action.robot.joint_position",
            "frame_rate": 30,
        }
    )

    assert list(captured["observations"]) == ["observation.robot.joint_state"]


def test_read_image_returns_binary_response() -> None:
    app = _debug_app()

    data, content_type = app.read_image("observation.front.image")

    assert data == b"jpeg"
    assert content_type == "image/jpeg"


def test_protobuf_url_preserves_https_and_appends_protocol_path() -> None:
    assert (
        module._protobuf_url("https://example.test/api/predict/model")
        == "https://example.test/api/predict/model/protobuf"
    )
    assert (
        module._protobuf_url("https://model.test/protobuf")
        == "https://model.test/protobuf"
    )


def test_protobuf_observation_maps_state_and_images() -> None:
    payload = {
        "prompt": "pick",
        "observations": {
            "observation.robot.joint_state": {"joint_positions": [0, 1, 2, 3, 4, 0.5]},
            "observation.front.image": {
                "width": 2,
                "height": 1,
                "encoding": "rgb888",
                "image": bytes([10, 20, 30, 40, 50, 60]),
            },
        },
    }

    proto = RynnProtoCodec()
    result = module._build_protobuf_observation(proto, payload)
    states = proto.MultiState()
    states.ParseFromString(result.state)
    images = proto.MultiImage()
    images.ParseFromString(result.image)

    assert result.prompt == "pick"
    assert states.state_list[0].name == "observation.state"
    assert list(states.state_list[0].state_data.shape) == [6]
    assert np.frombuffer(
        states.state_list[0].state_data.data,
        dtype=np.float32,
    ).tolist() == [0, 1, 2, 3, 4, 0.5]
    assert images.image_list[0].name == "observation.images.front"
    assert images.image_list[0].format == "jpeg"
    assert list(images.image_list[0].image_data.shape) == [1, 2, 3]
    with module.Image.open(
        io.BytesIO(images.image_list[0].image_data.data)
    ) as decoded:
        assert decoded.size == (2, 1)
        assert decoded.mode == "RGB"


def test_protobuf_request_uses_rynnbot_codec_and_decodes_raw_action(
    monkeypatch,
) -> None:
    proto = RynnProtoCodec()
    actions = proto.MultiAction()
    action = actions.action_list.add()
    action.name = "action"
    values = np.asarray([[0.1, 0.2], [0.3, 0.4]], dtype=np.float32)
    action.action_data.data = values.tobytes()
    action.action_data.shape.extend(values.shape)
    action.action_data.dtype = proto.DataType.FLOAT32
    action.action_rate = 20
    response = proto.build_data_packet(proto.PackageType.ACTION_DATA, actions)
    captured = {}

    class FakeResponse:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return None

        def read(self):
            return response.SerializeToString()

    def fake_urlopen(request, timeout):
        captured["request"] = request
        captured["timeout"] = timeout
        return FakeResponse()

    monkeypatch.setattr(module, "urlopen", fake_urlopen)
    result = module._request_protobuf_model(
        {
            "url": "https://example.test/model",
            "timeout_s": 5,
            "frame_rate": 30,
        },
        {
            "prompt": "pick",
            "observations": {
                "observation.robot.joint_state": {
                    "joint_positions": [0, 1, 2, 3, 4, 0.5]
                }
            },
        },
    )

    request_packet = proto.CombineDataPacket()
    request_packet.ParseFromString(captured["request"].data)
    assert captured["request"].full_url == "https://example.test/model/protobuf"
    assert captured["timeout"] == 5
    assert request_packet.prompt == "pick"
    assert result == {
        "frames": [
            {"joint_positions": pytest.approx([0.1, 0.2])},
            {"joint_positions": pytest.approx([0.3, 0.4])},
        ],
        "frame_rate": 20,
    }


def test_main_starts_rcp_server_before_web_ui(monkeypatch, tmp_path) -> None:
    events = []
    monkeypatch.setenv("RYNNRCP_HOME", str(tmp_path))
    monkeypatch.setenv("RYNNRCP_RUN_ID", "model-debug-test")

    class FakeApp:
        def __init__(self, config):
            events.append(("app", config))

        def start_server(self):
            events.append(("start",))
            return {"robot_name": "SO101", "observation_count": 2}

        def stop_server(self):
            events.append(("stop",))

    class FakeHttpServer:
        server_port = 8093

        def __init__(self, address, handler):
            events.append(("http", address, handler))

        def serve_forever(self):
            events.append(("serve",))

        def server_close(self):
            events.append(("close",))

    monkeypatch.setattr(module, "ModelDebugApp", FakeApp)
    monkeypatch.setattr(module, "ThreadingHTTPServer", FakeHttpServer)

    result = module.main(
        ["--server-config", "so101.yaml", "--host", "127.0.0.1", "--no-open"]
    )

    assert result == 0
    assert [event[0] for event in events] == [
        "app",
        "start",
        "http",
        "serve",
        "stop",
        "close",
    ]


@pytest.mark.parametrize(
    "disconnect_error",
    [BrokenPipeError, ConnectionAbortedError, ConnectionResetError],
)
def test_http_response_ignores_client_disconnect(disconnect_error) -> None:
    class DisconnectedWriter:
        def write(self, _body: bytes) -> None:
            raise disconnect_error

    handler = object.__new__(module.Handler)
    handler.send_response = lambda *_args: None
    handler.send_header = lambda *_args: None
    handler.end_headers = lambda: None
    handler.wfile = DisconnectedWriter()

    handler._send(200, b"{}", "application/json")
