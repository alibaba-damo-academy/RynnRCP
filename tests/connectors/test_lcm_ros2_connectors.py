"""Tests for LCM and ROS 2 connectors using in-memory fakes."""

from __future__ import annotations

import enum
import importlib
import json
import sys
import threading
import time
import types
from typing import Any, Callable, Dict

import pytest

import rynnrcp.connectors.lcm_connector as lcm_connector_mod


# ---------------------------------------------------------------------------
# LCM connector
# ---------------------------------------------------------------------------

class _FakeLCMInstance:
    def __init__(self, url: str | None = None) -> None:
        self.url = url
        self.published: list[tuple[str, bytes]] = []
        self.subscriptions: list[tuple[str, Callable]] = []
        self.handle_calls = 0

    def handle_timeout(self, timeout_ms: int) -> None:
        self.handle_calls += 1
        time.sleep(0.001)

    def publish(self, topic: str, payload: bytes) -> None:
        self.published.append((topic, payload))

    def subscribe(self, topic: str, handler: Callable) -> None:
        self.subscriptions.append((topic, handler))


@pytest.fixture
def lcm_connector(monkeypatch: pytest.MonkeyPatch):
    fake_lib = types.SimpleNamespace(LCM=_FakeLCMInstance)
    monkeypatch.setattr(lcm_connector_mod, "_lcm_lib", fake_lib, raising=False)
    monkeypatch.setattr(lcm_connector_mod, "LCM_AVAILABLE", True)
    connector = lcm_connector_mod.LCMConnector()
    yield connector
    connector.stop()


def test_lcm_connector_requires_lcm_library(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(lcm_connector_mod, "LCM_AVAILABLE", False)
    with pytest.raises(RuntimeError, match="lcm is not installed"):
        lcm_connector_mod.LCMConnector()


def test_lcm_connector_passes_url_through(monkeypatch: pytest.MonkeyPatch) -> None:
    fake_lib = types.SimpleNamespace(LCM=_FakeLCMInstance)
    monkeypatch.setattr(lcm_connector_mod, "_lcm_lib", fake_lib, raising=False)
    monkeypatch.setattr(lcm_connector_mod, "LCM_AVAILABLE", True)
    connector = lcm_connector_mod.LCMConnector(url="udpm://239.255.76.67:7667")
    try:
        assert connector.lcm_instance.url == "udpm://239.255.76.67:7667"
    finally:
        connector.stop()


def test_lcm_pub_encodes_protocol_json(lcm_connector) -> None:
    lcm_connector.pub({"topic": "rcp/topic"}, {"key": [1, 2]})
    topic, payload = lcm_connector.lcm_instance.published[0]
    assert topic == "rcp/topic"
    assert json.loads(payload.decode("utf-8")) == {"key": [1, 2]}


def test_lcm_pub_validates_params(lcm_connector) -> None:
    with pytest.raises(ValueError, match="topic is required"):
        lcm_connector.pub({}, {"x": 1})
    with pytest.raises(TypeError, match="protocol JSON data"):
        lcm_connector.pub({"topic": "t"}, b"raw-bytes")


def test_lcm_sub_decodes_json_payloads(lcm_connector) -> None:
    received: list[Any] = []
    lcm_connector.sub({"topic": "rcp/topic"}, received.append)
    topic, handler = lcm_connector.lcm_instance.subscriptions[0]
    assert topic == "rcp/topic"
    handler("rcp/topic", json.dumps({"v": 7}).encode("utf-8"))
    assert received == [{"v": 7}]


def test_lcm_sub_validates_params(lcm_connector) -> None:
    with pytest.raises(ValueError, match="topic is required"):
        lcm_connector.sub({}, lambda msg: None)
    with pytest.raises(ValueError, match="msg_type=json"):
        lcm_connector.sub({"topic": "t", "msg_type": "protobuf"}, lambda msg: None)


def test_lcm_loop_survives_handle_errors(monkeypatch: pytest.MonkeyPatch) -> None:
    class _Broken(_FakeLCMInstance):
        def handle_timeout(self, timeout_ms: int) -> None:
            self.handle_calls += 1
            raise RuntimeError("boom")

    fake_lib = types.SimpleNamespace(LCM=_Broken)
    monkeypatch.setattr(lcm_connector_mod, "_lcm_lib", fake_lib, raising=False)
    monkeypatch.setattr(lcm_connector_mod, "LCM_AVAILABLE", True)
    connector = lcm_connector_mod.LCMConnector()
    try:
        deadline = time.time() + 1.0
        while connector.lcm_instance.handle_calls < 2 and time.time() < deadline:
            time.sleep(0.01)
        assert connector.lcm_instance.handle_calls >= 2
        assert connector._thread.is_alive()
    finally:
        connector.stop()
    assert not connector._thread.is_alive()


# ---------------------------------------------------------------------------
# ROS 2 connector (fake rclpy injected into sys.modules)
# ---------------------------------------------------------------------------

class _Reliability(enum.Enum):
    RELIABLE = 1
    BEST_EFFORT = 2


class _History(enum.Enum):
    KEEP_LAST = 1
    KEEP_ALL = 2


class _Durability(enum.Enum):
    VOLATILE = 1
    TRANSIENT_LOCAL = 2


class _QoSProfile:
    def __init__(self, *, depth, reliability, history, durability) -> None:
        self.depth = depth
        self.reliability = reliability
        self.history = history
        self.durability = durability


class _FakeRosPublisher:
    def __init__(self) -> None:
        self.messages: list[Any] = []

    def publish(self, msg: Any) -> None:
        self.messages.append(msg)


class _FakeNode:
    def __init__(self, name: str) -> None:
        self.name = name
        self.publishers: list[tuple[type, str, Any]] = []
        self.subscriptions: list[tuple[Any, str, Callable, Any]] = []
        self.destroyed = False

    def create_publisher(self, msg_type: type, topic: str, qos: Any) -> _FakeRosPublisher:
        publisher = _FakeRosPublisher()
        self.publishers.append((msg_type, topic, qos))
        return publisher

    def create_subscription(self, msg_type: Any, topic: str, callback: Callable, qos: Any):
        self.subscriptions.append((msg_type, topic, callback, qos))

    def destroy_node(self) -> None:
        self.destroyed = True


class _FakeExecutor:
    def __init__(self, num_threads: int = 1, context: Any = None) -> None:
        self.nodes: list[Any] = []
        self._stop = threading.Event()

    def add_node(self, node: Any) -> None:
        self.nodes.append(node)

    def spin(self) -> None:
        self._stop.wait(timeout=5.0)

    def shutdown(self) -> None:
        self._stop.set()


class _FakeContext:
    def __init__(self) -> None:
        self._ok = True

    def ok(self) -> bool:
        return self._ok


def _install_fake_rclpy() -> Dict[str, Any]:
    rclpy_mod = types.ModuleType("rclpy")
    state: Dict[str, Any] = {"init": [], "shutdown": []}

    def _init(context=None):
        state["init"].append(context)

    def _create_node(name, context=None):
        return _FakeNode(name)

    def _shutdown(context=None):
        state["shutdown"].append(context)
        if context is not None:
            context._ok = False

    rclpy_mod.init = _init
    rclpy_mod.create_node = _create_node
    rclpy_mod.shutdown = _shutdown

    executors_mod = types.ModuleType("rclpy.executors")
    executors_mod.MultiThreadedExecutor = _FakeExecutor
    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.QoSProfile = _QoSProfile
    qos_mod.ReliabilityPolicy = _Reliability
    qos_mod.HistoryPolicy = _History
    qos_mod.DurabilityPolicy = _Durability
    context_mod = types.ModuleType("rclpy.context")
    context_mod.Context = _FakeContext

    rclpy_mod.executors = executors_mod
    rclpy_mod.qos = qos_mod
    rclpy_mod.context = context_mod

    sys.modules["rclpy"] = rclpy_mod
    sys.modules["rclpy.executors"] = executors_mod
    sys.modules["rclpy.qos"] = qos_mod
    sys.modules["rclpy.context"] = context_mod
    return state


@pytest.fixture(scope="module")
def ros2_module():
    had_real_rclpy = "rclpy" in sys.modules
    if had_real_rclpy:
        pytest.skip("real rclpy present; fake-based tests are redundant")
    _install_fake_rclpy()
    module = importlib.reload(importlib.import_module("rynnrcp.connectors.ros2_connector"))
    yield module
    for name in ("rclpy", "rclpy.executors", "rclpy.qos", "rclpy.context"):
        sys.modules.pop(name, None)
    importlib.reload(module)


@pytest.fixture
def ros2_connector(ros2_module):
    connector = ros2_module.ROS2Connector(node_name="test_node", num_threads=1)
    yield connector
    connector.stop()


def test_ros2_connector_requires_rclpy() -> None:
    module = importlib.import_module("rynnrcp.connectors.ros2_connector")
    if module.ROS2_AVAILABLE:
        pytest.skip("rclpy available")
    with pytest.raises(RuntimeError, match="rclpy is not installed"):
        module.ROS2Connector()


def test_ros2_build_qos_accepts_strings_and_enums(ros2_module) -> None:
    build = ros2_module.ROS2Connector._build_qos
    default = build({})
    assert default.depth == 10
    assert default.reliability is _Reliability.RELIABLE
    assert default.history is _History.KEEP_LAST
    assert default.durability is _Durability.VOLATILE

    custom = build(
        {
            "depth": 3,
            "reliability": "best_effort",
            "history": "keep_all",
            "durability": "transient_local",
        }
    )
    assert custom.depth == 3
    assert custom.reliability is _Reliability.BEST_EFFORT
    assert custom.history is _History.KEEP_ALL
    assert custom.durability is _Durability.TRANSIENT_LOCAL

    passthrough = build(
        {
            "reliability": _Reliability.BEST_EFFORT,
            "history": _History.KEEP_ALL,
            "durability": _Durability.TRANSIENT_LOCAL,
        }
    )
    assert passthrough.reliability is _Reliability.BEST_EFFORT


@pytest.mark.parametrize(
    "qos, message",
    [
        ({"reliability": "sometimes"}, "qos.reliability"),
        ({"history": "forever"}, "qos.history"),
        ({"durability": "granite"}, "qos.durability"),
    ],
)
def test_ros2_build_qos_rejects_unknown_policies(ros2_module, qos, message) -> None:
    with pytest.raises(ValueError, match=message):
        ros2_module.ROS2Connector._build_qos(qos)


def test_ros2_qos_config_merges_params_over_defaults(ros2_module) -> None:
    merged = ros2_module.ROS2Connector._qos_config(
        {"depth": 5, "reliability": "reliable"},
        {"qos": {"reliability": "best_effort"}},
    )
    assert merged == {"depth": 5, "reliability": "best_effort"}


def test_ros2_pub_pools_publishers(ros2_connector) -> None:
    class Msg:
        pass

    message = Msg()
    ros2_connector.pub({"topic": "/cmd"}, message)
    ros2_connector.pub({"topic": "/cmd"}, Msg())
    assert len(ros2_connector.publishers) == 1
    assert len(ros2_connector.node.publishers) == 1

    ros2_connector.pub({"topic": "/other"}, Msg())
    assert len(ros2_connector.publishers) == 2

    with pytest.raises(ValueError, match="topic is required"):
        ros2_connector.pub({}, message)


def test_ros2_sub_registers_subscription(ros2_connector, monkeypatch) -> None:
    module = sys.modules["rynnrcp.connectors.ros2_connector"]

    class FakeString:
        pass

    monkeypatch.setattr(module, "get_message_class", lambda name: FakeString)

    received: list[Any] = []
    ros2_connector.sub(
        {"topic": "/chatter", "msg_type": "std_msgs/msg/String"}, received.append
    )
    msg_type, topic, callback, _qos = ros2_connector.node.subscriptions[0]
    assert msg_type is FakeString and topic == "/chatter"
    callback("plain")
    assert received == ["plain"]

    with pytest.raises(ValueError, match="topic and params.msg_type"):
        ros2_connector.sub({"topic": "/x"}, received.append)


def test_ros2_sub_protocol_json_mode_decodes(ros2_connector, monkeypatch) -> None:
    module = sys.modules["rynnrcp.connectors.ros2_connector"]
    monkeypatch.setattr(module, "get_message_class", lambda name: object)

    received: list[Any] = []
    ros2_connector.sub(
        {
            "topic": "/json",
            "msg_type": "std_msgs/msg/String",
            "payload_mode": "protocol_json",
        },
        received.append,
    )
    _msg_type, _topic, callback, _qos = ros2_connector.node.subscriptions[-1]
    wrapper = types.SimpleNamespace(data=json.dumps({"k": 1}))
    callback(wrapper)
    assert received == [{"k": 1}]


def test_ros2_stop_shuts_everything_down(ros2_module) -> None:
    connector = ros2_module.ROS2Connector(node_name="stop_node")
    node = connector.node
    connector.stop()
    assert node.destroyed is True
    assert not connector._executor_thread.is_alive()


def test_decode_protocol_json_message_variants(ros2_module) -> None:
    decode = ros2_module._decode_protocol_json_message
    assert decode(types.SimpleNamespace(data='{"a":1}')) == {"a": 1}
    assert decode(types.SimpleNamespace(data=b'{"b":2}')) == {"b": 2}
    assert decode({"c": 3}) == {"c": 3}
    with pytest.raises(TypeError, match="must provide string, bytes, or dict"):
        decode(types.SimpleNamespace(data=123))
    with pytest.raises(TypeError, match="decode to a JSON object"):
        decode(types.SimpleNamespace(data="[1,2]"))
