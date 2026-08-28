"""Extended coverage for protocol input/image/action adapters."""

from __future__ import annotations

import types
from typing import Any

import pytest

from rynnrcp.adapters.protocol_action_output_adapter import (
    ProtocolActionOutputAdapter,
    _assign_path,
    _extract_path,
)
from rynnrcp.adapters.protocol_image_input_adapter import ProtocolImageInputAdapter
from rynnrcp.adapters.protocol_input_adapter import ProtocolInputAdapter


# ---------------------------------------------------------------------------
# ProtocolInputAdapter
# ---------------------------------------------------------------------------

def test_input_adapter_requires_params() -> None:
    with pytest.raises(ValueError, match="requires params"):
        ProtocolInputAdapter(None)


def test_input_adapter_field_access_paths() -> None:
    get = ProtocolInputAdapter._get_field
    msg = types.SimpleNamespace(
        position=[1.0, 2.0, 3.0],
        nested=types.SimpleNamespace(values=[10, 20, 30, 40]),
        table={"row": [5, 6]},
    )
    assert get(msg, "position") == [1.0, 2.0, 3.0]
    assert get(msg, "position[1]") == 2.0
    assert get(msg, "position[0:2]") == [1.0, 2.0]
    assert get(msg, "position[*]") == [1.0, 2.0, 3.0]
    assert get(msg, "nested.values[1:]") == [20, 30, 40]
    assert get(msg, "table") == {"row": [5, 6]}
    assert get({"a": {"b": 7}}, "a.b") == 7


def test_input_adapter_timestamp_extraction() -> None:
    extract = ProtocolInputAdapter._extract_timestamp
    ros2 = types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=1, nanosec=500000000))
    )
    assert extract(ros2) == pytest.approx(1.5)

    lcm = types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=None, stamp_sec=2, stamp_nanosec=250000000)
    )
    assert extract(lcm) == pytest.approx(2.25)

    assert extract({"timestamp": 42.0}) == 42.0
    assert extract(object()) > 0


def test_input_adapter_parse_with_mappings_converts_iterables() -> None:
    adapter = ProtocolInputAdapter(
        {
            "object_name": "obs",
            "mappings": [
                {"field": "position", "object_name": "observation.robot.joint_state"},
                {"field": "gen", "object_name": "observation.robot.velocities"},
            ],
        }
    )
    msg = types.SimpleNamespace(position=(1, 2), gen=range(3))
    _ts, parsed = adapter.parse(msg)
    assert parsed["observation.robot.joint_state"] == [1.0, 2.0]
    assert parsed["observation.robot.velocities"] == [0.0, 1.0, 2.0]


def test_input_adapter_parse_without_mappings() -> None:
    adapter = ProtocolInputAdapter({"object_name": "obs"})
    _ts, parsed = adapter.parse([3, 4])
    assert parsed == {"obs": [3.0, 4.0]}

    _ts, parsed = adapter.parse({"raw": True})
    assert parsed == {"obs": {"raw": True}}

    _ts, parsed = adapter.parse("scalar")
    assert parsed == {"obs": "scalar"}


def test_input_adapter_protocol_json_mode() -> None:
    adapter = ProtocolInputAdapter(
        {"object_name": "obs", "payload_mode": "protocol_json"}
    )
    _ts, parsed = adapter.parse(
        {"name": "observation.robot.joint_state", "value": {"joint_positions": [1]}}
    )
    assert parsed == {"observation.robot.joint_state": {"joint_positions": [1]}}

    for bad in ([1, 2], {"value": 1}, {"name": 5, "value": 1}, {"name": "x"}):
        with pytest.raises(ValueError, match="protocol_json input"):
            adapter.parse(bad)


# ---------------------------------------------------------------------------
# ProtocolImageInputAdapter
# ---------------------------------------------------------------------------

def test_image_adapter_requires_params() -> None:
    with pytest.raises(ValueError, match="requires params"):
        ProtocolImageInputAdapter(None)


def test_image_adapter_failed_tuple_returns_empty() -> None:
    adapter = ProtocolImageInputAdapter({"object_name": "img"})
    _ts, parsed = adapter.parse((False, 0, 0, "", b""))
    assert parsed == {}


def test_image_adapter_dict_passthrough() -> None:
    adapter = ProtocolImageInputAdapter({"object_name": "img"})
    value = {"width": 1, "height": 1, "encoding": "jpeg", "image": b"x"}
    _ts, parsed = adapter.parse(value)
    assert parsed == {"img": value}

    _ts, parsed = adapter.parse({"not_image": 1})
    assert parsed == {}


def test_image_adapter_ros2_image_message() -> None:
    adapter = ProtocolImageInputAdapter({"object_name": "img"})
    msg = types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=3, nanosec=0)),
        width=2,
        height=1,
        encoding="rgb8",
        data=b"\x00" * 6,
    )
    ts, parsed = adapter.parse(msg)
    assert ts == pytest.approx(3.0)
    assert parsed["img"]["width"] == 2
    assert parsed["img"]["image"] == msg.data


def test_image_adapter_compressed_message_needs_configured_meta() -> None:
    compressed = types.SimpleNamespace(
        header=types.SimpleNamespace(
            stamp=None, stamp_sec=4, stamp_nanosec=500000000
        ),
        format="jpeg",
        data=b"jpegbytes",
    )

    unconfigured = ProtocolImageInputAdapter({"object_name": "img"})
    with pytest.raises(ValueError, match="require width, height, and encoding"):
        unconfigured.parse(compressed)

    adapter = ProtocolImageInputAdapter(
        {"object_name": "img", "width": 640, "height": 480, "encoding": "jpeg"}
    )
    ts, parsed = adapter.parse(compressed)
    assert ts == pytest.approx(4.5)
    assert parsed["img"] == {
        "width": 640,
        "height": 480,
        "encoding": "jpeg",
        "image": b"jpegbytes",
    }


def test_image_adapter_unknown_message_returns_empty() -> None:
    adapter = ProtocolImageInputAdapter({"object_name": "img"})
    _ts, parsed = adapter.parse(object())
    assert parsed == {}


# ---------------------------------------------------------------------------
# ProtocolActionOutputAdapter helpers
# ---------------------------------------------------------------------------

def test_extract_path_supports_dict_sequence_and_attributes() -> None:
    value = {"pose": types.SimpleNamespace(xyz=[1.0, 2.0, 3.0])}
    assert _extract_path(value, "") is value
    assert _extract_path(value, "pose.xyz.1") == 2.0
    assert _extract_path({"a": [{"b": 5}]}, "a.0.b") == 5


def test_assign_path_walks_dicts_and_objects() -> None:
    target: dict[str, Any] = {}
    _assign_path(target, "linear.x", 1.5)
    assert target == {"linear": {"x": 1.5}}

    obj = types.SimpleNamespace(linear=types.SimpleNamespace(x=0.0))
    _assign_path(obj, "linear.x", 2.5)
    assert obj.linear.x == 2.5


# ---------------------------------------------------------------------------
# ProtocolActionOutputAdapter.build_step_output
# ---------------------------------------------------------------------------

def _adapter() -> ProtocolActionOutputAdapter:
    return ProtocolActionOutputAdapter()


def test_build_step_output_module_connector_wraps_value() -> None:
    params = {
        "rcp_action_name": "action.robot.joint_position",
        "rcp_action_type": "joint_position",
        "connector": "module",
    }
    step = {"action.robot.joint_position": {"joint_positions": [1.0]}}
    groups = _adapter().build_step_output(step, [params], fps=10.0)
    assert len(groups) == 1
    connector, out_params, msg, interval, repeat = groups[0][0]
    assert connector == "module"
    assert out_params is params
    assert msg == {"action.robot.joint_position": {"joint_positions": [1.0]}}
    assert interval == pytest.approx(0.1)
    assert repeat == 1


def test_build_step_output_protocol_json_plain_connector() -> None:
    params = {
        "rcp_action_name": "action.robot.joint_position",
        "rcp_action_type": "joint_position",
        "connector": "lcm",
        "payload_mode": "protocol_json",
    }
    groups = _adapter().build_step_output(
        {"action.robot.joint_position": {"joint_positions": [1, 2]}}, [params], fps=0.0
    )
    _connector, _params, msg, interval, _repeat = groups[0][0]
    assert msg == {
        "name": "action.robot.joint_position",
        "type": "joint_position",
        "value": {"joint_positions": [1.0, 2.0]},
    }
    assert interval == 0.0


def test_build_step_output_mappings_route_values(monkeypatch: pytest.MonkeyPatch) -> None:
    import rynnrcp.adapters.protocol_action_output_adapter as adapter_mod

    class TwistLike:
        def __init__(self) -> None:
            self.linear = types.SimpleNamespace(x=0.0)

    monkeypatch.setattr(
        adapter_mod, "get_message_class", lambda name: TwistLike
    )
    params = {
        "rcp_action_name": "action.base.base_velocity",
        "rcp_action_type": "base_velocity",
        "connector": "ros2",
        "msg_type": "geometry_msgs/msg/Twist",
        "mappings": [
            {
                "name": "action.base.base_velocity",
                "field": "linear.x",
                "from": "linear_vel.0",
            }
        ],
    }
    step = {"action.base.base_velocity": {"linear_vel": [0.7, 0.0, 0.0]}}
    groups = _adapter().build_step_output(step, [params], fps=30.0)
    _connector, _params, msg, _interval, _repeat = groups[0][0]
    assert msg.linear.x == 0.7


def test_build_step_output_mapping_validation(monkeypatch: pytest.MonkeyPatch) -> None:
    import rynnrcp.adapters.protocol_action_output_adapter as adapter_mod

    monkeypatch.setattr(
        adapter_mod, "get_message_class", lambda name: types.SimpleNamespace
    )
    base_params = {
        "rcp_action_name": "action.base.base_velocity",
        "rcp_action_type": "base_velocity",
        "connector": "ros2",
        "msg_type": "geometry_msgs/msg/Twist",
    }
    step = {"action.base.base_velocity": {"linear_vel": [0.0, 0.0, 0.0]}}
    with pytest.raises(ValueError, match="requires mappings"):
        _adapter().build_step_output(step, [dict(base_params)], 30.0)

    mismatched = {
        **base_params,
        "mappings": [{"name": "other.action", "field": "linear.x", "from": ""}],
    }
    with pytest.raises(ValueError, match="does not match action"):
        _adapter().build_step_output(step, [mismatched], 30.0)


def test_build_step_output_ros2_protocol_json_requires_data_field(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import rynnrcp.adapters.protocol_action_output_adapter as adapter_mod

    class NoData:
        __slots__ = ()

    monkeypatch.setattr(adapter_mod, "get_message_class", lambda name: NoData)
    params = {
        "rcp_action_name": "action.robot.joint_position",
        "rcp_action_type": "joint_position",
        "connector": "ros2",
        "payload_mode": "protocol_json",
        "msg_type": "std_msgs/msg/String",
    }
    with pytest.raises(TypeError, match="data field"):
        _adapter().build_step_output(
            {"action.robot.joint_position": {"joint_positions": [1]}}, [params], 30.0
        )


def test_build_step_output_empty_outputs_returns_no_groups() -> None:
    assert _adapter().build_step_output({"a": 1}, [], fps=30.0) == []
