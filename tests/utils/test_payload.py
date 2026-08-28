"""Tests for channel payload packing/parsing helpers in rynnrcp.utils.payload."""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass
from typing import Any, Dict

import pytest

from rynnrcp.utils import payload as payload_mod
from rynnrcp.utils.payload import (
    CachedSharedRefParser,
    bytes_payload,
    bytes_shared_payload,
    close_shared_reader_cache,
    decode_action_payload,
    image_adapter_shared_payload,
    json_dumps_bytes,
    json_observation_payload,
    json_safe,
    json_shared_payload,
    pack_channel_message,
    parse_channel_payload,
)
from rynnrcp.utils.shared_data_store import (
    PAYLOAD_REF_TYPE,
    SharedDataExpired,
    SharedDataStore,
)


@dataclass
class _Spec:
    channel: str = "observation.robot.joint_state"
    object_name: str = "observation.robot.joint_state"


@pytest.fixture
def store():
    store = SharedDataStore("test_payload_store", create=True, buffer_size=1 << 16, slot_count=2)
    yield store
    store.close(unlink=True)


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def test_pack_channel_message_prefixes_timestamp_header() -> None:
    packed = pack_channel_message(12.5, b"abc")
    (timestamp,) = struct.unpack_from("<d", packed)
    assert timestamp == 12.5
    assert packed[8:] == b"abc"


def test_json_safe_passes_scalars_and_containers() -> None:
    value = {"a": [1, 2.5, True, None, ("x",)], 3: "three"}
    assert json_safe(value) == {"a": [1, 2.5, True, None, ["x"]], "3": "three"}


def test_json_safe_rejects_non_json_values() -> None:
    with pytest.raises(TypeError, match="not JSON compatible"):
        json_safe({"bad": object()})


def test_bytes_payload_accepts_raw_bytes_and_tuples() -> None:
    timestamp, data = bytes_payload(b"raw")
    assert data == b"raw" and timestamp > 0

    timestamp, data = bytes_payload((True, 3.5, bytearray(b"ok")))
    assert (timestamp, data) == (3.5, b"ok")

    timestamp, data = bytes_payload((False, 7.0, b"ignored"))
    assert (timestamp, data) == (7.0, b"")


def test_bytes_payload_rejects_unknown_shapes() -> None:
    with pytest.raises(TypeError, match="bytes payload connector"):
        bytes_payload({"not": "supported"})


def test_json_observation_payload_wraps_message_under_object_name() -> None:
    raw = json_observation_payload({"pos": [1, 2]}, "obs.robot", 5.0)
    decoded = json.loads(raw)
    assert decoded == {"timestamp": 5.0, "extra": {"obs.robot": {"pos": [1, 2]}}}


def test_json_observation_payload_keeps_dict_containing_object_name() -> None:
    raw = json_observation_payload({"obs.robot": {"pos": 1}, "other": 2}, "obs.robot", 1.0)
    decoded = json.loads(raw)
    assert decoded["extra"] == {"obs.robot": {"pos": 1}, "other": 2}


def test_json_observation_payload_decodes_json_bytes() -> None:
    raw = json_observation_payload(json_dumps_bytes({"v": 1}), "obs", 2.0)
    assert json.loads(raw)["extra"] == {"obs": {"v": 1}}


def test_json_observation_payload_falls_back_for_invalid_bytes() -> None:
    raw = json_observation_payload(b"\xff\xfenot-json", "obs", 2.0)
    extra = json.loads(raw)["extra"]
    assert isinstance(extra["obs"], str)


# ---------------------------------------------------------------------------
# decode_action_payload
# ---------------------------------------------------------------------------

def _action_extra(**overrides: Any) -> Dict[str, Any]:
    extra: Dict[str, Any] = {
        "action_name": "joint_positions",
        "joint_positions": [[0.0] * 6],
        "fps": 30,
        "action_type": "joint_position",
    }
    extra.update(overrides)
    return extra


def test_decode_action_payload_accepts_valid_payload() -> None:
    raw = json_dumps_bytes({"timestamp": 1.0, "extra": _action_extra()})
    data, extra = decode_action_payload(raw, "action.robot")
    assert data["timestamp"] == 1.0
    assert extra["action_name"] == "joint_positions"


@pytest.mark.parametrize(
    "payload_bytes, message",
    [
        (b"\xff\xff", "must publish JSON"),
        (json_dumps_bytes([1, 2]), "must be a JSON object"),
        (json_dumps_bytes({"extra": "nope"}), "extra must be a JSON object"),
        (json_dumps_bytes({"extra": {}}), "missing action_name"),
        (
            json_dumps_bytes({"extra": {"action_name": "move", "fps": 1, "action_type": "x"}}),
            "missing move",
        ),
        (
            json_dumps_bytes({"extra": {"action_name": "move", "move": 1, "action_type": "x"}}),
            "missing fps",
        ),
        (
            json_dumps_bytes({"extra": {"action_name": "move", "move": 1, "fps": 1}}),
            "missing action_type",
        ),
    ],
)
def test_decode_action_payload_rejects_malformed_payloads(
    payload_bytes: bytes, message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        decode_action_payload(payload_bytes, "action.robot")


def test_decode_action_payload_resolves_shared_refs(store: SharedDataStore) -> None:
    action_bytes = json_dumps_bytes({"timestamp": 2.0, "extra": _action_extra()})
    ref = store.write(action_bytes, timestamp=2.0, codec="json").to_dict()
    raw = json_dumps_bytes(ref)

    data, extra = decode_action_payload(raw, "action.robot")
    assert data["timestamp"] == 2.0
    assert extra["fps"] == 30


def test_decode_action_payload_rejects_bad_shared_ref() -> None:
    bad_prefix = b'{"type":"rynnrcp.shared_payload_ref.v1"'
    with pytest.raises(ValueError, match="ref must be JSON"):
        decode_action_payload(bad_prefix + b"broken", "action.robot")

    # Duplicate JSON keys keep the last value, so the decoded type mismatches
    # while the raw payload still starts with the shared-ref prefix.
    tampered = bad_prefix + b',"type":"other"}'
    with pytest.raises(ValueError, match="unsupported type"):
        decode_action_payload(tampered, "action.robot")


# ---------------------------------------------------------------------------
# parse_channel_payload (inline JSON + shared refs)
# ---------------------------------------------------------------------------

def test_parse_channel_payload_inline_json() -> None:
    spec = _Spec()
    raw = json_observation_payload({"pos": [1]}, spec.object_name, 1.0)
    assert parse_channel_payload(spec, raw) == {"pos": [1]}


@pytest.mark.parametrize(
    "payload_bytes, message",
    [
        (b"\xff", "must publish a JSON state payload"),
        (json_dumps_bytes([1]), "must be a JSON object"),
        (json_dumps_bytes({"extra": 5}), "extra must be a JSON object"),
        (json_dumps_bytes({"extra": {}}), "missing configured object_name"),
    ],
)
def test_parse_channel_payload_rejects_bad_inline_json(
    payload_bytes: bytes, message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        parse_channel_payload(_Spec(), payload_bytes)


def _shared_ref_payload(
    store: SharedDataStore,
    *,
    object_name: str,
    payload_type: str,
    data: bytes,
    meta: Dict[str, Any] | None = None,
) -> bytes:
    ref = store.write(data, timestamp=1.0, codec=payload_type, meta=meta or {}).to_dict()
    ref["payload_type"] = payload_type
    ref["object_name"] = object_name
    return json_dumps_bytes(ref)


def test_parse_channel_payload_shared_bytes(store: SharedDataStore) -> None:
    spec = _Spec()
    raw = _shared_ref_payload(
        store, object_name=spec.object_name, payload_type="bytes", data=b"blob"
    )
    assert parse_channel_payload(spec, raw) == b"blob"


def test_parse_channel_payload_shared_json(store: SharedDataStore) -> None:
    spec = _Spec()
    inner = json_observation_payload({"v": 3}, spec.object_name, 1.0)
    raw = _shared_ref_payload(
        store, object_name=spec.object_name, payload_type="json", data=inner
    )
    assert parse_channel_payload(spec, raw) == {"v": 3}


def test_parse_channel_payload_shared_image(store: SharedDataStore) -> None:
    spec = _Spec(object_name="observation.front.image")
    meta = {"width": 4, "height": 2, "encoding": "jpeg", "size_bytes": 3}
    raw = _shared_ref_payload(
        store, object_name=spec.object_name, payload_type="image", data=b"img", meta=meta
    )
    parsed = parse_channel_payload(spec, raw)
    assert parsed["width"] == 4
    assert parsed["height"] == 2
    assert parsed["encoding"] == "jpeg"
    assert bytes(parsed["image"]) == b"img"


def test_parse_channel_payload_shared_ref_validation(store: SharedDataStore) -> None:
    spec = _Spec()
    with pytest.raises(ValueError, match="ref must be JSON"):
        parse_channel_payload(spec, b'{"type":"rynnrcp.shared_payload_ref.v1"broken')

    mismatched = _shared_ref_payload(
        store, object_name="other.object", payload_type="bytes", data=b"x"
    )
    with pytest.raises(ValueError, match="object_name mismatch"):
        parse_channel_payload(spec, mismatched)

    unsupported = _shared_ref_payload(
        store, object_name=spec.object_name, payload_type="mystery", data=b"x"
    )
    with pytest.raises(ValueError, match="unsupported payload_type"):
        parse_channel_payload(spec, unsupported)


def test_parse_channel_payload_reader_cache_reattaches_after_expiry(
    store: SharedDataStore,
) -> None:
    spec = _Spec()
    cache: Dict[str, SharedDataStore] = {}
    raw = _shared_ref_payload(
        store, object_name=spec.object_name, payload_type="bytes", data=b"first"
    )
    assert parse_channel_payload(spec, raw, reader_cache=cache) == b"first"
    assert len(cache) == 1

    # Overwrite every slot so the first ref expires, then publish fresh data.
    for index in range(4):
        newest = _shared_ref_payload(
            store,
            object_name=spec.object_name,
            payload_type="bytes",
            data=f"new-{index}".encode(),
        )
    assert parse_channel_payload(spec, newest, reader_cache=cache) == b"new-3"
    close_shared_reader_cache(cache)
    assert cache == {}


# ---------------------------------------------------------------------------
# Shared payload writers
# ---------------------------------------------------------------------------

def test_bytes_shared_payload_roundtrip(store: SharedDataStore) -> None:
    spec = _Spec()
    timestamp, raw = bytes_shared_payload(b"blob", spec.object_name, store)
    assert timestamp > 0
    assert parse_channel_payload(spec, raw) == b"blob"


def test_bytes_shared_payload_passes_through_empty_payload(store: SharedDataStore) -> None:
    timestamp, raw = bytes_shared_payload((False, 5.0, b""), "obs", store)
    assert (timestamp, raw) == (5.0, b"")


def test_json_shared_payload_roundtrip(store: SharedDataStore) -> None:
    spec = _Spec()
    raw = json_shared_payload({"v": 9}, spec.object_name, 4.0, store)
    assert parse_channel_payload(spec, raw) == {"v": 9}


def test_image_adapter_shared_payload_from_tuple(store: SharedDataStore) -> None:
    spec = _Spec(object_name="observation.front.image")
    msg = (True, 4, 2, "jpeg", b"imgdata")
    timestamp, raw = image_adapter_shared_payload(msg, None, spec.object_name, store)
    parsed = parse_channel_payload(spec, raw)
    assert timestamp > 0
    assert bytes(parsed["image"]) == b"imgdata"
    assert parsed["width"] == 4 and parsed["height"] == 2


def test_image_adapter_shared_payload_skips_failed_tuple(store: SharedDataStore) -> None:
    timestamp, raw = image_adapter_shared_payload(
        (False, 0, 0, "", b""), None, "obs.image", store
    )
    assert raw == b"" and timestamp > 0


def test_image_adapter_shared_payload_uses_input_adapter(store: SharedDataStore) -> None:
    spec = _Spec(object_name="observation.front.image")

    class Adapter:
        def parse(self, msg: Any):
            return 7.5, {
                spec.object_name: {
                    "width": 2,
                    "height": 1,
                    "encoding": "rgb8",
                    "image": memoryview(b"\x01\x02\x03\x04\x05\x06"),
                }
            }

    timestamp, raw = image_adapter_shared_payload("msg", Adapter(), spec.object_name, store)
    assert timestamp == 7.5
    parsed = parse_channel_payload(spec, raw)
    assert bytes(parsed["image"]) == b"\x01\x02\x03\x04\x05\x06"


def test_image_adapter_shared_payload_skips_missing_object(store: SharedDataStore) -> None:
    class Adapter:
        def parse(self, msg: Any):
            return 1.0, {}

    timestamp, raw = image_adapter_shared_payload("msg", Adapter(), "obs.image", store)
    assert (timestamp, raw) == (1.0, b"")


def test_image_adapter_shared_payload_accepts_numpy_frames(store: SharedDataStore) -> None:
    numpy = pytest.importorskip("numpy")
    spec = _Spec(object_name="observation.front.image")
    frame = numpy.arange(6, dtype=numpy.uint8).reshape(2, 3)
    msg = (True, 3, 2, "mono8", frame)
    _timestamp, raw = image_adapter_shared_payload(msg, None, spec.object_name, store)
    parsed = parse_channel_payload(spec, raw)
    assert bytes(parsed["image"]) == frame.tobytes()


# ---------------------------------------------------------------------------
# CachedSharedRefParser
# ---------------------------------------------------------------------------

def test_cached_parser_parses_inline_and_all_shared_types(store: SharedDataStore) -> None:
    parser = CachedSharedRefParser()
    try:
        spec = _Spec()
        inline = json_observation_payload({"pos": 1}, spec.object_name, 1.0)
        assert parser.parse(spec, inline) == {"pos": 1}

        raw = _shared_ref_payload(
            store, object_name=spec.object_name, payload_type="bytes", data=b"cached"
        )
        assert parser.parse(spec, raw) == b"cached"
        # Second parse must reuse the attached reader.
        raw2 = _shared_ref_payload(
            store, object_name=spec.object_name, payload_type="bytes", data=b"cached2"
        )
        assert parser.parse(spec, raw2) == b"cached2"
        assert len(parser._readers) == 1

        image_spec = _Spec(object_name="obs.image")
        image_raw = _shared_ref_payload(
            store,
            object_name=image_spec.object_name,
            payload_type="image",
            data=b"i",
            meta={"width": 1, "height": 1, "encoding": "jpeg"},
        )
        image = parser.parse(image_spec, image_raw)
        assert bytes(image["image"]) == b"i"

        json_raw = _shared_ref_payload(
            store,
            object_name=spec.object_name,
            payload_type="json",
            data=json_observation_payload({"v": 2}, spec.object_name, 1.0),
        )
        assert parser.parse(spec, json_raw) == {"v": 2}
    finally:
        parser.close()
    assert parser._readers == {}


def test_cached_parser_rejects_invalid_refs(store: SharedDataStore) -> None:
    parser = CachedSharedRefParser()
    spec = _Spec()
    try:
        with pytest.raises(ValueError, match="ref must be JSON"):
            parser.parse(spec, b'{"type":"rynnrcp.shared_payload_ref.v1"broken')

        mismatch = _shared_ref_payload(
            store, object_name="other", payload_type="bytes", data=b"x"
        )
        with pytest.raises(ValueError, match="object_name mismatch"):
            parser.parse(spec, mismatch)

        unsupported = _shared_ref_payload(
            store, object_name=spec.object_name, payload_type="mystery", data=b"x"
        )
        with pytest.raises(ValueError, match="unsupported payload_type"):
            parser.parse(spec, unsupported)

        missing_name = json.loads(
            _shared_ref_payload(
                store, object_name=spec.object_name, payload_type="bytes", data=b"x"
            )
        )
        missing_name["name"] = ""
        with pytest.raises(ValueError, match="missing 'name'"):
            parser.parse(spec, json_dumps_bytes(missing_name))
    finally:
        parser.close()


def test_cached_parser_drops_stale_reader_on_expiry(store: SharedDataStore) -> None:
    parser = CachedSharedRefParser()
    spec = _Spec()
    try:
        stale = _shared_ref_payload(
            store, object_name=spec.object_name, payload_type="bytes", data=b"old"
        )
        assert parser.parse(spec, stale) == b"old"

        # Rotate through every slot so the stale ref's slot is reused.
        for index in range(4):
            fresh = _shared_ref_payload(
                store,
                object_name=spec.object_name,
                payload_type="bytes",
                data=f"fresh-{index}".encode(),
            )
        with pytest.raises(SharedDataExpired):
            parser.parse(spec, stale)
        assert parser._readers == {}
        # Next parse transparently re-attaches.
        assert parser.parse(spec, fresh) == b"fresh-3"
    finally:
        parser.close()
