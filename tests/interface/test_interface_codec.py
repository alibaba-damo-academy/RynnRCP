from __future__ import annotations

from rynnrcp.interface.codec import (
    STATUS_OK,
    InterfaceRequest,
    ok_response,
    pack_request,
    pack_response,
    unpack_request,
    unpack_response,
)


def test_messagepack_request_roundtrip_supports_dynamic_payload() -> None:
    request = InterfaceRequest(
        method="get_observations",
        payload={"names": ["observation.robot.joint_state"], "raw": b"bytes"},
        metadata={"client_id": "test"},
        timeout_ms=100,
    )

    decoded = unpack_request(pack_request(request))

    assert decoded.method == "get_observations"
    assert decoded.payload == {"names": ["observation.robot.joint_state"], "raw": b"bytes"}
    assert decoded.metadata == {"client_id": "test"}
    assert decoded.timeout_ms == 100


def test_messagepack_response_roundtrip() -> None:
    response = ok_response("req-1", {"state": [1.0, 2.0]})

    decoded = unpack_response(pack_response(response))

    assert decoded.request_id == "req-1"
    assert decoded.status == STATUS_OK
    assert decoded.payload == {"state": [1.0, 2.0]}
