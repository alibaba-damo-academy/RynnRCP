from __future__ import annotations

from rynnrcp.utils import device_identity


def test_machine_mac_suffix_is_stable_and_does_not_expose_raw_mac(monkeypatch) -> None:
    monkeypatch.setattr(device_identity.uuid, "getnode", lambda: 0x001122334455)

    first = device_identity.machine_mac_suffix()
    second = device_identity.machine_mac_suffix()

    assert first == second
    assert len(first) == 8
    assert first != "22334455"


def test_with_machine_suffix_replaces_an_existing_generated_suffix() -> None:
    assert (
        device_identity.with_machine_suffix("so101_follower_deadbeef", "so101_follower", "1234abcd")
        == "so101_follower_1234abcd"
    )
