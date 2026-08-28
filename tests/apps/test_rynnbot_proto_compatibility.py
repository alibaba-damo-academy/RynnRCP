"""Compatibility checks for the protobuf runtime used by RynnBot."""

from rynnrcp_app_rynnbot.proto_codec import RynnProtoCodec


def test_cloud_packet_wire_format_stays_stable_with_protobuf_4_runtime() -> None:
    proto = RynnProtoCodec()
    actions = proto.MultiAction()
    action = actions.action_list.add(id=7, name="action", action_rate=30)
    action.action_data.data = b"\x00\x00\x80?"
    action.action_data.shape.extend([1, 1])
    action.action_data.dtype = proto.DataType.FLOAT32
    packet = proto.DataPacket()
    packet.common_part_attr.timestamp = 123456789
    packet.common_part_attr.id = 42
    packet.common_part_attr.type = proto.PackageType.ACTION_DATA
    packet.data = actions.SerializeToString()

    assert packet.SerializeToString().hex() == (
        "0a0908959aef3a102a1803121c0a1a08071206616374696f6e"
        "1a0c0a040000803f120201011806201e"
    )
