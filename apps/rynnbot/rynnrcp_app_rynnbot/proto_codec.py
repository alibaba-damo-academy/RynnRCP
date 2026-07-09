# rynnrcp_app_rynnbot/proto_codec.py

"""
Protobuf packet codec for the Rynnbot transport protocol.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rynnrcp_app_rynnbot.proto_codec.RynnProtoCodec`,
a small helper that decodes/encodes the Rynnbot protobuf wire format.

It provides:
- parsing raw bytes into a transport ``DataPacket``
- decoding the inner business message based on the packet ``type``
- building a ``DataPacket`` from an inner protobuf message (with timestamp/id)
- serializing packets back to bytes for transmission

The codec also exposes commonly used protobuf message/types as instance attributes
to keep call sites concise.
"""

from __future__ import annotations

from typing import Any, Optional, Tuple
import time

from .proto import (
    RobotServerData_pb2,
    RobotServerTransportPacket_pb2,
    RobotServerCombinePacket_pb2,
)


class RynnProtoCodec:
    def __init__(self) -> None:
        """Initialize the protocol codec and attach commonly used protobuf types to the instance."""
        self.DataPacket = RobotServerTransportPacket_pb2.DataPacket
        self.CommonPartAttr = RobotServerTransportPacket_pb2.CommonPartAttr
        self.PackageType = RobotServerTransportPacket_pb2.PackageType

        self.Image = RobotServerData_pb2.Image
        self.MultiImage = RobotServerData_pb2.MultiImage
        self.State = RobotServerData_pb2.State
        self.MultiState = RobotServerData_pb2.MultiState
        self.Action = RobotServerData_pb2.Action
        self.MultiAction = RobotServerData_pb2.MultiAction
        self.Array = RobotServerData_pb2.Array
        self.DataType = RobotServerData_pb2.DataType

        self.ReqImage = RobotServerData_pb2.ReqImage
        self.ReqState = RobotServerData_pb2.ReqState
        self.FinishActionChunk = RobotServerData_pb2.FinishActionChunk

        self.CombineDataPacket = RobotServerCombinePacket_pb2.CombineDataPacket

    def parse_data_packet(
        self, raw_bytes: bytes
    ) -> RobotServerTransportPacket_pb2.DataPacket:
        """Parse raw bytes into a DataPacket object."""
        packet = self.DataPacket()
        packet.ParseFromString(raw_bytes)
        return packet

    def parse_inner_payload(
        self, packet: RobotServerTransportPacket_pb2.DataPacket
    ) -> Tuple[int, Optional[Any]]:
        """Parse the internal business message based on the `type` field in the DataPacket."""
        pkg_type = packet.common_part_attr.type

        if pkg_type == self.PackageType.IMAGE_DATA:
            msg = self.MultiImage()
        elif pkg_type == self.PackageType.STATE_DATA:
            msg = self.MultiState()
        elif pkg_type == self.PackageType.ACTION_DATA:
            msg = self.MultiAction()
        elif pkg_type == self.PackageType.REQ_IMAGE:
            msg = self.ReqImage()
        elif pkg_type == self.PackageType.REQ_STATE:
            msg = self.ReqState()
        elif pkg_type == self.PackageType.ACTION_FINISH:
            msg = self.FinishActionChunk()
        elif pkg_type == self.PackageType.RESV:
            return pkg_type, packet.data
        else:
            return pkg_type, None

        msg.ParseFromString(packet.data)
        return pkg_type, msg

    def build_data_packet(
        self, pkg_type: int, inner_msg: Any, id: int = 0
    ) -> RobotServerTransportPacket_pb2.DataPacket:
        """Build a DataPacket from an internal business message."""
        packet = self.DataPacket()
        packet.common_part_attr.timestamp = int(time.time() * 1000)
        packet.common_part_attr.id = id
        packet.common_part_attr.type = pkg_type
        packet.data = inner_msg.SerializeToString()
        return packet

    @staticmethod
    def serialize_data_packet(packet: Any) -> bytes:
        """Serialize a DataPacket or any protobuf object to bytes."""
        return packet.SerializeToString()
