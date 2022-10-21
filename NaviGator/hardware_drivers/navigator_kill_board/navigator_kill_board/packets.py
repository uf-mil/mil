import struct
from enum import IntEnum

from mil_usb_to_can import ApplicationPacket, CommandPacket, ReceivePacket
from navigator_kill_board import constants


class KillMessage(CommandPacket):
    @classmethod
    def create_message(cls, message: bytes):
        assert (
            message != constants["REQUEST"]["KILL_STATE_REQUEST"]
        ), "Cannot use KillMessage with REQUEST, use RequestMessage Instead"
        return super().create_send_packet(
            message, int.from_bytes(constants["CAN_ID"], "big")
        )

    def __str__(self):
        return f"KillMessage(data={self.payload})"


class RequestMessage(ReceivePacket):
    @classmethod
    def create_message(cls):
        return super(ReceivePacket, cls).create_receive_packet(
            int.from_bytes(constants["CAN_ID"], "big"),
            constants["REQUEST"]["KILL_STATE_REQUEST"],
        )

    def __str__(self):
        return f"RequestMessage(data={self.payload})"
