from dataclasses import dataclass

from mil_usb_to_can import Packet


@dataclass
class HeartbeatPacket(Packet, msg_id=0x01, subclass_id=0x00, payload_format=""):
    pass


@dataclass
class ThrustSetRequestPacket(
    Packet, msg_id=0x01, subclass_id=0x01, payload_format="Bf"
):
    """
    Packet to set the speed of a specific thruster.

    Attributes:
        thruster_id (int): The ID of the thruster to set the speed of. The ID of
            the thruster corresponds to a specific thruster:

            +--------+------+
            |  name  |  id  |
            +========+======+
            |  FLH   |  0   |
            +--------+------+
            |  FRH   |  1   |
            +--------+------+
            |  FLV   |  2   |
            +--------+------+
            |  FRV   |  3   |
            +--------+------+
            |  BLH   |  4   |
            +--------+------+
            |  BRH   |  5   |
            +--------+------+
            |  BLV   |  6   |
            +--------+------+
            |  BRV   |  7   |
            +--------+------+
        speed (float): The speed to set the thruster to.
    """

    thruster_id: int
    speed: float


@dataclass
class KillSetPacket(Packet, msg_id=0x01, subclass_id=0x02, payload_format=""):
    set: bool
