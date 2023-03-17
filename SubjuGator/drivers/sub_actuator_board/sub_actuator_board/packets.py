from dataclasses import dataclass

from mil_usb_to_can.sub9 import Packet


@dataclass
class ActuatorSetPacket(Packet, msg_id=0x03, subclass_id=0x00, payload_format="BB"):
    """
    Packet used by the actuator board to set a specific valve.

    Attributes:
        address (int): The actuator ID to set.
        open (bool): Whether to open the specified actuator. ``True`` requests opening,
            ``False`` requests closing.
    """

    address: int
    open: bool


@dataclass
class ActuatorPollRequestPacket(
    Packet, msg_id=0x03, subclass_id=0x01, payload_format=""
):
    """
    Packet used by the actuator board to request the status of all valves.
    """

    pass


@dataclass
class ActuatorPollResponsePacket(
    Packet, msg_id=0x03, subclass_id=0x02, payload_format="B"
):
    """
    Packet used by the actuator board to return the status of all valves.

    Attributes:
        values (int): The statues of all actuators. Bits 0-3 represent the opened
            status of actuators 0-3.
    """

    values: int
