from dataclasses import dataclass
from enum import IntEnum

from mil_usb_to_can.sub9 import Packet


class ActuatorPacketId(IntEnum):
    """
    Enumerator representing each controllable actuator.
    """

    #: The gripper actuator.
    GRIPPER = 0
    #: The torpedo launcher actuator.
    TORPEDO_LAUNCHER = 1
    #: The ball drop actuator. Only one actuator is used for both balls.
    BALL_DROP = 2


@dataclass
class ActuatorSetPacket(Packet, msg_id=0x03, subclass_id=0x00, payload_format="BB"):
    """
    Packet used by the actuator board to set a specific valve.

    Attributes:
        address (ActuatorPacketId): The actuator ID to set.
        open (bool): Whether to open the specified actuator. ``True`` requests opening,
            ``False`` requests closing.
    """

    address: ActuatorPacketId
    open: bool


@dataclass
class ActuatorPollRequestPacket(
    Packet,
    msg_id=0x03,
    subclass_id=0x01,
    payload_format="",
):
    """
    Packet used by the actuator board to request the status of all valves.
    """


@dataclass
class ActuatorPollResponsePacket(
    Packet,
    msg_id=0x03,
    subclass_id=0x02,
    payload_format="B",
):
    """
    Packet used by the actuator board to return the status of all valves.

    Attributes:
        values (int): The statues of all actuators. Bits 0-3 represent the opened
            status of actuators 0-3.
    """

    values: int

    @property
    def gripper_opened(self) -> bool:
        """
        Whether the gripper is opened.
        """
        return bool(self.values & (0b0001 << ActuatorPacketId.GRIPPER))

    @property
    def torpedo_launcher_opened(self) -> bool:
        """
        Whether the torpedo launcher is opened.
        """
        return bool(self.values & (0b0001 << ActuatorPacketId.TORPEDO_LAUNCHER))

    @property
    def ball_drop_opened(self) -> bool:
        """
        Whether the ball drop is opened.
        """
        return bool(self.values & (0b0001 << ActuatorPacketId.BALL_DROP))
