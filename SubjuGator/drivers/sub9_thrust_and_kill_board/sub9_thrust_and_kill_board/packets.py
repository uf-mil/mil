from dataclasses import dataclass
from enum import IntEnum

from electrical_protocol import Packet


@dataclass
class HeartbeatSetPacket(Packet, class_id=0x02, subclass_id=0x00, payload_format=""):
    """
    Heartbeat packet sent by the motherboard to the thrust/kill board.
    """


@dataclass
class HeartbeatReceivePacket(
    Packet,
    class_id=0x02,
    subclass_id=0x01,
    payload_format="",
):
    """
    Heartbeat packet sent by the thrust/kill board to the motherboard.
    """


class ThrusterId(IntEnum):
    """
    Enum to represent the thruster ID.
    """

    #: Front left horizontal thruster.
    FLH = 0
    #: Front right horizontal thruster.
    FRH = 1
    #: Front left vertical thruster.
    FLV = 2
    #: Front right vertical thruster.
    FRV = 3
    #: Back left horizontal thruster.
    BLH = 4
    #: Back right horizontal thruster.
    BRH = 5
    #: Back left vertical thruster.
    BLV = 6
    #: Back right vertical thruster.
    BRV = 7


@dataclass
class ThrustSetPacket(Packet, class_id=0x02, subclass_id=0x02, payload_format="<Bf"):
    """
    Packet to set the speed of a specific thruster.

    Attributes:
        thruster_id (ThrusterId): The ID of the thruster to set the speed of. The ID of
            the thruster corresponds to a specific thruster:
        speed (float): The speed to set the thruster to.
    """

    thruster_id: ThrusterId
    speed: float


class KillStatus(IntEnum):
    """
    Enum to represent the purpose behind a kill.
    """

    #: A software user manually requested a kill on the sub.
    SOFTWARE_REQUESTED = 0
    #: The board reported that it stopped hearing the heartbeat from the motherboard.
    MOBO_HEARTBEAT_LOST = 1
    #: The motherboard reported that it stopped hearing the heartbeat from the board.
    BOARD_HEARTBEAT_LOST = 2
    #: The physical kill switch was pulled, requesting an immediate kill.
    KILL_SWITCH = 3
    #: The battery is too low to continue operation.
    BATTERY_LOW = 4


@dataclass
class KillSetPacket(Packet, class_id=0x02, subclass_id=0x03, payload_format="<BB"):
    """
    Packet sent by the motherboard to set/unset the kill.

    Attributes:
        set (bool): Whether the kill is set or unset.
        status (KillStatus): The reason for the kill.
    """

    set: bool
    status: KillStatus


@dataclass
class KillReceivePacket(Packet, class_id=0x02, subclass_id=0x04, payload_format="<BB"):
    """
    Packet sent by the motherboard to set/unset the kill.

    Attributes:
        set (bool): Whether the kill is set or unset.
        status (KillStatus): The reason for the kill.
    """

    set: bool
    status: KillStatus
