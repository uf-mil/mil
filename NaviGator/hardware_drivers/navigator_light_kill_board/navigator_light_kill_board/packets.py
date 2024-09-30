from dataclasses import dataclass
from enum import Enum

from electrical_protocol import Packet


class KillStatus(Enum):
    MOBO_REQUESTED = 0
    RF_KILL = 1
    EMERGENCY_STOP = 2


@dataclass
class KillSetPacket(Packet, class_id=0x10, subclass_id=0x00, payload_format="<?B"):
    set: bool
    status: KillStatus


@dataclass
class KillReceivePacket(Packet, class_id=0x10, subclass_id=0x01, payload_format="<?B"):
    set: bool
    status: KillStatus


@dataclass
class SetMovementModePacket(
    Packet,
    class_id=0x10,
    subclass_id=0x02,
    payload_format="<?",
):
    autonomous: bool
