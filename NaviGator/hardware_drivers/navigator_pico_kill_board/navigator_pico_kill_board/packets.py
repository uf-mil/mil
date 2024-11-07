from dataclasses import dataclass
from enum import IntEnum

from electrical_protocol import Packet


class KillStatus(IntEnum):
    MOBO_REQUESTED = 0
    RF_KILL = 1
    EMERGENCY_STOP = 2


@dataclass
class KillSetPacket(Packet, class_id=0x10, subclass_id=0x0, payload_format="?B"):
    set: bool
    status: KillStatus


@dataclass
class KillRecievePacket(Packet, class_id=0x10, subclass_id=0x1, payload_format="?B"):
    set: bool
    status: KillStatus
