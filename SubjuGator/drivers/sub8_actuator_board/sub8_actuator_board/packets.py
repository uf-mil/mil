from dataclasses import dataclass

from mil_usb_to_can import Packet


@dataclass
class ActuatorSetPacket(Packet, msg_id=0x03, subclass_id=0x00, payload_format="BB"):
    address: int
    open: bool


@dataclass
class ActuatorPollRequest(Packet, msg_id=0x03, subclass_id=0x01, payload_format=""):
    pass


@dataclass
class ActuatorPollResponse(Packet, msg_id=0x03, subclass_id=0x02, payload_format=""):
    values: int
