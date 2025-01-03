from dataclasses import dataclass
from enum import Enum

from electrical_protocol import Packet


@dataclass
class HeartbeatReceivePacket(
    Packet,
    class_id=0x20,
    subclass_id=0x00,
    payload_format="<i",
):
    """
    Heartbeat packet sent from the drone.

    Attributes:
        status (int): The status of the drone (1=stowed, 2=deloyed, 3=faulted)
    """

    status: int


@dataclass
class HeartbeatSetPacket(Packet, class_id=0x20, subclass_id=0x01, payload_format=""):
    """
    Heartbeat packet sent from the boat.
    """


@dataclass
class GPSDronePacket(Packet, class_id=0x20, subclass_id=0x02, payload_format="<fff"):
    """
    GPS location of drone packet.

    Attributes:
        lat (float): The latitude of the drone.
        lon (float): The longitude of the drone.
        alt (float): The altitude of the drone.
    """

    lat: float
    lon: float
    alt: float


@dataclass
class EStopPacket(Packet, class_id=0x20, subclass_id=0x03, payload_format=""):
    """
    Emergency stop drone packet.
    """


@dataclass
class StartPacket(Packet, class_id=0x20, subclass_id=0x04, payload_format="<20s"):
    """
    Start drone mission packet.

    Attributes:
        name (str): The name of the mission to run on the drone. Limited to 20 characters.
    """

    name: str


@dataclass
class StopPacket(Packet, class_id=0x20, subclass_id=0x05, payload_format=""):
    """
    Stop drone and return packet.
    """


class Color(Enum):
    """
    Enum to represent the color of a target.
    """

    BLUE = "b"
    GREEN = "g"
    RED = "r"


class Logo(Enum):
    """
    Whether the drone detected the "R" logo pad, or the "N" logo pad.
    """

    R_LOGO = "R"
    N_LOGO = "N"


@dataclass
class TinPacket(Packet, class_id=0x20, subclass_id=0x07, payload_format="<ic"):
    """
    Tin packet and status of tin

    Attributes:
        status(int): status of drone for the tin
        color(Color): color of tin
    """

    status: int
    color: Color


@dataclass
class TargetPacket(Packet, class_id=0x20, subclass_id=0x06, payload_format="<ffc"):
    """
    GPS of drone-identified target packet.

    Attributes:
        lat (float): The latitude of the target.
        lon (float): The longitude of the target.
        logo (Logo): The logo of the target.
    """

    lat: float
    lon: float
    logo: Logo
