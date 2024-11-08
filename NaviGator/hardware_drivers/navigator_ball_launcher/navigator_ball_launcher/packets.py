from dataclasses import dataclass

from electrical_protocol import Packet


@dataclass
class ReleaseBallPacket(Packet, class_id=0x11, subclass_id=0x00, payload_format=""):
    """
    Packet sent by the motherboard to the board to release a ball (aka, spin the servo
    such that one ball is released.)

    A valid response to this packet being sent would be :class:`~electrical_protocol.AckPacket`
    (if the ball was successfully released) or :class:`~electrical_protocol.NackPacket`
    (if there was an issue).
    """

    pass


@dataclass
class SetSpinPacket(Packet, class_id=0x11, subclass_id=0x01, payload_format="<B"):
    """
    Packet sent by the motherboard to the board to spin up the flywheel.

    A valid response to this packet being sent would be :class:`~electrical_protocol.AckPacket`
    (if the flywheel was successfully spun up) or :class:`~electrical_protocol.NackPacket`
    (if there was an issue).

    Attributes:
        spin_up (bool): A boolean used to represent whether the flywheel should
            be spun up (aka, sped up to a fast rpm) or whether it should be spun
            down (aka, slowed to zero rpm).
    """

    spin_up: bool
