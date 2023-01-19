from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import ClassVar


@dataclass
class Packet:
    """
    Represents one packet sent or received by a device handle communicating to/from
    the USB to CAN board. This class is able to handle packaging unique data
    values into a :class:`bytes` object for sending over a data stream.

    This class should be overridden to implement unique packet payloads. Note
    that this class supports three subclass arguments to assign unique message IDs,
    subclass IDs, and payload formats.

    .. code-block:: python

        class ExamplePacket(Packet, msg_id = 0x02, subclass_id = 0x01, payload_format = "BHHf"):
            example_char: int
            example_short: int
            example_short_two: int
            example_float: float

    .. container:: operations

        .. describe:: bytes(x)

            Returns a :class:`bytes` object representing the data of the packet
            in the specified packet format.

    Arguments:
        msg_id (int): The message ID. Can be between 0 and 255.
        subclass_id (int): The message subclass ID. Can be between 0 and 255.
        payload_format (str): The format for the payload. This determines how
            the individual payload is assembled. Each character in the format
            string represents the position of one class variable. The class variables
            are assembled in the order they are defined in.
    """

    msg_id: ClassVar[int]
    subclass_id: ClassVar[int]
    payload_format: ClassVar[str]

    def __init_subclass__(cls, msg_id: int, subclass_id: int, payload_format: str = ""):
        cls.msg_id = msg_id
        cls.subclass_id = subclass_id
        cls.payload_format = payload_format

    def __bytes__(self):
        payload = struct.pack(self.payload_format, *self.__dict__.values())
        return struct.pack(
            f"BBBBH{len(payload)}s",
            0x37,
            0x01,
            self.msg_id,
            self.subclass_id,
            len(payload),
            payload,
        )


@dataclass
class AckPacket(Packet, msg_id=0x00, subclass_id=0x01, payload_format=""):
    """
    Common acknowledgment packet. Should only be found in response operations.
    """

    pass


@dataclass
class NackPacket(Packet, msg_id=0x00, subclass_id=0x00, payload_format=""):
    """
    Common not-acknowledged packet. Should only be found in response operations.
    """

    pass
