from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import Enum
from functools import lru_cache
from typing import ClassVar, get_type_hints

SYNC_CHAR_1 = ord("3")
SYNC_CHAR_2 = ord("7")

_packet_registry: dict[int, dict[int, type[Packet]]] = {}


def hexify(data: bytes) -> str:
    return ":".join(f"{c:02x}" for c in data)


@lru_cache(maxsize=None)
def get_cache_hints(cls):
    return get_type_hints(cls)


class ChecksumException(OSError):
    def __init__(
        self,
        packet: type[Packet],
        received: tuple[int, int],
        expected: tuple[int, int],
    ):
        super().__init__(
            f"Invalid checksum in packet of type {packet.__qualname__}: received {received}, expected {expected}",
        )


@dataclass
class Packet:
    """
    Represents one packet sent or received by a device handle communicating to/from
    the USB to CAN board. This class is able to handle packaging unique data
    values into a :class:`bytes` object for sending over a data stream.

    This class should be overridden to implement unique packet payloads. Note
    that this class supports three subclass arguments to assign unique message IDs,
    subclass IDs, and payload formats. Note that all subclasses must be decorated
    with :meth:`dataclasses.dataclass`.

    If any class members are annotated with a subclass of :class:`enum.Enum`,
    the class will always make an attempt to convert the raw data value to an
    instance of the enum before constructing the rest of the values in the class.

    .. code-block:: python

        from dataclasses import dataclass

        @dataclass
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
        packets = [p for mid in _packet_registry.values() for p in mid.values()]
        for packet in packets:
            if packet.msg_id == msg_id and packet.subclass_id == subclass_id:
                raise ValueError(
                    f"Cannot reuse msg_id 0x{msg_id:0x} and subclass_id 0x{subclass_id}, already used by {packet.__qualname__}",
                )
        _packet_registry.setdefault(msg_id, {})[subclass_id] = cls

    def __post_init__(self):
        for name, field_type in get_cache_hints(self.__class__).items():
            if (
                name
                not in [
                    "msg_id",
                    "subclass_id",
                    "payload_format",
                ]
                and not isinstance(self.__dict__[name], field_type)
                and issubclass(field_type, Enum)
            ):
                setattr(self, name, field_type(self.__dict__[name]))

    @classmethod
    def _calculate_checksum(cls, data: bytes) -> tuple[int, int]:
        sum1, sum2 = 0, 0
        for byte in data:
            sum1 = (sum1 + byte) % 255
            sum2 = (sum2 + sum1) % 255
        return sum1, sum2

    def __bytes__(self):
        payload = struct.pack(self.payload_format, *self.__dict__.values())
        data = struct.pack(
            f"BBBBH{len(payload)}s",
            SYNC_CHAR_1,
            SYNC_CHAR_2,
            self.msg_id,
            self.subclass_id,
            len(payload),
            payload,
        )
        checksum = self._calculate_checksum(data[2:])
        return data + struct.pack("BB", *checksum)

    @classmethod
    def from_bytes(cls, packed: bytes) -> Packet:
        """
        Constructs a packet from a packed packet in a :class:`bytes` object.
        If a packet is found with the corresponding message and subclass ID,
        then an instance of that packet class will be returned, else :class:`Packet`
        will be returned.
        """
        msg_id = packed[2]
        subclass_id = packed[3]
        if msg_id in _packet_registry and subclass_id in _packet_registry[msg_id]:
            subclass = _packet_registry[msg_id][subclass_id]
            payload = packed[6:-2]
            if struct.unpack("BB", packed[-2:]) != cls._calculate_checksum(
                packed[2:-2],
            ):
                raise ChecksumException(
                    subclass,
                    struct.unpack("BB", packed[-2:]),
                    cls._calculate_checksum(packed[2:-2]),
                )
            unpacked = struct.unpack(subclass.payload_format, payload)
            return subclass(*unpacked)
        raise LookupError(
            f"Attempted to reconstruct packet with msg_id 0x{msg_id:02x} and subclass_id 0x{subclass_id:02x}, but no packet with IDs was found.",
        )


@dataclass
class AckPacket(Packet, msg_id=0x00, subclass_id=0x01, payload_format=""):
    """
    Common acknowledgment packet. Should only be found in response operations.
    """


@dataclass
class NackPacket(Packet, msg_id=0x00, subclass_id=0x00, payload_format=""):
    """
    Common not-acknowledged packet. Should only be found in response operations.
    """
