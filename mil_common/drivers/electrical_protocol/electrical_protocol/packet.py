from __future__ import annotations

import struct
from dataclasses import dataclass, fields
from enum import Enum
from functools import lru_cache
from typing import ClassVar, TypeVar, get_type_hints

SYNC_CHAR_1 = 0x37
SYNC_CHAR_2 = 0x01

_packet_registry: dict[int, dict[int, type[Packet]]] = {}


PacketSelf = TypeVar("PacketSelf", bound="Packet")


def hexify(data: bytes) -> str:
    return ":".join(f"{c:02x}" for c in data)


@lru_cache(maxsize=None)
def get_cache_hints(cls):
    return get_type_hints(cls)


class ChecksumException(OSError):
    """
    An invalid checksum appeared.
    """

    def __init__(
        self,
        packet: type[Packet],
        received: tuple[int, int],
        expected: tuple[int, int],
    ):
        """
        Attributes:
            packet (Type[:class:`~.Packet`]): The packet with the invalid checksum.
            received (Tuple[int, int]): The received Fletcher's checksum.
            expected (Tuple[int, int]): The expected Fletcher's checksum.
        """
        super().__init__(
            f"Invalid checksum in packet of type {packet.__qualname__}: received {received}, expected {expected}",
        )


@dataclass
class Packet:
    """
    Represents one packet that can be sent or received by a serial device.
    This class is able to handle packaging unique data
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
        class ExamplePacket(Packet, class_id = 0x02, subclass_id = 0x01, payload_format = "BHHf"):
            example_char: int
            example_short: int
            example_short_two: int
            example_float: float

    .. container:: operations

        .. describe:: bytes(x)

            Returns a :class:`bytes` object representing the data of the packet
            in the specified packet format.

    Arguments:
        class_id (int): The message ID. Can be between 0 and 255.
        subclass_id (int): The message subclass ID. Can be between 0 and 255.
        payload_format (str): The format for the payload. This determines how
            the individual payload is assembled. Each character in the format
            string represents the position of one class variable. The class variables
            are assembled in the order they are defined in.
    """

    class_id: ClassVar[int]
    subclass_id: ClassVar[int]
    payload_format: ClassVar[str] = ""

    def __init_subclass__(
        cls,
        class_id: int,
        subclass_id: int,
        payload_format: str = "",
    ):
        cls.class_id = class_id
        cls.subclass_id = subclass_id
        cls.payload_format = payload_format
        packets = [p for mid in _packet_registry.values() for p in mid.values()]
        for packet in packets:
            if packet.class_id == class_id and packet.subclass_id == subclass_id:
                raise ValueError(
                    f"Cannot reuse class_id 0x{class_id:0x} and subclass_id 0x{subclass_id}, already used by {packet.__qualname__}",
                )
        _packet_registry.setdefault(class_id, {})[subclass_id] = cls

    def __post_init__(self):
        for name, field_type in get_cache_hints(self.__class__).items():
            if (
                name
                not in [
                    "class_id",
                    "subclass_id",
                    "payload_format",
                ]
                and not isinstance(self.__dict__[name], field_type)
                and issubclass(field_type, Enum)
            ):
                setattr(self, name, field_type(self.__dict__[name]))
        if self.payload_format and not self.payload_format.startswith(
            ("<", ">", "=", "!"),
        ):
            raise ValueError(
                "The payload format does not start with a standard size character: ('<', '>', '!', '=').",
            )
        available_chars: dict[type, list[str]] = {
            bool: ["c", "b", "B", "?"],
            int: ["b", "B", "h", "H", "i", "I", "l", "L", "q", "Q"],
            float: ["f", "d"],
        }
        stripped_format = self.payload_format.lstrip("<>=!@")
        for i, field in enumerate(fields(self)):
            if field.type not in available_chars:
                continue
            chars = available_chars[field.type]
            if i >= len(stripped_format):
                raise ValueError(
                    f"The payload format for the packet is too short to support all dataclass fields; expected: {len(fields(self))}, found: {len(self.payload_format)}.",
                )
            represented_char = stripped_format[i]
            if represented_char not in chars:
                raise ValueError(
                    f"The type of {field.name} in the payload format is '{represented_char}', which does not correspond to its dataclass type of {field.type}.",
                )

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
            f"<BBBBH{len(payload)}s",
            SYNC_CHAR_1,
            SYNC_CHAR_2,
            self.class_id,
            self.subclass_id,
            len(payload),
            payload,
        )
        checksum = self._calculate_checksum(data[2:])
        return data + struct.pack("<BB", *checksum)

    def __len__(self) -> int:
        return self.__class__._expected_len()

    @classmethod
    def _expected_len(cls) -> int:
        # We cannot use one calcsize since payload_format should start with a standard size character
        return struct.calcsize("<BBBBHBB") + struct.calcsize(cls.payload_format)

    @classmethod
    def from_bytes(
        cls: type[PacketSelf],
        packed: bytes,
        trim: bool = True,
    ) -> PacketSelf:
        """
        Constructs a packet from a packed packet in a :class:`bytes` object.
        If a packet is found with the corresponding message and subclass ID,
        then an instance (or subclass) of that packet class will be returned.

        Arguments:
            packed (bytes): The packed packet to unpack.
            trim (bool): If True, only the required number of bytes will be used
                to construct the packet. Otherwise, the entire packet will be used.
                Default: `True`.

        Raises:
            ChecksumException: The checksum is invalid.
            LookupError: No packet with the specified class and subclass IDs exist.

        Returns:
            An instance of the appropriate packet subclass.
        """
        class_id = packed[2]
        subclass_id = packed[3]
        if class_id in _packet_registry and subclass_id in _packet_registry[class_id]:
            subclass = _packet_registry[class_id][subclass_id]
            if trim:
                packed = packed[: subclass._expected_len()]
            if len(packed) < subclass._expected_len():
                raise ValueError(
                    f"Packet is too short to be a valid packet. (provided len: {len(packed)}, expected len: {cls._expected_len()})",
                )
            payload = packed[6:-2]
            if struct.unpack("<BB", packed[-2:]) != cls._calculate_checksum(
                packed[2:-2],
            ):
                raise ChecksumException(
                    subclass,
                    struct.unpack("<BB", packed[-2:]),
                    cls._calculate_checksum(packed[2:-2]),
                )
            unpacked = struct.unpack(subclass.payload_format, payload)
            packet = subclass(*unpacked)
            if not isinstance(packet, cls):
                raise RuntimeError(
                    f"Attempted to resolve packet of type {cls.__qualname__}, but found {packet.__class__.__qualname__} for bytes: {hexify(packed)}",
                )
            return packet
        raise LookupError(
            f"Attempted to reconstruct packet with class_id 0x{class_id:02x} and subclass_id 0x{subclass_id:02x}, but no packet with IDs was found.",
        )


@dataclass
class AckPacket(Packet, class_id=0x00, subclass_id=0x01, payload_format=""):
    """
    Common acknowledgment packet.
    """


@dataclass
class NackPacket(Packet, class_id=0x00, subclass_id=0x00, payload_format=""):
    """
    Common not-acknowledged packet.
    """
