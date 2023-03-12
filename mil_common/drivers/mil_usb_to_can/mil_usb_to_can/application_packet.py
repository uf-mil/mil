#! /usr/bin/env python3
from __future__ import annotations

import struct
from typing import TypeVar

T = TypeVar("T", bound="ApplicationPacket")


class ApplicationPacketWrongIdentifierException(Exception):
    """
    Exception thrown when the identifier for a MIL application level CAN packet
    had a different identifier from what was expected.

    Inherits from :class:`Exception`.
    """

    def __init__(self, received: int, expected: int):
        """
        Attributes:
            received (int): A value representing what received found.
            expected (int): What the found value should have been.
        """
        super().__init__(f"Expected identified {expected}, got {received}")


class ApplicationPacket:
    """
    Represents an application-specific packet structure used by a CAN device. This
    class does not implement the entire packet that will be sent over the CAN bus;
    the packet only includes the identifier and payload in the packet.

    This class should be used to generate packets to send to the board. This packet
    does not handle communication with the board; this is instead handled by other
    packet classes.

    This class can be inherited from to implement packet structures for specific
    applications.

    .. code-block:: python

        class SendALetterMessage(ApplicationPacket):

            IDENTIFIER = 0xAA
            STRUCT_FORMAT = "B"

            def __init__(self, letter: str):
                self.letter = letter
                super().__init__(self.IDENTIFIER, struct.pack(self.STRUCT_FORMAT, ord(letter)))

            @classmethod
            def from_bytes(cls, data: bytes) -> SendALetterMessage:
                return cls(*struct.unpack(self.STRUCT_FORMAT, data))

    .. container:: operations

        .. describe:: bytes(x)

            Assembles the packet into a form suitable for sending through a data
            stream. Packs :attr:`~.identifier` and :attr:`~.payload` into a single
            :class:`bytes` object.

    Attributes:
        identifier (int): The identifier for the packet. Allowed range is between 0
            and 255.
        payload (bytes): The payload of bytes to be sent in the packet.
    """

    def __init__(self, identifier: int, payload: bytes):
        self.identifier = identifier
        self.payload = payload

    def __bytes__(self) -> bytes:
        """
        Packs the packet into a series of bytes using :meth:`struct.Struct.pack`.
        The identifier is packed as an unsigned integer, while the payload of bytes
        is packed as a sequence of bytes equal to the length of the payload.

        Returns:
            bytes: The packed bytes.
        """
        return struct.pack(f"B{len(self.payload)}s", self.identifier, self.payload)

    @classmethod
    def from_bytes(
        cls: type[T], data: bytes, expected_identifier: int | None = None
    ) -> T:
        """
        Unpacks a series of packed bytes representing an application packet using
        :meth:`struct.Struct.unpack`, which produces the packet identifier and array of data.
        These values are then used to produce the new instance of the class.

        Args:
            data (bytes): The packed packet.
            expected_identifier (Optional[int]): The identifier that is expected to
              result from the packet. If ``None``, then the identifier is not verified.
              If this value is passed and the identifiers do not match, then the below
              error is thrown.

        Raises:
            ApplicationPacketWrongIdentifierException: If the ``expected_identifier``
              does not match the identifier found in the packet, then this is raised.

        Returns:
            ApplicationPacket: The data represented as an application packet.
        """
        payload_len = len(data) - 1
        packet = cls(*struct.unpack(f"B{payload_len}s", data))
        if expected_identifier is not None and expected_identifier != packet.identifier:
            raise ApplicationPacketWrongIdentifierException(
                packet.identifier, expected_identifier
            )
        return packet

    def __repr__(self):
        return "{}(identifier={}, payload={})".format(
            self.__class__.__name__, self.identifier, self.payload
        )

    def __eq__(self, other):
        if not isinstance(other, ApplicationPacket):
            raise NotImplementedError
        return self.identifier == other.identifier and self.payload == other.payload
