#! /usr/bin/env python3
from __future__ import annotations

import struct
from typing import Any, Optional


class ApplicationPacketWrongIdentifierException(Exception):
    """
    Exception thrown when the identifer for a MIL appliction level CAN packet
    had a different identifer from what was expected.

    Inherits from :class:`Exception`.
    """

    def __init__(self, was: Any, should_be: Any):
        """
        Attributes:
            was (Any): A value representing what was found.
            should_be (Any): What the found value should have been.
        """
        super(ApplicationPacketWrongIdentifierException, self).__init__(
            "Expected identified '{}', got '{}'".format(should_be, was)
        )


class ApplicationPacket:
    """
    One packet of data that is used to communicate with CAN devices.

    Attributes:
        identifier (int): The identifier for the packet.
        payload (bytes): The payload of bytes to be sent in the packet.
    """

    def __init__(self, identifier: int, payload: bytes):
        self.identifier = identifier
        self.payload = payload

    def to_bytes(self) -> bytes:
        """
        Packs the packet into a series of bytes using :meth:`struct.Struct.pack`. The identifier
        is packed as an unsigned integer, while the payload of bytes is packed as
        a sequence of bytes equal to the length of the payload.

        Returns:
            bytes: The packed bytes.
        """
        return struct.pack(
            "B{}s".format(len(self.payload)), self.identifier, self.payload
        )

    @classmethod
    def from_bytes(
        cls, data: bytes, expected_identifier: Optional[int] = None
    ) -> ApplicationPacket:
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
            ApplicationPacketWrongIdentifierException: If the ```expected_identifier``
              does not match the identifier found in the packet, then this is raised.

        Returns:
            ApplicationPacket: The data represented as an application packet.
        """
        payload_len = len(data) - 1
        packet = cls(*struct.unpack("B{}s".format(payload_len), data))
        if expected_identifier is not None and expected_identifier != packet.identifier:
            raise ApplicationPacketWrongIdentifierException(
                packet.identifier, expected_identifier
            )
        return packet

    def __str__(self):
        return "MilApplicationPacket(identifer={}, payload={})".format(
            self.identifier, self.payload
        )
