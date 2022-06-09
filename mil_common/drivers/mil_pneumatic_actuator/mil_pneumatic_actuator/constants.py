#!/usr/bin/env python3
import struct
from typing import Tuple


class Constants:
    """
    Constant codes and functions used in both the driver and the simulated board.

    Attributes:
        OPEN_REQUEST_BASE (int): A constant representing a request to the board to open a valve.
            Currently set to ``0x20``.
        OPEN_RESPONSE (int): A constant representing a response from the board when the open
            request is sent. Currently set to ``0x01``.
        CLOSE_REQUEST_BASE (int): A constant representing a request to the board to close a valve.
            Currently set to ``0x30``.
        CLOSE_RESPONSE (int): A constant representing a response from the board when the close
            request is sent. Currently set to ``0x00``.
        READ_REQUEST_BASE (int): A constant representing a request to the board to read
            the value of a valve. Currently set to ``0x40``.
        PING_REQUEST (int): A constant representing a ping request. Currently set to ``0x10``.
        PING_RESPONSE (int): A constant representing a response from the board when the ping
            request is sent. Currently set to ``0x11``.
        CHECKSUM_CODE (int): The checksum code used by the board. Currently sent to ``0xFF``.
    """
    OPEN_REQUEST_BASE = 0x20
    OPEN_RESPONSE = 0x01
    CLOSE_REQUEST_BASE = 0x30
    CLOSE_RESPONSE = 0x00
    READ_REQUEST_BASE = 0x40
    PING_REQUEST = 0x10
    PING_RESPONSE = 0x11
    CHECKSUM_CODE = 0xFF

    @classmethod
    def create_checksum(cls, byte: int) -> int:
        """
        Creates a checksum given a piece of data. The checksum is calculated through
        the XOR operation.

        Args:
            byte (int): The data to compute the checksum on.

        Returns:
            int: The checksum.
        """
        return byte ^ cls.CHECKSUM_CODE

    @classmethod
    def verify_checksum(cls, byte: int, checksum: int) -> bool:
        """
        Verifies that two checksums are the same.

        Args:
            byte (int): The data to verify the constructed checksum of.
            checksum (int): The checksum to validate the constructed checksum against.

        Returns:
            bool: Whether the two checksums are the same.
        """
        return checksum == cls.create_checksum(byte)

    @classmethod
    def serialize_packet(cls, byte: int) -> bytes:
        """
        Serializes a piece of data (an integer) into a bytes object.

        Returns:
            bytes: The serialized packet.
        """
        return struct.pack("BB", byte, cls.create_checksum(byte))

    @classmethod
    def deserialize_packet(cls, data: bytes) -> Tuple[int, int]:
        """
        Deserializes a piece of data (a bytes object) into a tuple of two numbers.
        The first number is the data carried by the packet, and the second is the checksum
        computed by the board.

        Returns:
            bytes: The serialized packet.
        """
        return struct.unpack("BB", data)
