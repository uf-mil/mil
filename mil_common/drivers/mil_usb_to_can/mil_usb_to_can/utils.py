#!/usr/bin/python3
from __future__ import annotations

import struct
import serial
from typing import Optional, Union


class USB2CANException(Exception):
    """
    Base class for exception in USB2CAN board functionality. Inherits from :class:`Exception`.
    """
    pass


class ChecksumException(USB2CANException):
    """
    Exception thrown when the checksum between motherboard and CANtoUSB board is invalid.
    Inherits from :class:`USB2CANException`.
    """
    def __init__(self, calculated, expected):
        super(ChecksumException, self).__init__(
            "Checksum was calculated as {} but reported as {}".format(
                calculated, expected
            )
        )


class PayloadTooLargeException(USB2CANException):
    """
    Exception thrown when payload of data sent/received from CAN2USB is too large.
    Inherits from :class:`USB2CANException`.
    """
    def __init__(self, length):
        super(PayloadTooLargeException, self).__init__(
            "Payload is {} bytes, which is greater than the maximum of 8".format(length)
        )


class InvalidFlagException(USB2CANException):
    """
    Exception thrown when a constant flag in the CAN2USB protocol is invalid. Inherits
    from :class:`USB2CANException`.
    """
    def __init__(self, description, expected, was):
        super(InvalidFlagException, self).__init__(
            "{} flag should be {} but was {}".format(description, expected, was)
        )


class InvalidStartFlagException(InvalidFlagException):
    """
    Exception thrown when the SOF flag is invalid. Inherits from :class:`InvalidFlagException`.
    """
    def __init__(self, was):
        super(InvalidStartFlagException, self).__init__("SOF", Packet.SOF, was)


class InvalidEndFlagException(InvalidFlagException):
    """
    Exception thrown when the EOF flag is invalid. Inherits from :class:`InvalidFlagException`.
    """
    def __init__(self, was):
        super(InvalidEndFlagException, self).__init__("EOF", Packet.EOF, was)


class Packet:
    """
    Represents a packet to or from the CAN to USB board.

    Attributes:
        SOF (int): Flag used to mark the beginning of each packet. Equal to `0xC0`.
        EOF (int): Flag used to mark the beginning of each packet. Equal to `0xC1`.
    """
    # Flag used to mark beginning of each packet
    SOF = 0xC0
    # Flag used to mark end of each packet
    EOF = 0xC1

    def __init__(self, payload: bytes):
        """
        Create a Packet object with the specified payload

        Args:
            payload (bytes): The bytes to send in the packet.
        """
        self.payload = payload

    def to_bytes(self) -> bytes:
        """
        Returns the binary represnetation of this packet to be sent accross the CAN network.
        Uses :meth:`struct.Struct.pack` to pack the payload between the :attr:`.SOF` and
        :attr:`.EOF`.

        Returns:
            The packed bytes.
        """
        return struct.pack(
            "B{}sB".format(len(self.payload)), self.SOF, self.payload, self.EOF
        )

    @classmethod
    def unpack_payload(cls, data: bytes) -> Optional[bytes]:
        """
        Attempts to obtain the raw data from a packed payload.

        Raises:
            InvalidStartFlagException: The start flag (first unsigned integer) of
              the payload is invalid.
            InvalidEndFlagException: The end flag (last unsigned integer) of the payload
              is invalid.

        Returns:
            Optional[bytes]: The raw data inside the packet payload. If the data
              has zero length, then ``None`` is returned.
        """
        payload_len = len(data) - 2
        if payload_len < 1:
            return None
        sof, payload, eof = struct.unpack("B{}sB".format(payload_len), data)
        if sof != cls.SOF:
            raise InvalidStartFlagException(sof)
        if eof != cls.EOF:
            raise InvalidEndFlagException(eof)
        return payload

    @classmethod
    def from_bytes(cls, data: bytes) -> Optional[Packet]:
        """
        Parses a packet from a bytes string into a Packet instance.

        Args:
        	data (bytes): The data to put into a packet.

        Returns:
            Optional[Packet]: The packet (if one can be created), otherwise ``None``.
        """
        payload = cls.unpack_payload(data)
        if payload is None:
            return None
        return cls(payload)

    def __str__(self):
        return "Packet(payload={})".format(self.payload)

    @classmethod
    def read_packet(cls, ser: Union[serial.Serial, 'SimulatedUSBtoCAN']) -> Optional[Packet]:
        """
        Read a packet with a known size from a serial device

        Args:
        	ser (Union[serial.Serial, SimulatedUSBtoCAN]): A instance of a serial device
              to read from.

        Raises:
            InvalidStartFlagException: The start flag of the packet read was invalid.
            InvalidEndFlagException: The end flag of the packet read was invalid.

        Returns:
            Optional[Packet]: If found, read a packet from the serial device. Otherwise,
              return ``None``.
        """
        # Read until SOF is encourntered incase buffer contains the end of a previous packet
        sof = None
        for _ in range(10):
            sof = ser.read(1)
            if sof is None or len(sof) == 0:
                return None
            if ord(sof) == cls.SOF:
                break
        if ord(sof) != cls.SOF:
            raise InvalidStartFlagException(ord(sof))
        data = sof
        eof = None
        for _ in range(10):
            eof = ser.read(1)
            if eof is None or len(eof) == 0:
                return None
            data += eof
            if ord(eof) == cls.EOF:
                break
        if ord(eof) != cls.EOF:
            raise InvalidEndFlagException(ord(eof))
        # print hexify(data)
        return cls.from_bytes(data)


class ReceivePacket(Packet):
    @property
    def device(self) -> int:
        """
        The device ID associated with the packet.
        """
        return struct.unpack("B", self.payload[0])[0]

    @property
    def data(self) -> bytes:
        """
        The data inside the packet.
        """
        return self.payload[2:-1]

    @property
    def length(self):
        return struct.unpack("B", self.payload[1])[0]

    @classmethod
    def create_receive_packet(cls, device_id: int, payload: bytes) -> ReceivePacket:
        """
        Creates a command packet to request data from a CAN device.

        Args:
        	filter_id (int): The CAN device ID to request data from.
        	payload (bytes): The data to send in the packet.

        Returns:
            ReceivePacket: The packet to request from the CAN device.
        """
        if len(payload) > 8:
            raise PayloadTooLargeException(len(payload))
        checksum = device_id + len(payload) + cls.SOF + cls.EOF
        for byte in payload:
            checksum += ord(byte)
        checksum %= 16
        data = struct.pack(
            "BB{}sB".format(len(payload)), device_id, len(payload), payload, checksum
        )
        return cls(data)

    @classmethod
    def from_bytes(cls, data: bytes) -> ReceivePacket:
        """
        Creates a receive packet from packed bytes. This strips the checksum from 
        the bytes and then unpacks the data to gain the raw payload.

        Raises:
            ChecksumException: The checksum found in the data differs from that
            found in the data.

        Returns:
            ReceivePacket: The packet constructed from the packed bytes.
        """
        expected_checksum = 0
        for byte in data[:-2]:
            expected_checksum += ord(byte)
        expected_checksum += ord(data[-1])
        expected_checksum %= 16
        real_checksum = ord(data[-2])
        if real_checksum != expected_checksum:
            raise ChecksumException(expected_checksum, real_checksum)
        payload = cls.unpack_payload(data)
        return cls(payload)


def can_id(task_group, ecu_number):
    return (task_group & 240) + (ecu_number & 15)


class CommandPacket(Packet):
    """
    Represents a packet to the CAN board from the motherboard.
    This packet can either request data from a device or send data to a device.
    """
    @property
    def length_byte(self) -> int:
        """
        The first header byte which encodes the length and the receive flag.

        Returns:
            :class:`int`
        """
        return struct.unpack("B", self.payload[0])[0]

    @property
    def is_receive(self) -> bool:
        """
        True if this CommandPacket is requesting data.

        Returns:
            :class:`bool`
        """
        return bool(self.length_byte & 128)

    @property
    def length(self) -> int:
        """
        The number of bytes of data sent or requested.

        Returns:
            :class:`int`
        """
        return (self.length_byte & 7) + 1

    @property
    def filter_id(self) -> int:
        """
        An integer representing the CAN device ID specified by this packet.

        Returns:
            :class:`int`
        """
        return struct.unpack("B", self.payload[1])[0]

    @property
    def data(self) -> bytes:
        """
        The data to be sent (empty string for data request commands).

        Returns:
            :class:`bytes`
        """
        return self.payload[2:]

    @classmethod
    def create_command_packet(cls, length_byte: int, filter_id: int, data: bytes = b"") -> CommandPacket:
        """
        Creates a command packet.

        .. warning::

            This method should rarely be used. Instead, use :meth:`.create_send_packet`
            or :meth:`.create_receive_packet` instead.

        Args:
        	length_byte (int): The first header byte
        	filter_id (int): The second header byte
        	data (bytes): Optional data payload when this is a send command. Defaults
            to an empty byte string.
        """
        if len(data) > 8:
            raise PayloadTooLargeException(len(data))
        payload = struct.pack("BB{}s".format(len(data)), length_byte, filter_id, data)
        return cls(payload)

    @classmethod
    def create_send_packet(cls, data: bytes, can_id: int = 0) -> CommandPacket:
        """
        Creates a command packet to send data to the CAN bus from the motherboard.

        Args:
        	data (bytes): The data payload.

        Returns:
            CommandPacket: The packet responsible for sending information to the CAN bus
            from the motherboard.
        """
        length_byte = len(data) - 1
        return cls.create_command_packet(length_byte, can_id, data)

    @classmethod
    def create_request_packet(cls, filter_id: int, receive_length: int) -> CommandPacket:
        """
        Creates a command packet to request data from a CAN device.

        Args:
        	filter_id (int): The CAN device ID to request data from.
        	receive_length (int): The number of bytes to request.

        Returns:
            CommandPacket: The command packet responsibel for requesting data from
            a CAN device.
        """
        length_byte = (receive_length - 1) | 128
        return cls.create_command_packet(length_byte, filter_id)

    @staticmethod
    def calculate_checksum(data):
        checksum = 0
        for byte in data:
            checksum += ord(byte)
        return checksum % 16

    @classmethod
    def from_bytes(cls, data):
        checksum_expected = 0
        checksum_expected += ord(data[0])
        checksum_expected += ord(data[1]) & 135
        for byte in data[2:]:
            checksum_expected += ord(byte)
        checksum_expected %= 16
        checksum_real = (ord(data[1]) & 120) >> 3
        if checksum_expected != checksum_real:
            raise ChecksumException(checksum_expected, checksum_real)
        payload = cls.unpack_payload(data)
        if payload is None:
            return None
        return cls(payload)

    def to_bytes(self):
        data = super(CommandPacket, self).to_bytes()
        checksum = 0
        for byte in data:
            checksum += ord(byte)
        checksum %= 16
        header_byte = (checksum << 3) | ord(data[1])
        data = data[:1] + chr(header_byte) + data[2:]
        return data

    def __str__(self):
        if self.is_receive:
            return "CommandPacket(filter_id={}, is_receive=True, receive_length={})".format(
                self.filter_id, self.length
            )
        else:
            return "CommandPacket(filter_id={}, is_receive=False, data={})".format(
                self.filter_id, self.data
            )
