#!/usr/bin/python
import struct


class USB2CANException(Exception):
    '''
    Base class for exception in USB2CAN board functionality
    '''
    pass


class ChecksumException(USB2CANException):
    '''
    Exception thrown when the checksum between motherboard and CAN2USb board is invalid
    '''
    def __init__(self, calculated, expected):
        super(ChecksumException, self).__init__(
            'Checksum was calculated as {} but reported as {}'.format(calculated, expected))


class PayloadTooLargeException(USB2CANException):
    '''
    Exception thrown when payload of data sent/received from CAN2USB is too large
    '''
    def __init__(self, length):
        super(PayloadTooLargeException, self).__init__(
            'Payload is {} bytes, which is greater than the maximum of 8'.format(length))


class InvalidFlagException(USB2CANException):
    '''
    Exception thrown when a constant flag in the CAN2USB protocol is invalid
    '''
    def __init__(self, description, expected, was):
        super(InvalidFlagException, self).__init__(
            '{} flag should be {} but was {}'.format(description, expected, was))


class InvalidStartFlagException(InvalidFlagException):
    '''
    Exception thrown when the SOF flag is invalid
    '''
    def __init__(self, was):
        super(InvalidStartFlagException, self).__init__('SOF', Packet.SOF, was)


class InvalidEndFlagException(InvalidFlagException):
    '''
    Exception thrown when the EOF flag is invalid
    '''
    def __init__(self, was):
        super(InvalidEndFlagException, self).__init__('EOF', Packet.EOF, was)


class Packet(object):
    '''
    Represents a packet to or from the CAN to USB board
    '''
    # Flag used to mark beginning of each packet
    SOF = 0xC0
    # Flag used to mark end of each packet
    EOF = 0xC1

    def __init__(self, payload):
        '''
        Create a Packet object with the specified payload
        @param payload: bytes/string object
        '''
        self.payload = payload

    def to_bytes(self):
        '''
        Returns the binary represnetation of this packet to be sent accross the CAN network
        @return bytes/string object for this packet
        '''
        return struct.pack('B{}sB'.format(len(self.payload)), self.SOF, self.payload, self.EOF)

    @classmethod
    def unpack_payload(cls, data):
        payload_len = len(data) - 2
        if payload_len < 1:
            return None
        sof, payload, eof = struct.unpack('B{}sB'.format(payload_len), data)
        if sof != cls.SOF:
            raise InvalidStartFlagException(sof)
        if eof != cls.EOF:
            raise InvalidEndFlagException(eof)
        return payload

    @classmethod
    def from_bytes(cls, data):
        '''
        Parses a packet from a bytes string into a Packet instance
        @param data: bytes/string containing exactly one packet
        @return a Packet instance of succesfully parsed or None if failed
        TODO: raise exceptions on invalid SOF/EOF
        '''
        payload = cls.unpack_payload(data)
        if payload is None:
            return None
        return cls(payload)

    def __str__(self):
        return 'Packet(payload={})'.format(self.payload)

    @classmethod
    def read_packet(cls, ser):
        '''
        Read a packet with a known size from a serial device
        @param ser: a instance of serial.Serial (or simulated version) to read from
        @param payload_length: the length of the payload of this packet you are expecting, in bytes
        @return a Packet object of the read packet or None if it failed
        '''
        # Read until SOF is encourntered incase buffer contains the end of a previous packet
        sof = None
        for _ in xrange(10):
            sof = ser.read(1)
            if sof is None:
                return None
            if ord(sof) == cls.SOF:
                break
        if ord(sof) != cls.SOF:
            raise InvalidStartFlagException(ord(sof))
        data = sof
        eof = None
        for _ in xrange(10):
            eof = ser.read(1)
            if eof is None:
                return None
            data += eof
            if ord(eof) == cls.EOF:
                break
        if ord(eof) != cls.EOF:
            raise InvalidEndFlagException(ord(eof))
        return cls.from_bytes(data)


class ReceivePacket(Packet):
    @property
    def device(self):
        return struct.unpack('B', self.payload[0])[0]

    @property
    def data(self):
        return self.payload[1:-1]

    @classmethod
    def create_receive_packet(cls, device_id, payload):
        '''
        Creates a command packet to request data from a CAN device
        @param filter_id: the CAN device ID to request data from
        @param receive_length: the number of bytes to request
        '''
        if len(payload) > 8:
            raise PayloadTooLargeException(len(payload))
        checksum = device_id + cls.SOF + cls.EOF
        for byte in payload:
            checksum += ord(byte)
        checksum %= 16
        data = struct.pack('B{}sB'.format(len(payload)), device_id, payload, checksum)
        return cls(data)

    @classmethod
    def from_bytes(cls, data):
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
    '''
    Represents a packet to the CAN board from the motherboard.
    This packet can either request data from a device or send data to a device
    '''
    @property
    def length_byte(self):
        '''
        @return: the first header byte which encodes the length and the receive flag
        '''
        return struct.unpack('B', self.payload[0])[0]

    @property
    def is_receive(self):
        '''
        @return: True if this CommandPacket is requesting data
        '''
        return bool(self.length_byte & 128)

    @property
    def length(self):
        '''
        @return: the number of bytes of data sent or requested
        '''
        return (self.length_byte & 7) + 1

    @property
    def filter_id(self):
        '''
        @return: an integer representing the CAN device ID specified by this packet
        '''
        return struct.unpack('B', self.payload[1])[0]

    @property
    def data(self):
        '''
        @return: the data to be sent (empty string for data request commands)
        '''
        return self.payload[2:]

    @classmethod
    def create_command_packet(cls, length_byte, filter_id, data=''):
        '''
        Creates a command packet.
        Note: you should use create_send_packet or create_receive_packet instead.
        @param length_byte: the first header byte
        @param filter_id: the second header byte
        @param data: option data payload when this is a send command
        '''
        if len(data) > 8:
            raise PayloadTooLargeException(len(data))
        payload = struct.pack('BB{}s'.format(len(data)), length_byte, filter_id, data)
        return cls(payload)

    @classmethod
    def create_send_packet(cls, data, can_id=0):
        '''
        Creates a command packet to send data to the CAN bus from the motherboard
        @param data: the data payload as string/bytes
        '''
        length_byte = len(data) - 1
        return cls.create_command_packet(length_byte, can_id, data)

    @classmethod
    def create_request_packet(cls, filter_id, receive_length):
        '''
        Creates a command packet to request data from a CAN device
        @param filter_id: the CAN device ID to request data from
        @param receive_length: the number of bytes to request
        '''
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
            return 'CommandPacket(filter_id={}, is_receive=True, receive_length={})'.format(self.filter_id, self.length)
        else:
            return 'CommandPacket(filter_id={}, is_receive=False, data={})'.format(self.filter_id, self.data)
