#!/usr/bin/python
import struct


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
            return None
        if eof != cls.EOF:
            return None
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
    def read_packet(cls, ser, payload_length):
        '''
        Read a packet with a known size from a serial device
        @param ser: a instance of serial.Serial (or simulated version) to read from
        @param payload_length: the length of the payload of this packet you are expecting, in bytes
        @return a Packet object of the read packet or None if it failed
        '''
        packet_length = payload_length + 2
        data = ser.read(packet_length)
        if len(data) != packet_length:
            return None
        return cls.from_bytes(data)


class ReceivePacket(Packet):
    @property
    def data(self):
        return self.payload[:-1]

    @classmethod
    def create_receive_packet(cls, data):
        '''
        Creates a command packet to request data from a CAN device
        @param filter_id: the CAN device ID to request data from
        @param receive_length: the number of bytes to request
        '''
        checksum = cls.SOF + cls.EOF
        for byte in data:
            checksum += ord(byte)
        checksum %= 16
        data = data + chr(checksum)
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
            raise Exception('invalid checksum')
        payload = cls.unpack_payload(data)
        return cls(payload)

    @classmethod
    def read_packet(cls, ser, data_length):
        return super(ReceivePacket, cls).read_packet(ser, data_length + 1)


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
        payload = struct.pack('BB{}s'.format(len(data)), length_byte, filter_id, data)
        return cls(payload)

    @classmethod
    def create_send_packet(cls, filter_id, data):
        '''
        Creates a command packet to send data to a CAN device from the motherboard
        @param filter_id: the CAN device ID to send data to
        @param data: the data payload as string/bytes
        '''
        length_byte = len(data) - 1
        return cls.create_command_packet(length_byte, filter_id, data)

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
            raise Exception('bla')
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
