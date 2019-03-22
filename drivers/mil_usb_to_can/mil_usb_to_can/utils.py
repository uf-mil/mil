#!/usr/bin/python
import struct


class Packet(object):
    '''
    Represents a packet to or from the CAN to USB board
    '''
    SOF = 0xC0
    EOF = 0xC1

    def __init__(self, payload):
        self.payload = payload

    def to_bytes(self):
        return struct.pack('B{}sB'.format(len(self.payload)), self.SOF, self.payload, self.EOF)

    @classmethod
    def from_bytes(cls, data):
        payload_len = len(data) - 2
        if payload_len < 1:
            return None
        sof, payload, eof = struct.unpack('B{}sB'.format(payload_len), data)
        if sof != cls.SOF:
            return None
        if eof != cls.EOF:
            return None
        return cls(payload)

    def __str__(self):
        return 'Packet(payload={})'.format(self.payload)

    @classmethod
    def read_packet(cls, ser, payload_length):
        packet_length = payload_length + 2
        data = ser.read(packet_length)
        if len(data) != packet_length:
            return None
        return cls.from_bytes(data)


class CommandPacket(Packet):
    '''
    Represents a packet to the CAN board from the motherboard.
    This packet can either request data from a device or send data to a device
    '''
    @property
    def length_byte(self):
        return struct.unpack('B', self.payload[0])[0]

    @property
    def is_receive(self):
        return bool(self.length_byte & 128)

    @property
    def length(self):
        if self.is_receive:
            return self.length_byte & 127

    @property
    def filter_id(self):
        return struct.unpack('B', self.payload[1])[0]

    @property
    def data(self):
        return self.payload[2:]

    @classmethod
    def create_command_packet(cls, length_byte, filter_id, data=''):
        payload = struct.pack('BB{}s'.format(len(data)), length_byte, filter_id, data)
        return cls(payload)

    @classmethod
    def create_send_packet(cls, filter_id, data):
        length_byte = len(data)
        return cls.create_command_packet(length_byte, filter_id, data)

    @classmethod
    def create_receive_packet(cls, filter_id, receive_length):
        length_byte = receive_length | 128
        return cls.create_command_packet(length_byte, filter_id)

    def __str__(self):
        if self.is_receive:
            return 'CommandPacket(filter_id={}, is_receive=True, receive_length={})'.format(self.filter_id, self.length)
        else:
            return 'CommandPacket(filter_id={}, is_receive=False, data={})'.format(self.filter_id, self.data)
