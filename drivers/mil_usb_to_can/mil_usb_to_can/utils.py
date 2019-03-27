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
    def from_bytes(cls, data):
        '''
        Parses a packet from a bytes string into a Packet instance
        @param data: bytes/string containing exactly one packet
        @return a Packet instance of succesfully parsed or None if failed
        TODO: raise exceptions on invalid SOF/EOF
        '''
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
        if self.is_receive:
            return self.length_byte & 127

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
        length_byte = len(data)
        return cls.create_command_packet(length_byte, filter_id, data)

    @classmethod
    def create_receive_packet(cls, filter_id, receive_length):
        '''
        Creates a command packet to request data from a CAN device
        @param filter_id: the CAN device ID to request data from
        @param receive_length: the number of bytes to request
        '''
        length_byte = receive_length | 128
        return cls.create_command_packet(length_byte, filter_id)

    def __str__(self):
        if self.is_receive:
            return 'CommandPacket(filter_id={}, is_receive=True, receive_length={})'.format(self.filter_id, self.length)
        else:
            return 'CommandPacket(filter_id={}, is_receive=False, data={})'.format(self.filter_id, self.data)
