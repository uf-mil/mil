#!/usr/bin/env python
import struct


class Constants(object):
    '''
    Constant codes and functions used in both the driver and the simulated
    board.
    '''
    OPEN_REQUEST_BASE = 0x20
    OPEN_RESPONSE = 0x01
    CLOSE_REQUEST_BASE = 0x30
    CLOSE_RESPONSE = 0x00
    READ_REQUEST_BASE = 0x40
    PING_REQUEST = 0x10
    PING_RESPONSE = 0x11
    CHECKSUM_CODE = 0xFF

    @classmethod
    def create_checksum(cls, byte):
        return byte ^ cls.CHECKSUM_CODE

    @classmethod
    def verify_checksum(cls, byte, checksum):
        return checksum == cls.create_checksum(byte)

    @classmethod
    def serialize_packet(cls, byte):
        return struct.pack('BB', byte, cls.create_checksum(byte))

    @classmethod
    def deserialize_packet(cls, data):
        return struct.unpack('BB', data)
