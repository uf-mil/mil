import struct


class ApplicationPacketWrongIdentifierException(Exception):
    '''
    Exception thrown when the identifer for a MIL appliction level CAN packet
    had a different identifer from what was expected
    '''
    def __init__(self, was, should_be):
        super(ApplicationPacketWrongIdentifierException, self).__init__(
            "Expected identified '{}', go '{}'".format(should_be, was))


class ApplicationPacket(object):
    '''
    '''
    def __init__(self, identifier, payload):
        self.identifier = identifier
        self.payload = payload

    def to_bytes(self):
        return struct.pack('B{}s'.format(len(self.payload)), self.identifier, self.payload)

    @classmethod
    def from_bytes(cls, data):
        payload_len = len(data) - 1
        return cls(*struct.unpack('B{}s'.format(payload_len), data))

    def __str__(self):
        return 'MilApplicationPacket(identifer={}, payload={})'.format(self.identifier, self.payload)
