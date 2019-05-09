import struct
from mil_usb_to_can import ApplicationPacket


# CAN channel to send thrust messages to
THRUST_SEND_ID = 0x21
# CAN channel ot send kill messages to
KILL_SEND_ID = 0x11


class KillMessage(ApplicationPacket):
    '''
    Represents a packet sent to kill/thrust board which contains
    a command or response related to kill status
    '''
    IDENTIFIER = ord('K')
    COMMAND = 0x43
    RESPONSE = 0x52
    HARD = 0x48
    SOFT = 0x53
    ASSERTED = 0x41
    UNASSERTED = 0x55
    PADDING = 0x00

    @classmethod
    def create_kill_message(cls, command=False, hard=False, asserted=False):
        command_byte = cls.COMMAND if command else cls.RESPONSE
        hard_soft_byte = cls.HARD if hard else cls.SOFT
        assert_unassert_byte = cls.ASSERTED if asserted else cls.UNASSERTED
        payload = struct.pack('BBBB', command_byte, hard_soft_byte, assert_unassert_byte, cls.PADDING)
        return cls(cls.IDENTIFIER, payload)

    @property
    def is_command(self):
        return ord(self.payload[0]) == self.COMMAND

    @property
    def is_response(self):
        return ord(self.payload[0]) == self.RESPONSE

    @property
    def is_hard(self):
        return ord(self.payload[1]) == self.HARD

    @property
    def is_soft(self):
        return ord(self.payload[1]) == self.SOFT

    @property
    def is_asserted(self):
        return ord(self.payload[2]) == self.ASSERTED

    @property
    def is_unasserted(self):
        return ord(self.payload[2]) == self.UNASSERTED

    def __str__(self):
        return 'KillMessage(command={}, hard={}, asserted={})'.format(self.is_command, self.is_hard, self.is_asserted)


class HeartbeatMessage(ApplicationPacket):
    '''
    Represents the special hearbeat packet send to kill/thrust board
    '''
    IDENTIFIER = ord('H')

    @classmethod
    def create(cls):
        return cls(cls.IDENTIFIER, struct.pack('BB', ord('M'), 0))

    def __str__(self):
        return 'HeartbeatMessage()'


class GoMessage(ApplicationPacket):
    '''
    Represents the Go message received from the kill/thrust board indiciating
    the status of the go plug.
    '''
    IDENTIFIER = ord('G')
    ASSERTED = 0x41
    UNASSERTED = 0x55
    PADDING = 0x00

    @classmethod
    def create_go_message(cls, asserted=False):
        asserted_byte = cls.ASSERTED if asserted else cls.UNASSERTED
        payload = struct.pack('BB', asserted_byte, cls.PADDING)
        return cls(cls.IDENTIFIER, payload)

    @property
    def asserted(self):
        return ord(self.payload[0]) == self.ASSERTED

    def __str__(self):
        return 'GoMessage(asserted={})'.format(self.asserted)


class ThrustPacket(ApplicationPacket):
    '''
    Represents a command send to the thrust/kill board to set
    the PWM of a thruster
    '''
    IDENTIFIER = ord('T')
    ID_MAPPING = {
        'FLH': 0,
        'FRH': 1,
        'FLV': 2,
        'FRV': 3,
        'BLH': 4,
        'BRH': 5,
        'BLV': 6,
        'BRV': 7,
    }

    @classmethod
    def create_thrust_packet(cls, thruster_id, command):
        payload = struct.pack('=Bf', thruster_id, command)
        return cls(cls.IDENTIFIER, payload)

    @property
    def thruster_id(self):
        return struct.unpack('B', self.payload[0])[0]

    @property
    def command(self):
        return struct.unpack('f', self.payload[1:])[0]

    def __str__(self):
        return 'ThrustPacket(thruster_id={}, command={})'.format(self.thruster_id, self.command)
