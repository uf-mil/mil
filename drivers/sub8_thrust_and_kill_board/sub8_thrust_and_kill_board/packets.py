import struct
from mil_usb_to_can import ApplicationPacket
from collections import namedtuple


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

KillStatus = namedtuple('KillStatus', ['heartbeat_lost', 'mobo_soft_kill',
                                       'switch_soft_kill', 'soft_killed',
                                       'hard_killed', 'thrusters_initializing',
                                       'go_switch', 'soft_kill_switch',
                                       'hard_kill_switch'])
class StatusMessage(KillStatus):
    BIT_MASK = KillStatus(1 << 3, 1 << 4, 1 << 5, 1 << 6, 1 << 7,
                              1 << 11, 1 << 12, 1 << 13, 1 << 14)
    STRUCT_FORMAT = '=h'

    def __init__(self, *args):
        super(StatusMessage, self).__init__(*args)

    @classmethod
    def from_bytes(cls, data):
        unpacked = struct.unpack(cls.STRUCT_FORMAT, data)[0]
        args = []
        for field in KillStatus._fields:
                args.append(bool(unpacked & getattr(cls.BIT_MASK,field)))
        return cls(*args)

    def to_bytes(self):
        out = 0
        for field in KillStatus._fields:
            if getattr(self, field):
                out = out | getattr(self.BIT_MASK, field)
        return struct.pack(self.STRUCT_FORMAT, out)


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
