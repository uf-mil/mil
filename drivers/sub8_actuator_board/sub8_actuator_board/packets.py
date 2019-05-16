import struct
from mil_usb_to_can import ApplicationPacket


# CAN ID for the channel from MOBO to actuator board
SEND_ID = 0x51


class CommandMessage(ApplicationPacket):
    '''
    Represents a packet sent from the motherboard to the actuator board
    '''
    IDENTIFIER = ord('A')
    NUM_VALVES = 12

    @property
    def address(self):
        '''
        the integer address of this command
        '''
        return struct.unpack('B', self.payload[0])[0]

    @property
    def write(self):
        '''
        If true, will set the valve open or closed.
        If false, this message requests the status of a valve
        '''
        return struct.unpack('B', self.payload[1])[0] == 1

    @property
    def on(self):
        '''
        If true, valve is commanded to be open or requested actuator's status is open
        '''
        return struct.unpack('B', self.payload[2])[0] == 1

    @classmethod
    def create_command_message(cls, address=0, write=0, on=0):
        '''
        Create a new command message
        @param address: integer address of valve from 0-11
        @param write: If true, set valve open or close. If False, request the status of the specified valve
        @param on: If true, command valve to open. If write if False, this has no effect
        '''
        if address < 0 or address >= cls.NUM_VALVES:
            raise InvalidAddressException(address)
        write_byte = 1 if write else 0
        on_byte = 1 if on else 0
        payload = struct.pack('BBB', address, write_byte, on_byte)
        return cls(cls.IDENTIFIER, payload)

    def __str__(self):
        return 'CommandMessage(address={}, write={}, on={})'.format(self.address, self.write, self.on)


class InvalidAddressException(RuntimeError):
    def __init__(self, address):
        super(InvalidAddressException, self).__init__(
            'Attempted to command valve {}, but valid addresses are only [0,{}]'.format(
                address, CommandMessage.NUM_VALVES - 1))


class FeedbackMessage(ApplicationPacket):
    '''
    Represents a message received from the actuator board sent to the motherboard
    '''
    IDENTIFIER = ord('A')

    @property
    def address(self):
        '''
        Valve this status message references, integer from 0-11
        '''
        return struct.unpack('B', self.payload[0])[0]

    @property
    def on(self):
        '''
        If True, valve is currently open
        If False, valve is currently closed
        '''
        return struct.unpack('B', self.payload[1])[0] == 1

    @classmethod
    def create_feedback_message(cls, address=0, on=False):
        '''
        Create a feedback message
        @param address: valve address this status references, integer 0-11
        @param on: If True, valve is open
        '''
        on_byte = 1 if on else 0
        payload = struct.pack('BBB', address, on_byte, 0)
        return cls(cls.IDENTIFIER, payload)

    def __str__(self):
        return 'FeedbackMessage(address={}, on={})'.format(self.address, self.on)
