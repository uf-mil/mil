import struct
from mil_usb_to_can import ApplicationPacket


# CAN ID for the channel from MOBO to actuator board
SEND_ID = 0x51


class CommandMessage(ApplicationPacket):
    """
    Represents a packet sent from the motherboard to the actuator board. Being a packet,
    inherits from :class:`ApplicationPacket`.

    Attributes:
        IDENTIFIER (int): The ASCII value of "A".
        NUM_VALVES (int): The number of valves. Set to ``12`` currently.
    """

    IDENTIFIER = ord("A")
    NUM_VALVES = 12

    @property
    def address(self) -> int:
        """
        The integer address of this command.

        :rtype: int
        """
        return struct.unpack("B", self.payload[0])[0]

    @property
    def write(self) -> bool:
        """
        If True, this message sets the valve open or closed. If False, this
        message requests the status of a valve.

        :rtype: bool
        """
        return struct.unpack("B", self.payload[1])[0] == 1

    @property
    def on(self) -> bool:
        """
        If True, this message commanded the valve to be open. Else, the valve
        is commanded to be closed.

        This parameter only has any effect if :attr:`~.write` is true.

        :rtype: bool
        """
        return struct.unpack("B", self.payload[2])[0] == 1

    @classmethod
    def create_command_message(cls, address: int = 0, write: int = 0, on: int = 0):
        """
        Create a new command message.

        Args:
            address (int): Address of valve, ranging from 0-11.
            write (int): If true, set valve open or close. If False, request
              the status of the specified valve.
            on (int): If true, command valve to open. If write if False, this has no effect

        Raises:
            InvalidAddressException: The valve address is not valid, and is outside
              of the 0-11 range.
        """
        if address < 0 or address >= cls.NUM_VALVES:
            raise InvalidAddressException(address)
        write_byte = 1 if write else 0
        on_byte = 1 if on else 0
        payload = struct.pack("BBB", address, write_byte, on_byte)
        return cls(cls.IDENTIFIER, payload)

    def __str__(self):
        return "CommandMessage(address={}, write={}, on={})".format(
            self.address, self.write, self.on
        )


class InvalidAddressException(RuntimeError):
    """
    Exception noting that the requested valve address is outside of the range available
    on the sub. Inherits from :class:`RuntimeError`.
    """

    def __init__(self, address: int):
        super(InvalidAddressException, self).__init__(
            "Attempted to command valve {}, but valid addresses are only [0,{}]".format(
                address, CommandMessage.NUM_VALVES - 1
            )
        )


class FeedbackMessage(ApplicationPacket):
    """
    Represents a message received from the actuator board sent to the motherboard

    Attributes:
        IDENITIFER (int): The packet identifier. Set to the ASCII value of "A".
    """

    IDENTIFIER = ord("A")

    @property
    def address(self) -> int:
        """
        The address of valve referenced by this command. Should be between 0-11.

        :rtype: int
        """
        return struct.unpack("B", self.payload[0])[0]

    @property
    def on(self) -> bool:
        """
        If True, valve is currently open. If False, valve is currently closed.

        :rtype: bool
        """
        return struct.unpack("B", self.payload[1])[0] == 1

    @classmethod
    def create_feedback_message(cls, address: int = 0, on: bool = False):
        """
        Create a feedback message.

        Args:
            address (int): valve address this status references, integer 0-11
            on (bool): If True, valve is open
        """
        on_byte = 1 if on else 0
        payload = struct.pack("BBB", address, on_byte, 0)
        return cls(cls.IDENTIFIER, payload)

    def __str__(self):
        return "FeedbackMessage(address={}, on={})".format(self.address, self.on)
