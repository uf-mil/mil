import struct
from collections import namedtuple

from mil_usb_to_can import ApplicationPacket

# CAN channel to send thrust messages to
THRUST_SEND_ID = 0x21
# CAN channel ot send kill messages to
KILL_SEND_ID = 0x11


class KillMessage(ApplicationPacket):
    """
    Represents a packet sent to kill/thrust board which contains
    a command or response related to kill status.

    Inherits from :class:`~mil_usb_to_can.ApplicationPacket`.

    .. container:: operations

        .. describe:: str(x)

            Pretty-prints the class name and each of the three packet attributes.

    Attributes:
        IDENTIFIER (int): The packet's identifier. Set equal to the ordinal value
            of "K", or 75.
        COMMAND (int): Identifier representing the packet as a command packet.
            Equal to 67. This identifier should only be found in the first byte
            of the payload.
        RESPONSE (int): Identifier representing the packet as a response packet.
            Equal to 82. This identifier should only be found in the first byte of
            the payload.
        HARD (int): Identifier representing the type of kill as a hard kill. Equal
            to 72. This identifier is designed to be used in the second byte of the
            payload.
        SOFT (int): Identifier representing the type of kill as a soft kill. Equal
            to 83. This identifier is designed to be used in the second byte of the
            payload.
        ASSERTED (int): Identifier equal to 65. This identifier is meant to be used
            in the third byte of the payload.
        UNASSERTED (int): Identifier equal to 85. This identifier is meant to be used
            in the third byte of the payload.
    """

    IDENTIFIER = ord("K")
    COMMAND = 0x43
    RESPONSE = 0x52
    HARD = 0x48
    SOFT = 0x53
    ASSERTED = 0x41
    UNASSERTED = 0x55
    PADDING = 0x00

    @classmethod
    def create_kill_message(
        cls, command: bool = False, hard: bool = False, asserted: bool = False
    ):
        """
        Creates a kill message containing three bytes of information, specified
        as parameters.

        Args:
            command (bool): Whether to make the kill message a command. If ``False``,
                then the packet will represent a response.
            hard (bool): Whether to make the packet kill type hard. If ``False``,
                then the packet will represent a soft kill.
            asserted (bool): Whether to make the packet asserted. If ``False``, then
                an unasserted packet is generated.
        """
        command_byte = cls.COMMAND if command else cls.RESPONSE
        hard_soft_byte = cls.HARD if hard else cls.SOFT
        assert_unassert_byte = cls.ASSERTED if asserted else cls.UNASSERTED
        payload = struct.pack(
            "BBBB", command_byte, hard_soft_byte, assert_unassert_byte, cls.PADDING
        )
        return cls(cls.IDENTIFIER, payload)

    @property
    def is_command(self) -> bool:
        """
        Whether the packet represents a command.

        Returns:
            bool: The status of the packet's command/response type.
        """
        return self.payload[0] == self.COMMAND

    @property
    def is_response(self) -> bool:
        """
        Whether the packet represents a response.

        Returns:
            bool: The status of the packet's command/response type.
        """
        return self.payload[0] == self.RESPONSE

    @property
    def is_hard(self) -> bool:
        """
        Whether the packet represents a hard kill.

        Returns:
            bool: The status of the packet's hard/soft kill type.
        """
        return self.payload[1] == self.HARD

    @property
    def is_soft(self) -> bool:
        """
        Whether the packet represents a soft kill.

        Returns:
            bool: The status of the packet's hard/soft kill type.
        """
        return self.payload[1] == self.SOFT

    @property
    def is_asserted(self):
        """
        Whether the packet represents an asserted packet.

        Returns:
            bool: The status of the packet's asserteed/unasserted type.
        """
        return self.payload[2] == self.ASSERTED

    @property
    def is_unasserted(self):
        """
        Whether the packet represents an unasserted packet.

        Returns:
            bool: The status of the packet's asserteed/unasserted type.
        """
        return self.payload[2] == self.UNASSERTED

    def __str__(self):
        return "KillMessage(command={}, hard={}, asserted={})".format(
            self.is_command, self.is_hard, self.is_asserted
        )


KillStatus = namedtuple(
    "KillStatus",
    [
        "heartbeat_lost",
        "mobo_soft_kill",
        "switch_soft_kill",
        "soft_killed",
        "hard_killed",
        "thrusters_initializing",
        "go_switch",
        "soft_kill_switch",
        "hard_kill_switch",
    ],
)


class StatusMessage(KillStatus):
    BIT_MASK = KillStatus(
        1 << 3, 1 << 4, 1 << 5, 1 << 6, 1 << 7, 1 << 11, 1 << 12, 1 << 13, 1 << 14
    )
    STRUCT_FORMAT = "=h"

    def __new__(cls, *args):
        """
        Constructs a new namedtuple to derive the class from. This can't be done
        in __init__ because namedtuples are immutable.
        """
        return super().__new__(cls, *args)

    @classmethod
    def from_bytes(cls, data):
        unpacked = struct.unpack(cls.STRUCT_FORMAT, data)[0]
        args = []
        for field in KillStatus._fields:
            args.append(bool(unpacked & getattr(cls.BIT_MASK, field)))
        return cls(*args)

    def to_bytes(self):
        out = 0
        for field in KillStatus._fields:
            if getattr(self, field):
                out = out | getattr(self.BIT_MASK, field)
        return struct.pack(self.STRUCT_FORMAT, out)


class HeartbeatMessage(ApplicationPacket):
    """
    Represents the special hearbeat packet send to kill and thruster board.

    Inherits from :class:`~mil_usb_to_can.ApplicationPacket`.

    .. container:: operations

        .. describe:: str(x)

            Pretty-prints the class name.

    Attributes:
        IDENTIFIER (int): The packet identifier. Set equal to the ordinal value of
            "H," or 72.
    """

    IDENTIFIER = ord("H")

    @classmethod
    def create(cls):
        """
        Creates a new heartbeat message to be sent.
        """
        return cls(cls.IDENTIFIER, struct.pack("BB", ord("M"), 0))

    def __str__(self):
        return "HeartbeatMessage()"


class ThrustPacket(ApplicationPacket):
    """
    Represents a command send to the thrust/kill board to set the PWM of a thruster.

    Inherits from :class:`~mil_usb_to_can.ApplicationPacket`.

    .. container:: operations

        .. describe:: str(x)

            Pretty-prints the class name and each of the two packet attributes,
            the thruster ID and the command.

    Attributes:
        IDENTIFIER (int): The packet identifier, equal to the ordinal value of "T,"
            or 84.
        ID_MAPPING (Dict[str, int]): A dictionary mapping 3-letter thruster codes
            to their respective IDs:

            +--------+------+
            |  Name  |  ID  |
            +========+======+
            |  FLH   |  0   |
            +--------+------+
            |  FRH   |  1   |
            +--------+------+
            |  FLV   |  2   |
            +--------+------+
            |  FRV   |  3   |
            +--------+------+
            |  BLH   |  4   |
            +--------+------+
            |  BRH   |  5   |
            +--------+------+
            |  BLV   |  6   |
            +--------+------+
            |  BRV   |  7   |
            +--------+------+
    """

    IDENTIFIER = ord("T")
    ID_MAPPING = {
        "FLH": 0,
        "FRH": 1,
        "FLV": 2,
        "FRV": 3,
        "BLH": 4,
        "BRH": 5,
        "BLV": 6,
        "BRV": 7,
    }

    @classmethod
    def create_thrust_packet(cls, thruster_id: int, command: float):
        """
        Creates a thruster packet given an ID and command.

        Args:
            thruster_id (int): The ID of the thruster to create a packet for.
            command (float): The command to associate with the packet.
        """
        payload = struct.pack("=Bf", thruster_id, command)
        return cls(cls.IDENTIFIER, payload)

    @property
    def thruster_id(self) -> int:
        """
        The thruster ID associated with the packet.

        Returns:
            int: The ID.
        """
        return struct.unpack("B", self.payload[0])[0]

    @property
    def command(self) -> float:
        """
        The command associated with the packet.

        Returns:
            float: The associated command.
        """
        return struct.unpack("f", self.payload[1:])[0]

    def __str__(self):
        return "ThrustPacket(thruster_id={}, command={})".format(
            self.thruster_id, self.command
        )
