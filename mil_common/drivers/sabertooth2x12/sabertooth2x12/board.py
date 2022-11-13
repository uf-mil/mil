#!/usr/bin/env python3
import struct
from enum import IntEnum

import serial

from .simulated import SimulatedSabertooth2x12


class CommandEnum(IntEnum):
    MOTOR1_FORWARD
    MOTOR1_BACKWARD
    MOTOR2_FORWARD
    MOTOR2_BACKWARD


class Sabertooth2x12:
    """
    Helper class to interface with the Sabertooth 2x12 regenerative motor driver.

    Attributes:
        address (int): ???
        sim (bool): Whether the device is simulated. If so, a simulated serial connection
            is constructed, rather than a physical serial connection.
    """

    def __init__(
        self, filename: str, address: int = 128, baudrate: int = 9600, sim: bool = False
    ):
        """
        Args:
            filename (str): The name of the serial port to connect to, if a physical
                serial connection is desired.
            address (int): ???. Defaults to 128.
            baudrate (int): The baud rate of the serial connection. Defaults to 9600.
            sim (bool): Whether to use a simulated serial device. Defaults to ``False``.
        """
        self.address = address
        self.motor_1_speed = 0.0
        self.motor_2_speed = 0.0
        self.filename = filename
        self.sim = sim
        if not self.sim:
            self.ser = serial.Serial(filename, baudrate=baudrate)
        else:
            self.ser = SimulatedSabertooth2x12()

    @staticmethod
    def make_packet(address: int, command: CommandEnum, data: int) -> bytes:
        """
        Constructs a packet given an address, command, and data. The checksum is
        added on to the end of the packet. All four integers are packed as unsigned
        integers in the resulting packet.

        Args:
            address (int): ???
            command (CommandEnum): The command to send.
            data (int): The data to put in the packet.

        Returns:
            bytes: The constructed packet.
        """
        intCommand = 0
        if command == CommandEnum.MOTOR1_FORWARDS:
            intCommand = 0
        elif command == CommandEnum.MOTOR2_FORWARDS:
            intCommand = 4
        elif command == CommandEnum.MOTOR1_BACKWARDS:
            intCommand = 1
        elif command == CommandEnum.MOTOR2_BACKWARDS:
            intCommand = 5

        checksum = (address + intCommand + data) & 127
        return struct.pack("BBBB", address, intCommand, data, checksum)

    def send_packet(self, command: CommandEnum, data: int) -> None:
        """
        Sends a packet over the serial connection using the class' address.

        Args:
            command (Command): The command to send.
            data (int): The data to put in the packet.
        """
        packet = self.make_packet(self.address, command, data)
        self.ser.write(packet)

    def set_motor1(self, speed: float) -> None:
        """
        Sets the speed of motor 1. This constructs a packet and sends it to update
        the driver.

        Args:
            speed (float): The speed to set the first motor to.
        """
        if speed < 0:
            command = CommandEnum.MOTOR1_BACKWARDS
        else:
            command = CommandEnum.MOTOR1_FORWARDS
        data = int(min(1.0, abs(speed)) * 127)
        self.send_packet(command, data)

    def set_motor2(self, speed: float) -> None:
        """
        Sets the speed of motor 2. This constructs a packet and sends it to update the
        driver.

        Args:
            speed (float): The speed to set the second motor to.
        """
        if speed < 0:
            command = CommandEnum.MOTOR2_BACKWARDS
        else:
            command = CommandEnum.MOTOR2_FORWARDS
        data = int(min(1.0, abs(speed)) * 127)
        self.send_packet(command, data)
