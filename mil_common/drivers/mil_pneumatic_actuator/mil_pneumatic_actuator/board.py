#!/usr/bin/env python3
import threading
from typing import Optional

import serial
from mil_ros_tools import thread_lock

from .constants import Constants
from .simulated_board import SimulatedPnuematicActuatorBoard

lock = threading.Lock()


class PnuematicActuatorDriverError(Exception):
    """
    The base exception class for all exceptions/errors related to the Pneumatic
    Actuator Board.

    Inherits from :class:`Exception`.
    """

    def __init__(self, message):
        super().__init__("Actuator board: " + message)


class PnuematicActuatorDriverChecksumError(PnuematicActuatorDriverError):
    """
    Exception representing an invalid checksum.

    Inherits from :class:`PnuematicActuatorDriverError`.
    """

    def __init__(self, checksum_is, checksum_should_be):
        message = f"Invalid checksum. Recievied {hex(checksum_is)}, should be {hex(checksum_should_be)}"
        super().__init__(message)


class PnuematicActuatorDriverResponseError(PnuematicActuatorDriverError):
    """
    Exception representing an invalid response.

    Inherits from :class:`PnuematicActuatorDriverError`.
    """

    def __init__(self, received, expected):
        message = f"Unexpected response. Expected {hex(received)}, received {hex(expected)}"
        super().__init__(message)


class PnuematicActuatorTimeoutError(PnuematicActuatorDriverError):
    """
    Exception representing a serial timeout experienced by the board.

    Inherits from :class:`PnuematicActuatorDriverError`.
    """

    def __init__(self):
        message = "Serial timeout"
        super().__init__(message)


class PnuematicActuatorDriver:
    """
    Allows high level ROS code to interface with Daniel's pneumatics board.

    For the dropper and grabber systems, call service with ``True`` or ``False``
    to open or close. For the shooter system, sending a ``True`` signal will
    pulse the valve.

    Further information on the board's communication protocol can be found in the
    design documentation.
    """

    # TODO: Add a function to try and reconnect to the serial port if we lose connection.

    def __init__(self, port: str, baud: int = 9600, simulated: bool = False):
        """
        Args:
            port (str): The nname of the board to establish a serial connection to.
            baud (int): The baud rate to establish the serial connection at.
            simulated (bool): Whether to use a simulated actuator board class
                or an interface to the physical board.
        """
        if simulated:
            self.ser = SimulatedPnuematicActuatorBoard()
        else:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=2.0)
        self.ser.flushInput()

    @classmethod
    def _verify_checksum(cls, byte: int, checksum: int) -> None:
        """
        Verifies that two checksums are equivalent. If they are not equivalent,
        then an exception is raised. Otherwise, ``None`` is implicitly returned.
        """
        if not Constants.verify_checksum(byte, checksum):
            raise PnuematicActuatorDriverChecksumError(
                checksum,
                Constants.create_checksum(byte),
            )

    def _get_response(self) -> int:
        """
        Internal method to return only the desired data sent by the actuator board.

        Returns:
            int: The response sent by the board.
        """
        data = self.ser.read(2)
        if len(data) != 2:
            raise PnuematicActuatorTimeoutError
        response = Constants.deserialize_packet(data)
        data = response[0]
        chksum = response[1]
        self._verify_checksum(data, chksum)
        return data

    @thread_lock(lock)
    def _send_request(self, byte: int, expected_response: Optional[int] = None) -> int:
        """
        Internal method which sends some data and compares it to the expected
        response (if desired) before returning it to the caller.
        """
        data = Constants.serialize_packet(byte)
        self.ser.write(data)
        response = self._get_response()
        if expected_response is not None and expected_response != response:
            raise PnuematicActuatorDriverResponseError(response, expected_response)
        return response

    def open_port(self, port: int) -> int:
        """
        Opens a particular port.

        Args:
            port (int): The port to open.

        Raises:
            PnuematicActuatorDriverResponseError: The expected response from the board
                was not received.
            PnuematicActuatorDriverChecksumError: The checksum expected and the checksum
                received were not the same. The board may be malfunctioning or the
                communication with the board may be disrupted.

        Returns:
            int: The response from the board, which is frequently a standard hexadecimal value
                indicating that the board opened the valve at the desired port.
                This value is equal to :attr:`mil_pneumatic_actuator.Constants.OPEN_RESPONSE`.
        """
        byte = Constants.OPEN_REQUEST_BASE + port
        return self._send_request(byte, Constants.OPEN_RESPONSE)

    def close_port(self, port: int) -> int:
        """
        Closes a particular port.

        Args:
            port (int): The port to close.

        Raises:
            PnuematicActuatorDriverResponseError: The expected response from the board
                was not received.
            PnuematicActuatorDriverChecksumError: The checksum expected and the checksum
                received were not the same. The board may be malfunctioning or the
                communication with the board may be disrupted.

        Returns:
            int: The response from the board, which is frequently a standard hexadecimal value
                indicating that the board closed the valve at the desired port.
                This value is equal to :attr:`mil_pneumatic_actuator.Constants.CLOSE_RESPONSE`.
        """
        byte = Constants.CLOSE_REQUEST_BASE + port
        return self._send_request(byte, Constants.CLOSE_RESPONSE)

    def set_port(self, port: int, do_open: bool) -> int:
        """
        Sets the data at a particular port to opened/closed. This does not depend
        on the internal state of the port when called, ie, if you request a port to
        be closed, and the port is already closed, then the response is sent anyways.

        Args:
            port (int): The specific port to open/close.
            do_open (bool): Whether to open the port. If ``False``, then the port
                is closed.

        Raises:
            PnuematicActuatorDriverResponseError: The expected response from the board
                was not received.
            PnuematicActuatorDriverChecksumError: The checksum expected and the checksum
                received were not the same. The board may be malfunctioning or the
                communication with the board may be disrupted.
        """
        if do_open:
            return self.open_port(port)
        else:
            return self.close_port(port)

    def get_port(self, port: int) -> int:
        """
        Reads the data at a specific port.

        Args:
            port (int): The port to read from.

        Raises:
            PnuematicActuatorDriverResponseError: The expected response from the board
                was not received.
            PnuematicActuatorDriverChecksumError: The checksum expected and the checksum
                received were not the same. The board may be malfunctioning or the
                communication with the board may be disrupted.

        Returns:
            int: The response served by the board.
        """
        byte = Constants.READ_REQUEST_BASE + port
        return self._send_request(byte)

    def ping(self) -> int:
        """
        Sends a ping message to the board and returns the response.

        Raises:
            PnuematicActuatorDriverResponseError: The expected response from the board
                was not received.
            PnuematicActuatorDriverChecksumError: The checksum expected and the checksum
                received were not the same. The board may be malfunctioning or the
                communication with the board may be disrupted.

        Returns:
            int: The response from the board.
        """
        return self._send_request(Constants.PING_REQUEST, Constants.PING_RESPONSE)
