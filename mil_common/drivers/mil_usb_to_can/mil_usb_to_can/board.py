#!/usr/bin/env python3
from threading import Lock

# from mil_tools import hexify
from typing import Optional

import serial

from mil_usb_to_can.simulation import SimulatedUSBtoCAN
from mil_usb_to_can.utils import (  # causes error if relative import used - GH-731
    CommandPacket,
    ReceivePacket,
)


class USBtoCANBoard:
    """
    ROS-independent wrapper which provides an interface to connect to the USB to CAN board
    via a serial (or simulated serial) device. Provides thread-safe functionality.

    Attributes:
        lock (threading.Lock): The thread lock.
        ser (Union[:class:`SimulatedUSBtoCAN`, :class:`serial.Serial`]): The serial connection.
    """

    def __init__(self, port: str, baud: int = 9600, simulated: bool = False, **kwargs):
        """
        Args:
                port (str): Path to serial device, such as ``/dev/ttyUSB0``.
                baud (int): Baud rate of serial device to connect to. Defaults to 9600.
                simulated (bool): If True, use a simulated serial device rather than a real device. Defaults to ``False``.
        """
        self.lock = Lock()
        if simulated:
            self.ser = SimulatedUSBtoCAN(**kwargs)
        else:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1, **kwargs)
        self.ser.flushOutput()
        self.ser.flushInput()

    def read_packet(self) -> Optional[ReceivePacket]:
        """
        Read a packet from the board, if available. Returns a :class:`ReceivePacket`
        instance if one was succefully read, or ``None`` if the in buffer is empty.

        Returns:
            Optional[:class:`ReceivePacket`]: The packet, if found, otherwise ``None``.
        """
        # TODO Does this actually return ReceivePacket? Appears it might only be
        # able to return Packet.
        with self.lock:
            if self.ser.in_waiting == 0:
                return None
            return ReceivePacket.read_packet(self.ser)

    def send_data(self, data: bytes, can_id: int = 0) -> None:
        """
        Sends data to a CAN device using the thread lock. Writes using the :meth:`write`
        method of the :attr:`.ser` attribute.

        Args:
            device_id (int): CAN device ID to send data to.
            data (bytes): Data (represented as bytes) to send to the device.

        Raises:
            PayloadTooLargeException: The payload is larger than 8 bytes.
        """
        p = CommandPacket.create_send_packet(data, can_id=can_id)
        with self.lock:
            p_bytes = bytes(p)
            # print 'SENDING ', hexify(p_bytes)
            self.ser.write(p_bytes)
