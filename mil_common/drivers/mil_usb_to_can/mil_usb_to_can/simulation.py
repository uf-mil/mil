#!/usr/bin/python3
from __future__ import annotations

import struct

from mil_misc_tools.serial_tools import SimulatedSerial

from .application_packet import ApplicationPacket
from .utils import CommandPacket, ReceivePacket


class SimulatedCANDevice:
    """
    Simulates a CAN device, with functions to be overridden to handle data requests
    and sends from motherboard.

    Child classes can inherit from this class to implement a simulated CAN device.
    """

    def __init__(self, sim_board: SimulatedUSBtoCAN, can_id: int):
        self._sim_board = sim_board
        self._can_id = can_id

    def send_data(self, data: bytes):
        """
        Send data onto the bus, delivering it to other simulated devices and to
        the driver node.
        """
        self._sim_board.send_to_bus(self._can_id, data)

    def on_data(self, data: bytes, can_id: int):
        """
        Called when the motherboard or another simulated device sends data onto the bus.

        .. note::

            Because the CAN bus is shared, you must verify that the received data
            is appropriate for your device.

        Args:
            data (bytes): The data payload as a string/bytes object.
        """


class ExampleSimulatedEchoDevice(SimulatedCANDevice):
    """
    Example implementation of a SimulatedCANDevice.
    On sends, stores the transmitted data in a buffer.
    When data is requested, it echos this data back.
    """

    def __init__(self, *args, **kwargs):
        # Call parent classes constructor
        super().__init__(*args, **kwargs)

    def on_data(self, data, can_id):
        # Echo data received back onto the bus
        self.send_data(data)


class ExampleSimulatedAdderDevice(SimulatedCANDevice):
    """
    Example implementation of a SimulatedCANDevice.
    On sends, stores the transmitted data in a buffer.
    When data is requested, it echos this data back.
    """

    def __init__(self, *args, **kwargs):
        # Call parent classes constructor
        super().__init__(*args, **kwargs)

    def on_data(self, data, can_id):
        packet = ApplicationPacket.from_bytes(data, expected_identifier=37)
        a, b = struct.unpack("hh", packet.payload)
        c = a + b
        res = struct.pack("i", c)
        self.send_data(ApplicationPacket(37, res).to_bytes())


class SimulatedUSBtoCAN(SimulatedSerial):
    """
    Simulates the USB to CAN board. Is supplied with a dictionary of simulated
    CAN devices to simulate the behavior of the whole CAN network.
    """

    def __init__(self, devices={0: SimulatedCANDevice}, can_id=-1):
        """
        Args:
            devices (Dict[:class:`int`, Any]): Dictionary containing CAN IDs and
                their associated simulated classes inheriting from :class:`SimulatedCANDevice`.
            can_id (int): ID of the CAN2USB device. Defaults to -1.
        """
        self._my_id = can_id
        self._devices = {
            can_id: device(self, can_id) for can_id, device in devices.items()
        }
        super().__init__()

    def send_to_bus(self, can_id: int, data: bytes, from_mobo: bool = False):
        """
        Sends data onto the simulated bus from a simulated device.

        Args:
            can_id (int): ID of sender.
            data (bytes): The payload to send.
            from_mobo (bool): Whether the data is from the motherboard. Defaults to
                False.
        """
        # If not from the motherboard, store this for future requests from motherboard
        if not from_mobo:
            self.buffer += ReceivePacket.create_receive_packet(can_id, data).to_bytes()
        # Send data to all simulated devices besides the sender
        for device_can_id, device in self._devices.items():
            if device_can_id != can_id:
                device.on_data(data, can_id)

    def write(self, data: bytes) -> int:
        """
        Parse incoming data as a command packet from the motherboard.
        """
        p = CommandPacket.from_bytes(data)
        self.send_to_bus(p.filter_id, p.data, from_mobo=True)
        return len(data)
