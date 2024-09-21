from __future__ import annotations

from typing import TYPE_CHECKING

from electrical_protocol import Packet

if TYPE_CHECKING:
    from .sub9_driver import SimulatedUSBtoCANStream, USBtoCANDriver


class SimulatedCANDeviceHandle:
    """
    Simulates a CAN device, with functions to be overridden to handle data requests
    and sends from motherboard.

    Child classes can inherit from this class to implement a simulated CAN device.

    Attributes:
        inbound_packets (type[Packet]): The types of packets listened to by this device.
            Packets of this type will be routed to the :meth:`~.on_data` method of
            the device handle.
    """

    def __init__(
        self,
        sim_board: SimulatedUSBtoCANStream,
        inbound_packets: list[type[Packet]],
    ):
        self._sim_board = sim_board
        self.inbound_packets = inbound_packets

    def send_data(self, data: bytes):
        """
        Sends data over the serial connection.
        """
        self._sim_board.send_to_bus(data)

    def on_data(self, packet: Packet):
        """
        Called when an relevant incoming packet is received over the serial
        connection. Relevant packets are those listed in :attr:`~.inbound_packets`.

        Partial data (ie, incomplete packets) are not sent through this event.

        Args:
            packet (Packet): The incoming packet.
        """
        del packet


class CANDeviceHandle:
    """
    Base class to allow developers to write handles for communication with a
    particular CAN device. The two methods of the handle allow the handle to listen
    to incoming data, as well as send data.
    """

    def __init__(self, driver: USBtoCANDriver):
        """
        Args:
            driver (USBtoCANBoard): Driver that is used to communicate with the board.
            device_id (int): The CAN ID of the device this class will handle. Not currently used.
        """
        self._driver = driver

    def on_data(self, data: Packet):
        """
        Called when a return packet is sent over the serial connection. In the
        USB to CAN protocol, it is assumed that packets will be returned to the
        motherboard in the order they are sent out. Therefore, the type of packet
        sent through this event is not guaranteed, and is only determined by the
        message and subclass ID sent by the individual board.

        Partial data (ie, incomplete packets) are not sent through this event.

        Args:
            packet (Packet): The incoming packet.
        """
        del data

    def send_data(self, data: Packet):
        """
        Sends a packet over the serial connection.

        Args:
            data (Packet): The packet to send.
        """
        return self._driver.send_data(self, data)
