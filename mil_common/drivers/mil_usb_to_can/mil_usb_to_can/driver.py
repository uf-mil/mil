#!/usr/bin/python3
from __future__ import annotations

import importlib
from collections import deque
from threading import Lock
from typing import TYPE_CHECKING, Literal, overload

import rospy
import serial
from mil_misc_tools.serial_tools import SimulatedSerial
from mil_usb_to_can.device import CANDeviceHandle, SimulatedCANDeviceHandle
from mil_usb_to_can.packet import SYNC_CHAR_1, Packet
from serial import SerialException

if TYPE_CHECKING:
    HandlePacketListing = tuple[
        type[SimulatedCANDeviceHandle | CANDeviceHandle], list[Packet]
    ]


class SimulatedUSBtoCANStream(SimulatedSerial):
    """
    Simulates the USB to CAN board. Is supplied with a dictionary of simulated
    CAN devices to simulate the behavior of the whole CAN network.
    """

    _devices: list[SimulatedCANDeviceHandle]

    def __init__(
        self,
        devices: list[tuple[type[SimulatedCANDeviceHandle], list[type[Packet]]]]
        | None = None,
    ):
        """
        Args:
            devices (List[Tuple[Type[SimulatedCANDeviceHandle], List[Type[Packet]]]]): List of
                the simulated device handles, along with the list of packets each handle
                is listening for.
        """
        if devices is None:
            devices = []

        self._devices = [device(self, packet_list) for device, packet_list in devices]
        super().__init__()

    def send_to_bus(self, data: bytes, *, from_mobo=False):
        """
        Sends data onto the simulated bus from a simulated device.

        Args:
            data (bytes): The payload to send on to the bus.
            from_mobo (bool): Whether the data is from the motherboard. Defaults to
                False.
        """
        # Deconstruct packet
        p = Packet.from_bytes(data)

        if not from_mobo:
            self.buffer += data

        # Send packet to appropriate handle
        sent = False
        for device in self._devices:
            if type(p) in device.inbound_packets:
                device.on_data(p)
                sent = True

        if not sent and from_mobo:
            rospy.logerr(
                f"{sent}, {from_mobo}: Received packet of type {p.__class__.__qualname__} on simulated bus, but no one is listening for it..."
            )

    def write(self, data: bytes) -> None:
        """
        Sends data to the bus. This method should only be called by the driver.
        """
        self.send_to_bus(data, from_mobo=True)


class USBtoCANDriver:
    """
    ROS Driver which implements the USB to CAN board. Allow users to specify a dictionary of
    device handle classes to be loaded at runtime to handle communication with
    specific devices.
    """

    _packet_deque: deque[
        SimulatedCANDeviceHandle | CANDeviceHandle
    ]  # Used to keep track of who gets incoming packets
    _inbound_listing: dict[type[Packet], CANDeviceHandle]

    def __init__(self):
        port = rospy.get_param("~port", "/dev/tty0")
        baud = rospy.get_param("~baudrate", 115200)
        can_id = rospy.get_param("~can_id", 0)
        simulation = rospy.get_param("/is_simulation", False)
        self.lock = Lock()
        self._packet_deque = deque()
        # If simulation mode, load simulated devices
        if simulation:
            rospy.logwarn(
                "CAN2USB driver in simulation! Will not talk to real hardware."
            )
            devices = self.read_devices(simulated=True)
            self.stream = SimulatedUSBtoCANStream(devices=devices)
        else:
            self.stream = serial.Serial(port=port, baudrate=baud, timeout=0.1)

        self.stream.reset_output_buffer()
        self.stream.reset_input_buffer()

        self.handles = []
        self._inbound_listing = {}
        for device in self.read_devices(simulated=False):
            handle = device[0](self)
            self.handles.append(handle)
            for packet in device[1]:
                self._inbound_listing[packet] = handle

        self.timer = rospy.Timer(rospy.Duration(1.0 / 20.0), self.process_in_buffer)

    def read_from_stream(self) -> bytes | None:
        # Read until SOF is encourntered in case buffer contains the end of a previous packet
        sof = None
        for _ in range(10):
            sof = self.stream.read(1)
            if sof is None or len(sof) == 0:
                return None
            sof_int = int.from_bytes(sof, byteorder="big")
            if sof_int == SYNC_CHAR_1:
                break
        assert isinstance(sof, bytes)
        sof_int = int.from_bytes(sof, byteorder="big")
        if sof_int != SYNC_CHAR_1:
            print("Where da start char at?")
        data = sof
        # Read sync char 2, msg ID, subclass ID
        data += self.stream.read(3)
        length = self.stream.read(2)  # read payload length
        data += length
        data += self.stream.read(
            int.from_bytes(length, byteorder="little") + 2
        )  # read data and checksum
        return data

    def read_packet(self) -> bool:
        """
        Attempt to read a packet from the board. If the packet has an appropriate device
        handler, then the packet is passed to the ``on_data`` method of that handler.

        Returns:
            bool: The success in reading a packet.
        """
        try:
            with self.lock:
                if self.stream.in_waiting == 0:
                    return False
            packed_packet = self.read_from_stream()
            assert isinstance(packed_packet, bytes)
            packet = Packet.from_bytes(packed_packet)
        except (SerialException) as e:
            rospy.logerr(f"Error reading packet: {e}")
            return False
        if packet is None:
            return False
        if packet.__class__ in self._inbound_listing:
            self._inbound_listing[packet.__class__].on_data(packet)
        elif not self._packet_deque:
            rospy.logwarn(
                f"Message of type {packet.__class__.__qualname__} received, but no device ready to accept"
            )
        else:
            self._packet_deque.popleft().on_data(packet)
        return True

    def process_in_buffer(self, *args) -> None:
        """
        Read all available packets in the board's in-buffer.
        """
        while True:
            try:
                self.read_packet()
            except Exception as e:
                rospy.logerr(f"Error when reading packets: {e}")

    def send_data(
        self, handle: CANDeviceHandle | SimulatedCANDeviceHandle, packet: Packet
    ) -> Exception | None:
        """
        Sends data using the :meth:`USBtoCANBoard.send_data` method.

        Returns:
            Optional[Exception]: If data was sent successfully, nothing is returned.
            Otherwise, the exception that was raised in sending is returned.
        """
        try:
            with self.lock:
                self.stream.write(bytes(packet))
                self._packet_deque.append(handle)
            return None
        except (SerialException) as e:
            rospy.logerr(f"Error writing packet: {e}")
            return e

    @overload
    def read_devices(
        self, *, simulated: Literal[True]
    ) -> list[tuple[type[SimulatedCANDeviceHandle], list[type[Packet]]]]:
        ...

    @overload
    def read_devices(
        self, *, simulated: Literal[False]
    ) -> list[tuple[type[CANDeviceHandle], list[type[Packet]]]]:
        ...

    def read_devices(
        self, *, simulated: bool
    ) -> list[
        tuple[
            type[SimulatedCANDeviceHandle] | type[CANDeviceHandle], list[type[Packet]]
        ],
    ]:
        """
        Generator to load classes from module strings specified in a dictionary.
        Imports all found classes.

        Yields:
            Generator[Tuple[int, Any], None, None]: Yields tuples containing the device
            ID and the associated class.
        """
        d = {}
        res: list[
            tuple[type[SimulatedCANDeviceHandle | CANDeviceHandle], list[type[Packet]]]
        ] = []
        if simulated:
            d = rospy.get_param("~simulated_devices")
        else:
            d = rospy.get_param("~device_handles")

        for module_name, packet_list in d.items():
            # Split module from class name, import module, and get class from module
            module_name, cls = module_name.rsplit(".", 1)
            module = importlib.import_module(module_name)
            imported_class = getattr(module, cls)
            if simulated:
                assert issubclass(imported_class, SimulatedCANDeviceHandle)
            else:
                assert issubclass(imported_class, CANDeviceHandle)
            packets: list[type[Packet]] = []
            for packet in packet_list:
                module_name, cls = packet.rsplit(".", 1)
                module = importlib.import_module(module_name)
                packets.append(getattr(module, cls))
            res.append((imported_class, packets))
        return res


if __name__ == "__main__":
    rospy.init_node("usb_to_can_driver")
    driver = USBtoCANDriver()
    rospy.spin()
