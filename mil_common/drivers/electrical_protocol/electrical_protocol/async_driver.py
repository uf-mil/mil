'''
from __future__ import annotations

import abc
import asyncio
import contextlib
from typing import Any, Generic, TypeVar, Union, cast, get_args, get_origin

import axros

# import rospy
import serial
import serial_asyncio

from .packet import SYNC_CHAR_1, Packet

SendPackets = TypeVar("SendPackets", bound=Packet)
RecvPackets = TypeVar("RecvPackets", bound=Packet)


class DeviceProtocol(asyncio.Protocol):

    def __init__(self, device: AsyncSerialDevice):
        self.device = device
        self.byte_count = 0
        self.buffer = b""
        self.expected_count = None
        print("in init?")

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport

        print("port opened", transport)
        self.transport.serial.reset_input_buffer()  # type: ignore
        self.transport.serial.reset_output_buffer()  # type: ignore

    def data_received(self, data: bytes) -> None:
        print("data received?")
        # Either wait to hear the next start byte, or keep accumulating bytes
        # for the next packet
        if self.expected_count is None:
            for i, byte in enumerate(data):
                if byte == SYNC_CHAR_1:
                    # We expect the following bytes:
                    # - SYNC_CHAR_1
                    # - SYNC_CHAR_2
                    # - msg ID
                    # - subclass ID
                    # - payload length
                    self.byte_count = len(data) - i
                    self.expected_count = 6 - self.byte_count
                    self.buffer = bytes(data[i:])
        # We are in the middle of reading a packet
        else:
            self.byte_count += len(data)
            self.buffer += data
            if self.byte_count == 6:
                self.expected_count = int.from_bytes(data[4:6], byteorder="little") + 2
            if self.byte_count == self.expected_count:
                self.device.on_data_received(self.buffer)
                self.expected_count = None
        self.buffer += data
        self.device.on_data_received(data)


class AsyncSerialDevice(Generic[SendPackets, RecvPackets]):
    """
    Represents a generic serial device, which is expected to be the main component
    of an individual ROS node.

    Attributes:
        port (Optional[str]): The port used for the serial connection, if provided.
        baudrate (Optional[int]): The baudrate to use with the device, if provided.
        device (Optional[serial.Serial]): The serial class used to communicate with
            the device.
        rate (float): The reading rate of the device, in Hertz. Set to `20` by default.
    """

    device: serial.Serial | None
    _recv_T: Any
    _send_T: Any

    def is_connected(self) -> bool:
        return self.is_open()

    def is_open(self) -> bool:
        return self.transport is not None

    def __init__(
        self,
        node_handle: axros.NodeHandle,
        port: str | None,
        baudrate: int | None,
    ) -> None:
        """
        Arguments:
            port (Optional[str]): The serial port to connect to. If ``None``, connection
                will not be established on initialization; rather, the user can use
                :meth:`~.connect` to connect later.
            baudrate (Optional[int]): The baudrate to connect with. If ``None`` and
                a port is specified, then 115200 is assumed.
        """
        self.node_handle = node_handle
        self.port = port
        self.baudrate = baudrate
        if port:
            self.device = serial.Serial(port, baudrate or 115200, timeout=0.1)
            if not self.device.is_open:
                self.device.open()
            self.device.reset_input_buffer()
            self.device.reset_output_buffer()
        else:
            self.device = None
        self.rate = 20.0
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self._process_buffer)  # type: ignore

    def __init_subclass__(cls) -> None:
        # this is a super hack lol :P
        # cred: https://stackoverflow.com/a/71720366
        cls._send_T = get_args(cls.__orig_bases__[0])[0]  # type: ignore
        cls._recv_T = get_args(cls.__orig_bases__[0])[1]  # type: ignore

    async def connect(self, port: str, baudrate: int, loop) -> None:
        """
        Connects to the port with the given baudrate. If the device is already
        connected, the input and output buffers will be flushed.

        Arguments:
            port (str): The serial port to connect to.
            baudrate (int): The baudrate to connect with.
        """
        print("3")
        self.port = port
        self.baudrate = baudrate

        # if self.transport:
        #    raise RuntimeError("Device is already connected.")
        print("the serial connection runs after this?")
        self.transport, self.protocol = await serial_asyncio.create_serial_connection(
            loop,
            DeviceProtocol,
            port,
            baudrate=baudrate,
        )

        print("the serial connection creator finished?")

    async def close(self) -> None:
        """
        Closes the serial device.
        """
        print("Closing serial device...")
        if not self.transport:
            raise RuntimeError("Device is not connected.")
        else:
            with contextlib.suppress(OSError):
                if self.transport.serial.in_waiting:  # type: ignore
                    print(
                        "Shutting down device, but packets were left in buffer...",
                    )
            self.transport.close()

            self.transport = None

    async def write(self, data: bytes) -> None:
        """
        Writes a series of raw bytes to the device. This method should rarely be
        used; using :meth:`~.send_packet` is preferred because of the guarantees
        it provides through the packet class.

        Arguments:
            data (bytes): The data to send.
        """
        if self.transport is None:
            raise RuntimeError("Device is not connected.")
        self.transport.write(data)

    def send_packet(self, packet: SendPackets) -> None:
        print("4")
        """
        Sends a given packet to the device.

        Arguments:
            packet (:class:`~.Packet`): The packet to send.
        """
        asyncio.create_task(self.write(bytes(packet)))

    async def _read_from_stream(self) -> bytes:
        # Read until SOF is encourntered in case buffer contains the end of a previous packet
        if not self.transport:
            raise RuntimeError("Device is not connected.")
        sof = None
        for _ in range(10):
            sof = self.transport.serial.read(1)
            if not len(sof):
                continue
            sof_int = int.from_bytes(sof, byteorder="big")
            if sof_int == SYNC_CHAR_1:
                break
        if not isinstance(sof, bytes):
            raise TimeoutError("No SOF received in one second.")
        sof_int = int.from_bytes(sof, byteorder="big")
        if sof_int != SYNC_CHAR_1:
            print("Where da start char at?")
        data = sof
        # Read sync char 2, msg ID, subclass ID
        data += self.transport.serial.read(3)
        length = self.transport.serial.read(2)  # read payload length
        data += length
        data += self.transport.serial.read(
            int.from_bytes(length, byteorder="little") + 2,
        )  # read data and checksum
        return data

    def _correct_type(self, provided: Any) -> bool:
        # either:
        #   1. RecvPackets is a Packet --> check isinstance on the type var
        #   2. RecvPackets is a Union of Packets --> check isinstance on all
        if get_origin(RecvPackets) is Union:
            return isinstance(provided, get_args(RecvPackets))
        else:
            return isinstance(provided, self._recv_T)

    async def _read_packet(self) -> bool:
        if not self.device:
            raise RuntimeError("Device is not connected.")
        try:
            if not self.is_open() or self.device.in_waiting == 0:
                return False
            if self.device.in_waiting > 200:
                print(
                    0.5,
                    "Packets are coming in much quicker than expected, upping rate...",
                )
                self.scale_read_rate(1 + self.device.in_waiting / 1000)
            packed_packet = self._read_from_stream()
            assert isinstance(packed_packet, bytes)
            packet = Packet.from_bytes(packed_packet)
        except serial.SerialException as e:
            print(f"Error reading packet: {e}")
            return False
        except OSError:
            print(1, "Cannot read from serial device.")
            return False
        if not self._correct_type(packet):
            print(
                f"Received unexpected packet: {packet} (expected: {self._recv_T})",
            )
            return False
        packet = cast(RecvPackets, packet)
        await self.on_packet_received(packet)
        return True

    async def _process_buffer(self, _: rospy.timer.TimerEvent) -> None:
        if not self.is_open():
            return
        try:
            await self._read_packet()
        except Exception as e:
            print(f"Error reading recent packet: {e}")
            import traceback

            traceback.print_exc()

    @abc.abstractmethod
    async def on_packet_received(self, packet: RecvPackets) -> None:
        """
        Abstract method to be implemented by subclasses for handling packets
        sent by the physical electrical board.

        Arguments:
            packet (:class:`~.Packet`): The packet that is received.
        """
        pass
'''
