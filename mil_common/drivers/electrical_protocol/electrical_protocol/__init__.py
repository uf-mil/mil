"""
The :mod:`electrical_protocol` module is meant to serve as a bridge between
a software library and a physical electrical device. The library is not dependent
on a particular communication standard (UART, CAN, etc.) being used. Rather, the
library just provides a packet structure that can be used by any project, and a
list of packets using this structure.

The library also provides a simple driver for UART/serial communication with a
physical electrical device -- :class:`~.electrical_protocol.ROSSerialDevice`.
If a CAN standard is desired, the :class:`mil_usb_to_can.sub9.CANDeviceHandle`
class can be used, which supports packets built through this library.
"""

from .driver import ROSSerialDevice
from .packet import AckPacket, ChecksumException, NackPacket, Packet
