#!/usr/bin/python3
"""
The ``mil_usb_to_can`` package implements a driver and helper class for the
USB to CAN driver board. The package provides full functionality for communication
with the board, along with helper classes to provide a better, more structured use
of data being sent to and received from the board.

To launch the driver, start ``driver.py``, an executable Python file. This file
will spin up a driver and interface to the board. If you are starting the driver
from a launch file, you can additionally provide information for the embedded
:class:`USBtoCANBoard` class, which handles connecting to the board. This information
can be provided through local ROS parameters:

.. code-block:: xml

    <launch>
      <node pkg="mil_usb_to_can" type="driver.py" name="usb_to_can_driver">
        <rosparam command="delete" />
        <rosparam>
        # Path of serial device (default: "/dev/tty0")
        port: /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A800GHCF-if00-port0
        # Baudrate of device (default: 115200)
        baudrate: 115200
        # The CAN device id of the board (default: 0)
        can_id: 0
        # List of python device handle classes (list of device id: python class implementation)
        device_handles:
          "18": sub8_thrust_and_kill_board.ThrusterAndKillBoard
          "83": sub_actuator_board.ActuatorBoard
        simulated_devices:
          "18": sub8_thrust_and_kill_board.ThrusterAndKillBoardSimulation
          "83": sub_actuator_board.ActuatorBoardSimulation
        </rosparam>
      </node>
    </launch>

Many of the parameters shown are used to connect the driver to the physical board.
However, the final two parameters, ``device_handles`` and ``simulated_devices``
are used to create device handles for specific devices. These device handles can
send and receive data from the board, and many be used to do both, or one or the
other.

As suggested in the above XML, the package implements drivers for a physical run,
as well as a simulated run. Whether the simulated drivers are used is controlled
by the global ``/is_simulation`` parameter. The physical drivers implement
:class:`CANDeviceHandle`, whereas the simulated drivers implement from
:class:`SimulatedCANDevice`. More information on writing device handles can be
found in the documentation of each type of class.
"""

from .application_packet import (
    ApplicationPacket,
    ApplicationPacketWrongIdentifierException,
)
from .board import USBtoCANBoard
from .device import CANDeviceHandle, ExampleAdderDeviceHandle, ExampleEchoDeviceHandle
from .simulation import (
    ExampleSimulatedAdderDevice,
    ExampleSimulatedEchoDevice,
    SimulatedCANDevice,
    SimulatedUSBtoCAN,
)
from .sub8_driver import USBtoCANDriver
from .utils import (
    ChecksumException,
    CommandPacket,
    InvalidEndFlagException,
    InvalidFlagException,
    InvalidStartFlagException,
    Packet,
    PayloadTooLargeException,
    ReceivePacket,
    USB2CANException,
)
