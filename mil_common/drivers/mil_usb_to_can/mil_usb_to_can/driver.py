#!/usr/bin/python3
import importlib
from typing import Any, Dict, Generator, Optional, Tuple

import rospy
from mil_usb_to_can.board import (  # relative import causes import error with rosrun - GH-731
    USBtoCANBoard,
)
from mil_usb_to_can.utils import USB2CANException
from serial import SerialException


class USBtoCANDriver:
    """
    ROS Driver which implements the USB to CAN board. Allow users to specify a dictionary of
    device handle classes to be loaded at runtime to handle communication with
    specific devices.

    Attributes:
        board (USBtoCANBoard): The board the driver is implementing.
        handles (Dict[int, Any]): The handles served by the driver. Each key represents
          a unique device ID, and each corresponding value represents an instance of
          a child class inheriting from :class:`CANDeviceHandle`. Upon initialization,
          each class is constructed after being parsed from dynamic reconfigure.
        timer (rospy.Timer): The timer controlling when buffers are processed.
    """

    def __init__(self):
        port = rospy.get_param("~port", "/dev/tty0")
        baud = rospy.get_param("~baudrate", 115200)
        can_id = rospy.get_param("~can_id", 0)
        simulation = rospy.get_param("/is_simulation", False)
        # If simulation mode, load simualted devices
        if simulation:
            rospy.logwarn(
                "CAN2USB driver in simulation! Will not talk to real hardware."
            )
            devices = dict(
                list(
                    self.parse_module_dictionary(rospy.get_param("~simulated_devices"))
                )
            )
            self.board = USBtoCANBoard(
                port=port,
                baud=baud,
                simulated=simulation,
                devices=devices,
                can_id=can_id,
            )
        else:
            self.board = USBtoCANBoard(port=port, baud=baud, simulated=simulation)

        # Add device handles from the modules specified in ROS params
        self.handles: Dict[int, Any] = dict(
            (device_id, cls(self, device_id))
            for device_id, cls in self.parse_module_dictionary(
                rospy.get_param("~device_handles")
            )
        )

        self.timer = rospy.Timer(rospy.Duration(1.0 / 20.0), self.process_in_buffer)

    def read_packet(self) -> bool:
        """
        Attempt to read a packet from the board. If the packet has an appropriate device
        handler, then the packet is passed to the ``on_data`` method of that handler.

        Returns:
            bool: The success in reading a packet.
        """
        try:
            packet = self.board.read_packet()
        except (SerialException, USB2CANException) as e:
            rospy.logerr("Error reading packet: {}".format(e))
            return False
        if packet is None:
            return False
        if packet.device in self.handles:
            self.handles[packet.device].on_data(packet.data)
        else:
            rospy.logwarn(
                "Message received for device {}, but no handle registered".format(
                    packet.device
                )
            )
        return True

    def process_in_buffer(self, *args) -> None:
        """
        Read all available packets in the board's in-buffer.
        """
        while self.read_packet():
            pass

    def send_data(self, *args, **kwargs) -> Optional[Exception]:
        """
        Sends data using the :meth:`USBtoCANBoard.send_data` method.

        Returns:
            Optional[Exception]: If data was sent successfully, nothing is returned.
            Otherwise, the exception that was raised in sending is returned.
        """
        try:
            self.board.send_data(*args, **kwargs)
            return None
        except (SerialException, USB2CANException) as e:
            rospy.logerr("Error writing packet: {}".format(e))
            return e

    @staticmethod
    def parse_module_dictionary(
        d: Dict[str, Any]
    ) -> Generator[Tuple[int, Any], None, None]:
        """
        Generator to load classes from module strings specified in a dictionary.
        Imports all found classes.

        Yields:
            Generator[Tuple[int, Any], None, None]: Yields tuples containing the device
            ID and the associated class.
        """
        for device_id, module_name in d.items():
            device_id = int(device_id)
            # Split module from class name
            module_name, cls = module_name.rsplit(".", 1)
            # import module
            module = importlib.import_module(module_name)
            # Yield a tuple (device_id, imported_class)
            yield device_id, getattr(module, cls)


if __name__ == "__main__":
    rospy.init_node("usb_to_can_driver")
    driver = USBtoCANDriver()
    rospy.spin()
