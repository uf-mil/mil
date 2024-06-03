#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mil_misc_tools.serial_tools import SimulatedSerial

from .constants import Constants


class SimulatedPnuematicActuatorBoard(SimulatedSerial, Node):
    """
    A simulation of the pneumatic actuator board's serial protocol
    """

    def __init__(self, *args, **kwargs):
        Node.__init__(self, 'simulated_pneumatic_actuator_board')
        SimulatedSerial.__init__(self)

    def write(self, data: bytes):
        """
        Writes a series of bytes and completes the necessary actions.
        """
        request = Constants.deserialize_packet(data)
        request = request[0]
        if request == Constants.PING_REQUEST:
            # self.get_logger().info("Ping received")
            byte = Constants.PING_RESPONSE
        elif 0x20 < request < 0x30:
            self.get_logger().info(f"Open port {request - 0x20}")
            byte = Constants.OPEN_RESPONSE
        elif 0x30 < request < 0x40:
            self.get_logger().info(f"Close port {request - 0x30}")
            byte = Constants.CLOSE_RESPONSE
        else:
            self.get_logger().info("Default")
            byte = 0x00
        self.buffer += Constants.serialize_packet(byte)
        return len(data)
