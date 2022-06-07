#!/usr/bin/env python3
import rospy
from mil_misc_tools.serial_tools import SimulatedSerial

from .constants import Constants


class SimulatedPnuematicActuatorBoard(SimulatedSerial):
    """
    A simulation of the pneumatic actuator board's serial protocol
    """

    def __init__(self, *args, **kwargs):
        super(SimulatedPnuematicActuatorBoard, self).__init__()

    def write(self, data: bytes):
        """
        Writes a series of bytes and completes the necessary actions.
        """
        request = Constants.deserialize_packet(data)
        request = request[0]
        if request == Constants.PING_REQUEST:
            rospy.loginfo("Ping received")
            byte = Constants.PING_RESPONSE
        elif request > 0x20 and request < 0x30:
            rospy.loginfo("Open port {}".format(request - 0x20))
            byte = Constants.OPEN_RESPONSE
        elif request > 0x30 and request < 0x40:
            rospy.loginfo("Close port {}".format(request - 0x30))
            byte = Constants.CLOSE_RESPONSE
        else:
            rospy.loginfo("Default")
            byte = 0x00
        self.buffer += Constants.serialize_packet(byte)
        return len(data)
