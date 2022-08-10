#!/usr/bin/env python3
import struct

import rospy
from mil_misc_tools.serial_tools import SimulatedSerial


class SimulatedSabertooth2x12(SimulatedSerial):
    """
    Creates a simulated serial device representing the Sabertooth 2x12 regenerative
    motor driver.

    Implements :class:`mil_tools.SimulatedSerial`.
    """

    def __init__(self, *args, **kwargs):
        super().__init__()

    def write(self, data: bytes) -> None:
        """
        Handles byte data received by the simulated device. Verifies the checksum
        of the data received, and logs changes in the motor speeds set by the device.
        """
        if len(data) != 4:
            rospy.logerr("wrong packet size")
            return

        data: int  # Temporary variable + type overwrite
        address, command, data, checksum = struct.unpack("BBBB", data)
        checksum_verify = (address + command + data) & 127
        if checksum_verify != checksum:
            rospy.logerr(f"Invalid checksum. Is {checksum} should be {checksum_verify}")
            return
        if command == 0:
            rospy.loginfo(f"Setting motor1 to {data / 127.0}")
        elif command == 1:
            rospy.loginfo(f"Setting motor1 to {-data / 127.0}")
        elif command == 4:
            rospy.loginfo(f"Setting motor2 to {data / 127.0}")
        elif command == 5:
            rospy.loginfo(f"Setting motor2 to {-data / 127.0}")
        else:
            rospy.logwarn(f"Unrecognized command {command}")
