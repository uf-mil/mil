#!/usr/bin/env python
import rospy
from mil_misc_tools.serial_tools import SimulatedSerial
import struct


class SimulatedSabertooth2x12(SimulatedSerial):
    def __init__(self, *args, **kwargs):
        super(SimulatedSabertooth2x12, self).__init__()

    def write(self, data):
        if len(data) != 4:
            rospy.logerr('wrong packet size')
            return
        address, command, data, checksum = struct.unpack('BBBB', data)
        checksum_verify = (address + command + data) & 127
        if checksum_verify != checksum:
            rospy.logerr('Invalid checksum. Is {} should be {}'.format(checksum, checksum_verify))
            return
        if command == 0:
            rospy.loginfo('Setting motor1 to {}'.format(data / 127.0))
        elif command == 1:
            rospy.loginfo('Setting motor1 to {}'.format(-data / 127.0))
        elif command == 4:
            rospy.loginfo('Setting motor2 to {}'.format(data / 127.0))
        elif command == 5:
            rospy.loginfo('Setting motor2 to {}'.format(-data / 127.0))
        else:
            rospy.logwarn('Unrecognized command {}'.format(command))
