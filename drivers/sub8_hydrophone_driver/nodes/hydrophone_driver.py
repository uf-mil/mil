#!/usr/bin/env python
import rospy
import rosparam

from sub8_msgs.srv import HydroListen, HydroListenResponse
from sub8_ros_tools import thread_lock, make_header
from sub8_alarm import AlarmBroadcaster

import threading
import serial
import binascii
import struct
import time
import crc16

lock = threading.Lock()
HEADER = 0xAA


class ActuatorDriver():
    '''
    Allows high level ros code to interface with Jake's Hydrophone board.
    * Adapted from the actuator driver.

    TODO: Add a function to try and reconnect to the serial port if we lose connection.
    '''
    def __init__(self, port, baud=9600):
        rospy.init_node("hydrophone_driver")

        alarm_broadcaster = AlarmBroadcaster()
        self.disconnection_alarm = alarm_broadcaster.add_alarm(
            name='hydrophone_board_disconnect',
            action_required=True,
            severity=0
        )
        self.packet_error_alarm = alarm_broadcaster.add_alarm(
            name='hydrophone_packet_error',
            action_required=False,
            severity=2
        )

        self.ser = serial.Serial(port=port, baudrate=baud, timeout=None)
        self.ser.flushInput()

        rospy.Service('~get_hydrophones', HydroListen, self.request_data)

        rospy.spin()

    def listener(self):
        '''
        Parse the response of the board.
        '''
        # We've found a packet now disect it.
        response = self.ser.read(19)
        if self.check_CRC(response):
            # The checksum matches the data so split the response into each piece of data.
            # For info on what these letters mean: https://docs.python.org/2/library/struct.html#format-characters
            data = struct.unpack('<BffffH', response)

            resp_data = HydroListenResponse()
            resp_data.header = make_header()
            resp_data.m0 = data[1]
            resp_data.m1 = data[2]
            resp_data.m2 = data[3]
            resp_data.m3 = data[4]
            return resp_data
        else:
            self.packet_error_alarm.raise_alarm(
                problem_description="Hydrophone board checksum error.",
                parameters={
                    'fault_info': {'data': response}
                }
            )
            return None

    @thread_lock(lock)
    def request_data(self, srv):
        '''
        A polling packet consists of only a header and checksum (CRC-16):
          HEADER     CHECKSUM
        [  0x77  | 0x40 | 0x26 ]
        '''
        self.ser.flushInput()

        message = HEADER
        message += CRC(message)

        rospy.loginfo("Writing: %s" % hex(message))

        try:
            self.ser.write(message)
        except:  # Except only serial errors in the future.
            self.disconnection_alarm.raise_alarm(
                problem_description="Hydrophone board serial connection has been terminated."
            )
            return False

        return self.listener

    def CRC(self, message):
        crc = crc16.crc16xmodem(message)
        return struct.pack('H', crc)

    def check_CRC(self, message):
        '''
        Given a message with a checksum as the last two bytes, this will return True or False if the checksum matches the
            given message.
        '''
        msg_checksum = message[-2:]
        raw_message = message[:-2]
        crc = crc16.crc16xmodem(raw_message)

        # If the two match the message was correct
        if crc == struct.unpack('H', msg_checksum)[0]:
            return True
        else:
            return False

if __name__ == "__main__":
    a = ActuatorDriver(rospy.get_param('~/actuator_driver/port'), 9600)
