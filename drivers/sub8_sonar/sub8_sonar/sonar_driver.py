#!/usr/bin/python
import rospy
import rosparam

from sub8_msgs.srv import Sonar, SonarResponse
from sub8_ros_tools import thread_lock, make_header
from sub8_alarm import AlarmBroadcaster
from multilateration import Multilaterator

import threading
import serial
import binascii
import struct
import time
import crc16
import sys

lock = threading.Lock()


class Sub8Sonar():
    '''
    Smart sensor that provides high level ROS code with the location of pinger pulses detected with
    Jake Easterling's Hydrophone board.

    TODO: Add a function to try and reconnect to the serial port if we lose connection.
    TODO: Express pulse location in map frame
    '''
    def __init__(self, method, c, hydrophone_locations, port, baud=19200):
        rospy.init_node("sonar")

        alarm_broadcaster = AlarmBroadcaster()
        self.disconnection_alarm = alarm_broadcaster.add_alarm(
            name='sonar_disconnect',
            action_required=True,
            severity=0
        )
        self.packet_error_alarm = alarm_broadcaster.add_alarm(
            name='sonar_packet_error',
            action_required=False,
            severity=2
        )

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=None)
            self.ser.flushInput()        
        except Exception, e:
            print "\x1b[31mSonar serial  connection error:\n\t", e, "\x1b[0m"
            return None

        self.c = c
        self.hydrophone_locations = hydrophone_locations
        self.sonar_sensor = Multilaterator(hydrophone_locations, self.c, method) # speed of sound in m/s

        rospy.Service('~/sonar/get_pinger_pulse', Sonar, self.get_pulse_from_timestamps)
        print "\x1b[32mSub8 sonar driver initialized\x1b[0m"
        rospy.spin()

    @thread_lock(lock)
    def get_pulse_from_timestamps(self, srv):
        self.ser.flushInput()

        try:
            self.ser.write('A')
            print "Sent request..."
        except:  # Except only serial errors in the future.
            self.disconnection_alarm.raise_alarm(
                problem_description="Sonar board serial connection has been terminated."
            )
            return False
        return self.sonar_sensor.getPulseLocation(self.timestamp_listener())

    def timestamp_listener(self):
        '''
        Parse the response of the board.
        '''
        print "Listening..."

        # We've found a packet now disect it.
        response = self.ser.read(19)
        # rospy.loginfo("Received: %s" % response) #uncomment for debugging
        if self.check_CRC(response):
            delete_last_lines(2) # Heard response!
            # The checksum matches the data so split the response into each piece of data.
            # For info on what these letters mean: https://docs.python.org/2/library/struct.html#format-characters
            data = struct.unpack('>BffffH', response)
            timestamps = [data[4], data[1], data[2], data[3] ]
            print "timestamps:", timestamps
            return timestamps
        else:
            self.packet_error_alarm.raise_alarm(
                problem_description="Sonar board checksum error.",
                parameters={
                    'fault_info': {'data': response}
                }
            )
            return None

    @thread_lock(lock)
    def get_pulse_from_raw_data(self, srv):
        self.ser.flushInput()

        try:
            self.ser.write('A')
            print "Sent request..."
        except:  # Except only serial errors in the future.
            self.disconnection_alarm.raise_alarm(
                problem_description="Sonar board serial connection has been terminated."
            )
            return False
        return self.sonar_sensor.getPulseLocation(self.listener())

    def raw_data_listener(self):
        '''
        Parse the response of the board.
        '''
        print "Listening..."

        # We've found a packet now disect it.
        response = self.ser.read(19)
        # rospy.loginfo("Received: %s" % response) #uncomment for debugging
        if self.check_CRC(response):
            delete_last_lines(2) # Heard response!
            # The checksum matches the data so split the response into each piece of data.
            # For info on what these letters mean: https://docs.python.org/2/library/struct.html#format-characters
            data = struct.unpack('>BffffH', response)
            timestamps = [data[4], data[1], data[2], data[3] ]
            print "timestamps:", timestamps
            return timestamps
        else:
            self.packet_error_alarm.raise_alarm(
                problem_description="Sonar board checksum error.",
                parameters={
                    'fault_info': {'data': response}
                }
            )
            return None

    def CRC(self, message):
        # You may have to change the checksum type.
        # Check the crc16 module online to see how to do that.
        crc = crc16.crc16xmodem(message, 0xFFFF)
        return struct.pack('>H', crc)

    def check_CRC(self, message):
        '''
        Given a message with a checksum as the last two bytes, this will return True or False
        if the checksum matches the given message.
        '''
        msg_checksum = message[-2:]
        raw_message = message[:-2]
        crc = crc16.crc16xmodem(raw_message, 0xFFFF)

        # If the two match the message was correct
        if crc == struct.unpack('>H', msg_checksum)[0]:
            return True
        else:
            return False

def delete_last_lines(n=1):
    CURSOR_UP_ONE = '\x1b[1A'
    ERASE_LINE = '\x1b[2K'
    for _ in range(n):
        sys.stdout.write(CURSOR_UP_ONE)
        sys.stdout.write(ERASE_LINE)

if __name__ == "__main__":
    d = Sub8Sonar('LS', 1.484, rospy.get_param('~/sonar_driver/hydrophones'),
                  "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH02X4IE-if00-port0",
                  19200)