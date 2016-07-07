#!/usr/bin/python
import rospy
import rosparam

import numpy as np
from scipy.signal import resample

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

# temp
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

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
        self.multilaterator = Multilaterator(hydrophone_locations, self.c, method) # speed of sound in m/s

        rospy.Service('~/sonar/get_pinger_pulse', Sonar, self.get_pulse_from_raw_data)
        print "\x1b[32mSub8 sonar driver initialized\x1b[0m"
        rospy.spin()

    @thread_lock(lock)
    def get_pulse_from_timestamps(self, srv):
        self.ser.flushInput()

        try:
            self.ser.write('A')
            print "Sent timestamp request..."
        except:  # Except only serial errors in the future.
            self.disconnection_alarm.raise_alarm(
                problem_description="Sonar board serial connection has been terminated."
            )
            return False
        return self.multilaterator.getPulseLocation(self.timestamp_listener())

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
        # request signals from hydrophone board
        self.ser.flushInput()
        try:
            self.ser.write('B')
            print "Sent raw data request..."
        except:  # Except only serial errors in the future.
            self.disconnection_alarm.raise_alarm(
                problem_description="Sonar board serial connection has been terminated."
            )
            return False
        return self.multilaterator.getPulseLocation(self.raw_data_listener())

    def raw_data_listener(self):
        '''
        Parse the response of the board.
        '''
        print "Listening..."

        # prepare arrays for storing signals
        signal_bias = 32767
        waves_recorded = 3
        samples_per_wave = 17.24
        upsample_scale = 3
        exp_recording_size = np.floor(samples_per_wave) * waves_recorded * upsample_scale - 1
        raw_signals = np.zeros([4, exp_recording_size], dtype=float)

        try:
            # read in start bit of packet
            timeout = 0
            start_bit = self.ser.read(1)
            # If start bit not read, keep trying
            while start_bit != '\xBB':
                start_bit = self.ser.read(1)
                timeout += 1
                if timeout > 600:
                    raise BufferError("Timeout: failed to read a start bit from sonar board")
            for elem in np.nditer(raw_signals, op_flags=['readwrite']):
                elem[...] = float(self.ser.read(5)) - signal_bias
        except BufferError as e:
            print e

        # Set how much of each signal to actually use for processing
        non_zero_end_idx = 57
        raw_signals = raw_signals[:, 0:non_zero_end_idx]

        # upsample the signals for successful cross-correlation
        upsampled_signals = [resample(x, 8*len(x)) for x in raw_signals]

        # scale waves so they all have the same variance
        equalized_signals = [x / np.std(x) for x in upsampled_signals]
        raw_signals = [x / np.std(x) for x in raw_signals]
        wave0_upsampled, wave1_upsampled, wave2_upsampled, wave3_upsampled = equalized_signals
        wave0, wave1, wave2, wave3 = raw_signals

        tup = np.arange(0, non_zero_end_idx, 0.125)
        tref = np.arange(0, non_zero_end_idx, 1)
        # t = np.arange(0, non_zero_end_idx, 1)
        # H1 = mpatches.Patch(color='blue', label='H1')
        # H2 = mpatches.Patch(color='green', label='H2')
        # H3 = mpatches.Patch(color='red', label='H3')
        # H0 = mpatches.Patch(color='cyan', label='H0')
        # plt.legend(handles=[H0,H1,H2,H3])
        plt.plot(tref,wave1,tref,wave2,tref,wave3,tref,wave0)
        plt.plot(tup,wave1_upsampled,tup,wave2_upsampled,tup,wave3_upsampled,tup,wave0_upsampled)
        plt.show()
        
        print "done!"
        return [0,0,0,0]




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