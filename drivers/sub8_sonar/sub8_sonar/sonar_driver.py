#!/usr/bin/env python
import rospy
import rosparam

import numpy as np
from scipy import optimize

from sub8_msgs.srv import Sonar, SonarResponse
from sub8_ros_tools import thread_lock, make_header
from sub8_alarm import AlarmBroadcaster

import threading
import serial
import binascii
import struct
import time
import crc16

lock = threading.Lock()

import sys
CURSOR_UP_ONE = '\x1b[1A'
ERASE_LINE = '\x1b[2K'

class Sub8Sonar():
    '''
    Smart sensor that provides high level ROS code with the location of pinger pulses detected with
    Jake Easterling's Hydrophone board.
    * Adapted from the actuator driver.

    TODO: Add a function to try and reconnect to the serial port if we lose connection.
    TODO: Express pulse location in map frame
    '''
    def __init__(self, hydrophone_locations, port, baud=9600):
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

        self.hydrophone_locations = hydrophone_locations
        self.sonar_sensor = EchoLocator(hydrophone_locations, c=1484) # speed of sound in m/s

        rospy.Service('~/sonar/get_pinger_pulse', Sonar, self.request_data)
        rospy.spin()

    def listener(self):
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
            timestamps = np.array([data[4], data[1], data[2], data[3] ])
            print timestamps
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
    def request_data(self, srv):
        '''
        A polling packet consists of only a header and checksum (CRC-16):
          HEADER     CHECKSUM
        [  0x41  |  ]
        '''
        self.ser.flushInput()

        message = struct.pack('B', 0x41)
        message += self.CRC(message)

        # rospy.loginfo("Writing: %s" % binascii.hexlify(message)) # uncomment for debugging

        try:
            self.ser.write('A')
            print "Sent request..."
        except:  # Except only serial errors in the future.
            self.disconnection_alarm.raise_alarm(
                problem_description="Sonar board serial connection has been terminated."
            )
            return False
        return self.sonar_sensor.getPulseLocation(self.listener())

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

class EchoLocator(object):
    '''
    Identifies the origin of a pulse in time and space given stamps of the time of detection of
    the pulse by individual sensors.
    c is the wave propagation speed in the medium of operation
    '''
    # hydrophone locations should be the dict returned by rospy.get_param('~/<node name>/hydrophones
    def __init__(self, hydrophone_locations, c):  # speed in m/s
        self.hydrophone_locations = np.array([1, 1, 1])  # just for apending, will not be included
        self.c = c
        for key in hydrophone_locations:
            sensor_location = np.array([hydrophone_locations[key]['x'], hydrophone_locations[key]['y'], hydrophone_locations[key]['z']])
            self.hydrophone_locations = np.vstack((self.hydrophone_locations, sensor_location))
        self.hydrophone_locations = self.hydrophone_locations[1:]

        alarm_broadcaster = AlarmBroadcaster()
        self.locate_pulse_error_alarm = alarm_broadcaster.add_alarm(
            name='sonar_pulse_locating_error',
            action_required=False,
            severity=2
        )

    def getPulseLocation(self, timestamps):
        '''
        Returns a ros message with the location and time of emission of a pinger pulse.
        '''
        assert timestamps.size == self.hydrophone_locations.shape[0]
        self.timestamps = timestamps
        init_guess = np.array([1, 1, 1])
        opt = {'disp': 1}
        opt_method = 'Powell'
        result = optimize.minimize(self._getCost, init_guess, method=opt_method, options=opt, tol=1e-15)
        print result.message
        resp_data = SonarResponse()
        if(result.success):
            resp_data.x = result.x[0]
            resp_data.y = result.x[1]
            resp_data.z = result.x[2]
        else:
            resp_data.x = 0
            resp_data.y = 0
            resp_data.z = 0
            self.locate_pulse_error_alarm.raise_alarm(
                problem_description=("SciPy optimize, using method '" + opt_method 
                    + "', failed to converge on a pinger pulse location."),
                parameters={
                    'fault_info': {'data': result.message}
                }
            )
        return resp_data

    def _getCost(self, potential_pulse):
        '''
        Compares the difference in observed and theoretical difference in time of arrival
        between the hydrophones and the reference hydrophone for potential source origins.

        Note: when there are 4 timestamps (not including reference), this cost is convex 
            SciPy Optimize will converge on the correct source origin.
            With only 3 time stamps, minimization methods will not convrge to the correct
            result, however, repeating the process for the source in the same location,
            all of the results from minimizing this cost lie on a single 3D line
        '''
        cost = 0
        t = self.timestamps
        x0 = self.hydrophone_locations[0,0]
        y0 = self.hydrophone_locations[0,1]
        z0 = self.hydrophone_locations[0,2]
        x = potential_pulse[0]
        y = potential_pulse[1]
        z = potential_pulse[2]
        d0 = np.sqrt((x0 - x)**2 + (y0 - x)**2 + (z0 - x)**2)
        for i in xrange(1, self.hydrophone_locations.shape[0]):
            xi = self.hydrophone_locations[i,0]
            yi = self.hydrophone_locations[i,1]
            zi = self.hydrophone_locations[i,2]
            di = np.sqrt((xi - x)**2 + (yi - x)**2 + (zi - x)**2)
            hydro_i_cost = (di - d0 - self.c * t[i])**2
            cost = cost + hydro_i_cost
        return cost

def testFile(filename):
    '''
    Runs the multilateration algorithm on timestamps written to a file in the following format:
    [ ref_tstamp tstamp1 tstamp2 tstamp3 ]
    lines that do not begin with '[' are ignored
    '''
    hydrophone_locations = rospy.get_param('~/sonar_driver/hydrophones') #DBG
    locator = EchoLocator(hydrophone_locations, 1484)
    with open(filename, "r") as data_file:
        for line in data_file:
            if line[0] == '[':
                words =line.split()
                timestamps = []
                for word in words:
                    if word[0] != "[" and word[0] != "]":
                        timestamps += [eval(word)]
                print locator.getPulseLocation(np.array(timestamps)), "\n"

def delete_last_lines(n=1):
    for _ in range(n):
        sys.stdout.write(CURSOR_UP_ONE)
        sys.stdout.write(ERASE_LINE)


if __name__ == "__main__":
    d = Sub8Sonar(rospy.get_param('~/sonar_driver/hydrophones'),
                  "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH02X4IE-if00-port0",
                  19200)
    # testFile("/home/santiago/bags/SonarTestData.txt")
