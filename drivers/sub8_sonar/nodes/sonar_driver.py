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
HEADER = 0xAA


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
        self.sonar_sensor = EchoLocator(hydrophone_locations, wave_propagation_speed=1484) # speed in m/s

        rospy.Service('~/sonar/get_pinger_pulse', Sonar, self.request_data)

        rospy.spin()

    def listener(self):
        '''
        Parse the response of the board.
        '''
        print "Listening..."

        # We've found a packet now disect it.
        response = self.ser.read(19)
        rospy.loginfo("Received: %s" % response)
        if self.check_CRC(response):
            print "Found!"
            # The checksum matches the data so split the response into each piece of data.
            # For info on what these letters mean: https://docs.python.org/2/library/struct.html#format-characters
            data = struct.unpack('>BffffH', response)

            return np.array([data[4], data[1], data[2], data[3] ])
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

        rospy.loginfo("Writing: %s" % binascii.hexlify(message))

        try:
            self.ser.write('A')
            print "Sent!"
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
    '''
    def __init__(self, hydrophone_locations, wave_propagation_speed):  # speed in m/s
        self.hydrophone_locations = np.array([1, 1, 1])  # just for apending, will not be included
        self.wave_propagation_speed = wave_propagation_speed
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
        print self.timestamps
        init_guess = np.array([0, 0, 0])
        for idx in range(1,4):
            if(timestamps[idx] > 0):
                init_guess = init_guess - self.hydrophone_locations[idx]
            else:
                init_guess = init_guess + self.hydrophone_locations[idx]
        print "Init guess:", init_guess
        opt = {'disp': True}
        opt_method = 'Nelder-Mead'
        result = optimize.minimize(self._getCost, init_guess, method=opt_method, options=opt)
        pulse_location = result.x[:3]
        if(result.success):
            resp_data = SonarResponse()
            resp_data.x = result.x[0]
            resp_data.y = result.x[1]
            resp_data.z = result.x[2]
            resp_data.t = -np.sqrt(np.sum(np.square(result.x))) / self.wave_propagation_speed
            return resp_data
        else:
            self.locate_pulse_error_alarm.raise_alarm(
                problem_description=("SciPy optimize, using method '" + opt_method 
                    + "', failed to converge on a pinger pulse location."),
                parameters={
                    'fault_info': {'data': result.message}
                }
            )
            return None


    def _getCost(self, potential_pulse):
        '''
        Compares the timestamps that would have been generated by $(potential_pulse) with the
        actual observed timestamps. Close matches generate low cost values.
        '''
        cost = 0
        dist_to_ref = np.sqrt(np.sum(np.square(potential_pulse - self.hydrophone_locations[0])))
        ref_time_of_flight = dist_to_ref / self.wave_propagation_speed
        for row in xrange(1, self.hydrophone_locations.shape[0]):
            distance = np.sqrt(np.sum(np.square(potential_pulse - self.hydrophone_locations[row])))
            pulse_time_of_flight = distance / self.wave_propagation_speed
            expected_tstamp = pulse_time_of_flight - ref_time_of_flight
            cost = cost + np.square(expected_tstamp - self.timestamps[row])
        # print "Cost:", cost, "Pulse:", potential_pulse
        return cost


def testFile(filename):
    '''
    Runs the trilateration algorithm on timestamps written to a file in the following format:
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


if __name__ == "__main__":
    # d = Sub8Sonar(rospy.get_param('~/sonar_driver/hydrophones'),
    #               rospy.get_param('~/sonar_driver/port'),
    #               rospy.get_param('~/sonar_driver/baud'))
    testFile("/home/santiago/bags/SonarTestData.txt")