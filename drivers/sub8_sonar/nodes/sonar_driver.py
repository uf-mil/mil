#!/usr/bin/env python
import rospy
import rosparam

import numpy as np

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
        self.locate_pulse_error_alarm = alarm_broadcaster.add_alarm(
            name='sonar_pulse_locating_error',
            action_required=False,
            severity=2
        )

        self.ser = serial.Serial(port=port, baudrate=baud, timeout=None)
        self.ser.flushInput()

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
            data = struct.unpack('<BffffH', response)

            return np.array([data[1], data[2], data[3], data[4]])
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
        [  0xAA  | 0x50 | 0xF5 ]
        '''
        self.ser.flushInput()

        message = struct.pack('B', HEADER)
        message += self.CRC(message)

        rospy.loginfo("Writing: %s" % binascii.hexlify(message))

        try:
            self.ser.write(message)
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
        return struct.pack('H', crc)

    def check_CRC(self, message):
        '''
        Given a message with a checksum as the last two bytes, this will return True or False
        if the checksum matches the given message.
        '''
        msg_checksum = message[-2:]
        raw_message = message[:-2]
        crc = crc16.crc16xmodem(raw_message, 0xFFFF)

        # If the two match the message was correct
        if crc == struct.unpack('H', msg_checksum)[0]:
            return True
        else:
            return False

class EchoLocator(object):
    '''
    Identifies the origin of a pulse in time and space given stamps of the time of detection of
    the pulse by individual sensors.
    '''
    def __init__(self, hydrophone_locations, wave_propagation_speed):  # speed in m/s
        self.hydrophone_locations = np.array()
        self.wave_propagation_speed = wave_propagation_speed
        for key in hydrophone_locations:
            sensor_location = np.array([hydrophone_locations[key]['x'], hydrophone_locations[key]['y'], hydrophone_locations[key]['z']])
            self.hydrophone_locations = np.vstack(self.hydrophone_locations, sensor_location)

    def getPulseLocation(self, timestamps):
        '''
        Returns a ros message with the location and time of emission of a pinger pulse.
        '''
        assert timestamps.size == self.hydrophone_locations.size[0]
        self.timestamps = timestamps
        init_guess = np.array([0,0,0,0])
        opt = {'disp': True}
        opt_method = 'Nelder-Mead'
        result = optimize.minimize(self._getCost, init_guess, method=opt_method, options=opt)
        pulse_location = result.x[:3]
        if(result.success):
            resp_data = HydroListenResponse()
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
        for row in xrange(self.hydrophone_locations.shape[0]):
            distance = np.sqrt(np.sum(np.square(potential_pulse[:3] - self.hydrophone_locations[row,:])))
            pulse_time_of_flight = distance / self.wave_propagation_speed
            cost = cost + self.timestamps[row] - potential_pulse[3] - pulse_time_of_flight
        return cost


if __name__ == "__main__":
    d = Sub8Sonar(rospy.get_param('~/sonar_driver/port'),
                  rospy.get_param('~/sonar_driver/baud'),
                  rospy.get_param('~/sonar_driver/hydrophones'))
