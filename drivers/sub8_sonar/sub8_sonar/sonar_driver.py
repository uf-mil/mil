#!/usr/bin/python
from __future__ import division
import math
import numpy as np
import numpy.linalg as la
from scipy import optimize
from itertools import combinations

import rospy
import rosparam

from sub8_msgs.srv import Sonar, SonarResponse
from sub8_ros_tools import thread_lock, make_header
from sub8_alarm import AlarmBroadcaster

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
    * Adapted from the actuator driver.

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

        rospy.Service('~/sonar/get_pinger_pulse', Sonar, self.request_data)
        print "\x1b[32mSub8 sonar driver initialized\x1b[0m"
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
    def request_data(self, srv):
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

class Multilaterator(object):
    '''
    Finds the origin location of a pulse given differential times of 
    arrival to the individual sensors. c is the wave speed in the medium of operation.
    Units:
        Hydrohone coordinates are expected in millimeters, pulse location will be given in millimeters.
        Timestamps are expected in microseconds. c is expected in millimeters per microsecond
    Note:
        hydrophone locations should be the dict returned by rospy.get_param('~/<node name>/hydrophones
    '''
    def __init__(self, hydrophone_locations, c, method):  # speed in millimeters/microsecond
        self.hydrophone_locations = []
        for key in hydrophone_locations:
            sensor_location = np.array([hydrophone_locations[key]['x'], hydrophone_locations[key]['y'], hydrophone_locations[key]['z']])
            self.hydrophone_locations += [sensor_location]
        self.pairs = list(combinations(range(len(hydrophone_locations)),2))
        self.c = c
        self.method = method
        print "\x1b[32mSpeed of Sound (c):", self.c, "millimeter/microsecond\x1b[0m"

    def getPulseLocation(self, timestamps, method=None):
        '''
        Returns a ros message with the location and time of emission of a pinger pulse.
        '''
        if method == None:
            method = self.method
        # print "\x1b[32mMultilateration algorithm:", method, "\x1b[0m"
        assert len(self.hydrophone_locations) == len(timestamps)
        source = None
        if method == 'bancroft':
            source = self.estimate_pos_bancroft(timestamps)
        elif method == 'LS':
            source = self.estimate_pos_LS(timestamps)
        else:
            print method, "is not an available multilateration algorithm"
            return
        response = SonarResponse()
        response.x = source[0]
        response.y = source[1]
        response.z = source[2]
        print "Reconstructed Pulse:\n\t" + "x: " + str(response.x) + " y: " + str(response.y) \
            + " z: " + str(response.z) + " (mm)"
        return response

    def estimate_pos_bancroft(self, reception_times):
        N = len(reception_times)
        assert N >= 4
        
        L = lambda a, b: a[0]*b[0] + a[1]*b[1] + a[2]*b[2] - a[3]*b[3]
        
        def get_B(delta):
            B = np.zeros((N, 4))
            for i in xrange(N):
                B[i] = np.concatenate([self.hydrophone_locations[i]/(self.c), [-reception_times[i]]]) + delta
            return B
        
        delta = min([.1*np.random.randn(4) for i in xrange(10)], key=lambda delta: np.linalg.cond(get_B(delta)))
        # delta = np.zeros(4) # gives very good heading for noisy timestamps, although range is completely unreliable

        B = get_B(delta)
        a = np.array([0.5 * L(B[i], B[i]) for i in xrange(N)])
        e = np.ones(N)
        
        Bpe = np.linalg.lstsq(B, e)[0]
        Bpa = np.linalg.lstsq(B, a)[0]
        
        Lambdas = quadratic(
            L(Bpe, Bpe),
            2*(L(Bpa, Bpe) - 1),
            L(Bpa, Bpa))
        if not Lambdas: 
            return [0, 0, 0]
        
        res = []
        for Lambda in Lambdas:
            u = Bpa + Lambda * Bpe
            position = u[:3] - delta[:3]
            time = u[3] + delta[3]
            if any(reception_times[i] < time for i in xrange(N)): continue
            res.append(position*self.c)
        if len(res) == 1:
            source = res[0]
        elif len(res) == 2:
            source = [x for x in res if x[2] < 0]   # Assume that the source is below us
            if not source: 
                source = res[0]
            else:
                source = source[0]
        else:
            source = [0, 0, 0]
        return source

    def estimate_pos_LS(self, timestamps):
        '''
        Returns a ros message with the location and time of emission of a pinger pulse.
        '''
        self.timestamps = timestamps
        init_guess = np.random.normal(0,100,3)
        opt = {'disp': 0}
        opt_method = 'Powell'
        result = optimize.minimize(self.cost_LS, init_guess, method=opt_method, options=opt, tol=1e-15)
        if(result.success):
            source = [result.x[0], result.x[1], result.x[2]]
        else:
            source = [0, 0, 0]
        return source

    def cost_LS(self, potential_pulse):
        '''
        Compares the difference in observed and theoretical difference in time of arrival
        between tevery unique pair of hydrophones.

        Note: the result will be along the direction of the heading but not at the right distance.
        '''
        cost = 0
        t = self.timestamps
        c = self.c
        x = np.array(potential_pulse)
        for pair in self.pairs:
            h0 = self.hydrophone_locations[pair[0]]
            h1 = self.hydrophone_locations[pair[1]]
            residual = la.norm(x-h0) - la.norm(x-h1) - c*(t[pair[0]] - t[pair[1]])
            cost += residual**2
        return cost

    def cost_LS2(self, potential_pulse):
        """
        Slightly less accurate than the one above in terms of heading but much faster.
        """
        cost = 0
        t = self.timestamps
        x0 = self.hydrophone_locations[0][0]
        y0 = self.hydrophone_locations[0][1]
        z0 = self.hydrophone_locations[0][2]
        x = potential_pulse[0]
        y = potential_pulse[1]
        z = potential_pulse[2]
        d0 = np.sqrt((x0 - x)**2 + (y0 - x)**2 + (z0 - x)**2)
        for i in xrange(1, len(self.hydrophone_locations)):
            xi = self.hydrophone_locations[i][0]
            yi = self.hydrophone_locations[i][1]
            zi = self.hydrophone_locations[i][2]
            di = np.sqrt((xi - x)**2 + (yi - x)**2 + (zi - x)**2)
            hydro_i_cost = (di - d0 - self.c * t[i])**2
            cost = cost + hydro_i_cost
        return cost


def quadratic(a, b, c):
    discriminant = b*b - 4*a*c
    if discriminant >= 0:
        first_times_a = (-b+math.copysign(math.sqrt(discriminant), -b))/2
        return [first_times_a/a, c/first_times_a]
    else:
        return []


def delete_last_lines(n=1):
    CURSOR_UP_ONE = '\x1b[1A'
    ERASE_LINE = '\x1b[2K'
    for _ in range(n):
        sys.stdout.write(CURSOR_UP_ONE)
        sys.stdout.write(ERASE_LINE)

if __name__ == "__main__":
    d = Sub8Sonar('bancroft', 1.484, rospy.get_param('~/sonar_driver/hydrophones'),
                  "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH02X4IE-if00-port0",
                  19200)