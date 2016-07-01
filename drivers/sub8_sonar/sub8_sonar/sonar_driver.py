#!/usr/bin/python

from __future__ import division
import math
import numpy as np

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
        print "Sub8 sonar driver initialized\nMultilateration algorithm: bancroft"
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
        self.c = c
        self.hydrophone_locations = []
        for key in hydrophone_locations:
            sensor_location = np.array([hydrophone_locations[key]['x'], hydrophone_locations[key]['y'], hydrophone_locations[key]['z']])
            self.hydrophone_locations += [sensor_location]

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
        res = estimate_pos(timestamps, self.hydrophone_locations)
        print res
        if len(res) == 1:
            source = res[0]
        elif len(res) == 2:
            source = [x for x in res if x[2] < 0]   # Assume that the source is below us
        else:
            source = [None, None, None]
        print source
        response = SonarResponse()
        response.x = source[0]
        response.y = source[1]
        response.z = source[2]
        return response

def quadratic(a, b, c):
    discriminant = b*b - 4*a*c
    if discriminant >= 0:
        first_times_a = (-b+math.copysign(math.sqrt(discriminant), -b))/2
        return [first_times_a/a, c/first_times_a]
    else:
        return []

def estimate_pos(reception_times, positions):
    assert len(positions) == len(reception_times)
    
    N = len(reception_times)
    assert N >= 4
    
    L = lambda a, b: a[0]*b[0] + a[1]*b[1] + a[2]*b[2] - a[3]*b[3]
    
    def get_B(delta):
        B = np.zeros((N, 4))
        for i in xrange(N):
            B[i] = np.concatenate([positions[i], [-reception_times[i]]]) + delta
        return B
    
    delta = min([.1*np.random.randn(4) for i in xrange(10)], key=lambda delta: np.linalg.cond(get_B(delta)))
    
    B = get_B(delta)
    a = np.array([0.5 * L(B[i], B[i]) for i in xrange(N)])
    e = np.ones(N)
    
    Bpe = np.linalg.lstsq(B, e)[0]
    Bpa = np.linalg.lstsq(B, a)[0]
    
    Lambdas = quadratic(
        L(Bpe, Bpe),
        2*(L(Bpa, Bpe) - 1),
        L(Bpa, Bpa))
    if not Lambdas: return []
    
    res = []
    for Lambda in Lambdas:
        u = Bpa + Lambda * Bpe
        position = u[:3] - delta[:3]
        time = u[3] + delta[3]
        if any(reception_times[i] < time for i in xrange(N)): continue
        res.append(position)
    
    return res

def delete_last_lines(n=1):
    for _ in range(n):
        sys.stdout.write(CURSOR_UP_ONE)
        sys.stdout.write(ERASE_LINE)

if __name__ == "__main__":
    d = Sub8Sonar(rospy.get_param('~/sonar_driver/hydrophones'),
                  "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH02X4IE-if00-port0",
                  19200)


# @apply
# def main():
#     np.random.seed(1)
    
#     r = 0.0254
#     positions = map(np.array, [(0, +r, 0), (-r, 0, 0), (r, 0, 0), (0, -r, 0)])
    
#     def valid(correct, solution):
#         return np.linalg.norm(correct - solution) < 1e-2
    
#     while True:
#         test_pos = 10*np.random.randn(3)
#         test_time = 10*np.random.randn()
        
#         reception_times = [test_time + np.linalg.norm(p - test_pos) for p in positions]
#         print 'problem:', test_pos, test_time
#         res = estimate_pos(reception_times, positions)
#         assert res
#         good = any(valid(test_pos, x) for x in res)
#         print 'result:', res
#         assert good
#         print