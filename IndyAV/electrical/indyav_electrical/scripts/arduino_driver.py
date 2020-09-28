#!/usr/bin/env python
## This script reads lines from arduino serial communication and saves it to 'output.txt'
## Requires package: pyserial

# TODO rewrite this in a proper c++ ros sensor driver
# TODO: test with arduino again, small changes have been made

import os
import serial
import rospy
import numpy as np

from indyav_electrical.msg import SteeringPotStamped
from indyav_electrical.msg import ThrottleBrakeStamped
from indyav_electrical.msg import WheelEncodersStamped
from std_msgs.msg import Float64

serial = serial.Serial("/dev/ttyACM0",115200,timeout = 1);

def get_throttle_msg(x):
    if (x < 0 or x > 1):
        raise Exception("Acceptible range for servo is [0.0, 1.0]. Attempted: " + str(x))
    a = int(x*62) + 51
    msg = [np.uint8(a & 0xff), np.uint8((a & 0xff00) >> 8)]
    msg.append(np.uint8(msg[0] ^ msg[1]))
    msg = np.array(msg).tobytes()
    return msg


def set_throttle(msg):
    serial.write(get_throttle_msg(msg.data))


rospy.init_node('chassis_sensors_driver')

steering_pub = rospy.Publisher('/steering/meausred', SteeringPotStamped, queue_size=1)
throttle_pub = rospy.Publisher('/throttle/meausred', ThrottleBrakeStamped, queue_size=1)
wheel_pub = rospy.Publisher('/wheels/measured', WheelEncodersStamped, queue_size=1)
throttle_sub = rospy.Subscriber('/throttle', Float64, set_throttle)


def check_checksum(msg):
  c = msg[-1];
  crc = np.uint8(0)
  for i in msg[:-1]:
    crc = np.bitwise_xor(crc, i)
  return crc == c


while True:
    data = np.fromstring(serial.read(9), dtype=np.uint8)
    if len(data) == 0:
        continue
    if check_checksum(data) == False:
        continue


    data = np.array([(data[1] << 8) | data[0],
                     (data[3] << 8) | data[2],
                     (data[5] << 8) | data[4],
                     (data[7] << 8) | data[6]])


    steering_msg = SteeringPotStamped()
    steering_msg.header.stamp = rospy.Time.now()
    steering_msg.steering_pot = data[2]
    steering_pub.publish(steering_msg)

    throttle_msg = ThrottleBrakeStamped()
    throttle_msg.header.stamp = rospy.Time.now()
    throttle_msg.accelerator_pedal = data[3]
    throttle_msg.brake_pressure = data[0]
    throttle_pub.publish(throttle_msg)

    wheel_msg = WheelEncodersStamped()
    wheel_msg.header.stamp = rospy.Time.now()
    wheel_msg.back_wheel = data[1]
    wheel_pub.publish(wheel_msg)
