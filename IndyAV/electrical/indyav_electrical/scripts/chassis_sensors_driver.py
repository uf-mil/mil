#!/usr/bin/env python
## This script reads lines from arduino serial communication and saves it to 'output.txt'
## Requires package: pyserial

# TODO rewrite this in a proper c++ ros sensor driver

import serial
import rospy
import numpy as np

from indyav_electrical.msg import SteeringPotStamped
from indyav_electrical.msg import ThrottleBrakeStamped
from indyav_electrical.msg import WheelEncodersStamped

rospy.init_node('chassis_sensors_driver')

steering_pub = rospy.Publisher('/steering/meausred', SteeringPotStamped)
throttle_pub = rospy.Publisher('/throttle/meausred', ThrottleBrakeStamped)
wheel_pub = rospy.Publisher('/wheels/measured', WheelEncodersStamped)

serial = serial.Serial("/dev/ttyACM0",115200,timeout = 1);
print(serial);

# Serial readline loop
'''
with open('output.csv', 'w') as new_file:
        csv_writer = csv.writer(new_file,delimiter = ',')
        csv_writer.writerow(["time","steeringPot","breakPed","accPed","brakePressure",
                             "rearSpeed","passSpeed","driverSpeed"])
'''
while True:
    data = np.array(serial.readline().strip().split(','))
    data = data.astype(np.int64)

    print(data);
    steering_msg = SteeringPotStamped()
    steering_msg.header.stamp = rospy.Time.now()
    steering_msg.steering_pot = data[1]
    steering_pub.publish(steering_msg)

    throttle_msg = ThrottleBrakeStamped()
    throttle_msg.header.stamp = rospy.Time.now()
    throttle_msg.accelerator_pedal = data[3]
    throttle_msg.brake_pedal = data[2]
    throttle_msg.brake_pressure = data[4]
    throttle_pub.publish(throttle_msg)

    wheel_msg = WheelEncodersStamped()
    wheel_msg.header.stamp = rospy.Time.now()
    wheel_msg.back_wheel = data[5]
    wheel_msg.left_wheel = data[7]
    wheel_msg.right_wheel = data[6]
    wheel_pub.publish(wheel_msg)
