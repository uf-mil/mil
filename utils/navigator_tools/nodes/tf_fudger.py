#!/usr/bin/env python
from __future__ import division

import rospy
import tf
import cv2
import argparse
import sys
import numpy as np

rospy.init_node("tf_fudger", anonymous=True)
br = tf.TransformBroadcaster()

usage_msg = "Useful to test transforms between two frames."
desc_msg = "Pass the name of the parent frame and the child name as well as a an optional multiplier. \n\
At anypoint you can press q to switch between displaying a quaternion and an euler angle."

parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
parser.add_argument(dest='tf_parent',
                    help="Parent tf frame.")
parser.add_argument(dest='tf_child',
                    help="Child tf frame.")
parser.add_argument('--range', action='store', type=float, default=3,
                    help="Maximum range allowed for linear displacement (in meters)")

args = parser.parse_args(sys.argv[1:])

cv2.namedWindow('tf', cv2.WINDOW_NORMAL)

# slider max values
x_max = 1000
ang_max = 1000

# tf ranges (symmetric about zero)
x_range = args.range   # meters
ang_range = 2 * np.pi  # radians

# tf fudging resolution
x_res = x_range / x_max
ang_res = ang_range / ang_max

print "Distance resolution: {} meters\nAngular resolution: {} radians".format(x_res, ang_res)

cv2.createTrackbar("x", 'tf', 0, x_max, lambda x: x)
cv2.createTrackbar("y", 'tf', 0, x_max, lambda x: x)
cv2.createTrackbar("z", 'tf', 0, x_max, lambda x: x)

cv2.createTrackbar("roll", 'tf', 0, ang_max, lambda x: x)
cv2.createTrackbar("pitch", 'tf', 0, ang_max, lambda x: x)
cv2.createTrackbar("yaw", 'tf', 0, ang_max, lambda x: x)

toTfLin = lambda x: x * x_res - 0.5 * x_range
toTfAng = lambda x: x * ang_res - 0.5 * ang_range

p = q = None
q_mode = False

while not rospy.is_shutdown():
    x, y, z = toTfLin(cv2.getTrackbarPos("x", "tf")), toTfLin(cv2.getTrackbarPos("y", "tf")), toTfLin(cv2.getTrackbarPos("z", "tf"))
    p = (x, y, z)
    rpy = (toTfAng(cv2.getTrackbarPos("roll", "tf")), toTfAng(cv2.getTrackbarPos("pitch", "tf")), toTfAng(cv2.getTrackbarPos("yaw", "tf")))
    q = tf.transformations.quaternion_from_euler(*rpy)

    rpy_feedback = "xyz: {}, euler: {}".format([round(x, 5) for x in p], [round(np.degrees(x), 5) for x in rpy])
    q_feedback = "xyz: {},     q: {}".format([round(x, 5) for x in p], [round(x, 5) for x in q])
    print q_feedback if q_mode else rpy_feedback

    k = cv2.waitKey(10) & 0xFF
    if k == ord('q'):
        q_mode = not q_mode

print  "\nFinal TF:", p, q