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
desc_msg = "Pass the name of the parent frame and the child name as well as a an optional multiplier."

parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
parser.add_argument(dest='tf_child',
                    help="Child tf frame.")
parser.add_argument(dest='tf_parent',
                    help="Parent tf frame.")
parser.add_argument('--mult', action='store', type=float, default=1,
                    help="A factor with which to muliply linear components of the transformations")

args = parser.parse_args(sys.argv[1:])

cv2.namedWindow('tf', cv2.WINDOW_NORMAL)

cv2.createTrackbar("x", 'tf', 0, 100, lambda x: x)
cv2.createTrackbar("y", 'tf', 0, 100, lambda x: x)
cv2.createTrackbar("z", 'tf', 0, 100, lambda x: x)

cv2.createTrackbar("roll", 'tf', 0, 628, lambda x: x)
cv2.createTrackbar("pitch", 'tf', 0, 628, lambda x: x)
cv2.createTrackbar("yaw", 'tf', 0, 628, lambda x: x)

while not rospy.is_shutdown():
    x, y, z = cv2.getTrackbarPos("x", "tf") / 100 - .5, cv2.getTrackbarPos("y", "tf") / 100 - .5, cv2.getTrackbarPos("z", "tf") / 100 - .5
    p = (x * args.mult, y * args.mult, z * args.mult)
    rpy = (cv2.getTrackbarPos("roll", "tf") / 100 - np.pi, cv2.getTrackbarPos("pitch", "tf") / 100 - np.pi, cv2.getTrackbarPos("yaw", "tf") / 100 - np.pi)
    q = tf.transformations.quaternion_from_euler(*rpy)

    print "xyz:", [round(x, 5) for x in p], "  rpy:", [round(np.degrees(x), 5) for x in rpy]
    br.sendTransform(p, q, rospy.Time.now(), args.tf_child, args.tf_parent)

    cv2.waitKey(10)

cv2.destroyAllWindows()