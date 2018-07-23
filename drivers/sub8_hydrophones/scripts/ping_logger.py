#!/usr/bin/env python

import os
import numpy

import roslib
import rospy

from hydrophones.msg import Ping
from hydrophones import util

roslib.load_manifest('hydrophones')


rospy.init_node('ping_logger')
output_dir = rospy.get_param("~output_dir", "pings")
if not os.path.exists(output_dir):
    os.makedirs(output_dir)


def save_ping(ping):
    path = os.path.join(output_dir, str(ping.header.seq) + ".csv")
    samples = util.ping_to_samples(ping)
    numpy.savetxt(path, samples.transpose(), fmt='%d', delimiter=',')


sub = rospy.Subscriber('hydrophones/ping', Ping, save_ping)
rospy.spin()
