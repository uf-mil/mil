#!/usr/bin/env python

PKG = 'sub8_thruster_mapper'
NAME = 'test_map'

import sys
import unittest
import numpy as np
from sub8_msgs.msg import Thrust, ThrusterCmd
from geometry_msgs.msg import WrenchStamped, Wrench, Vector3
import rospy
import rostest
import time


class TestMapThrusters(unittest.TestCase):
    def setUp(self, *args):
        '''TODO:
            - Assert that wrenches within bounds are close to the unbounded least-squares estimate
            - Make waiting for node functionality into a test_helpers util
            - Test random wrenches and assert valid behavior
        '''
        self.got_msg = False
        self.test_data = []

    def thrust_callback(self, msg):
        self.got_msg = True
        self.test_data.append(msg)

    def test_map_good(self):
        '''Test desired wrenches that are known to be achievable
        '''
        # wait at most 5 seconds for the thruster_mapper to be registered
        timeout_t = time.time() + 5.0
        while not (rostest.is_subscriber(
            rospy.resolve_name('/wrench'),
            rospy.resolve_name('thruster_mapper')) and time.time() < timeout_t):
            time.sleep(0.1)

        self.assert_(
            rostest.is_subscriber(
                rospy.resolve_name('/wrench'),
                rospy.resolve_name('thruster_mapper')
            ), 
            "{} is not up".format('thruster_mapper')
        )

        thrust_pub = rospy.Publisher('/wrench', WrenchStamped, queue_size=1, latch=True)
        wrenches = [
            np.zeros(6),
            np.arange(6) * 5,
            np.arange(6)[::-1] * 5,
            np.arange(6) * -5,
            np.arange(6)[::-1] * -5,
        ]
        rospy.Subscriber("/thrust", Thrust, self.thrust_callback)

        for num, wrench in enumerate(wrenches):
            wrench_msg = WrenchStamped(
                wrench=Wrench(
                    force=Vector3(*wrench[:3]),
                    torque=Vector3(*wrench[3:])
                )
            )

            thrust_pub.publish(wrench_msg)
            timeout_t = time.time() + 0.2
            while not rospy.is_shutdown() and time.time() < timeout_t and not self.got_msg:
                time.sleep(0.01)

            self.assertEqual(len(self.test_data) - 1, num, msg="Could not compute wrench for " + str(wrench) + " within timeout")
            self.got_msg = False


if __name__ == '__main__':
    rospy.init_node(NAME, anonymous=True)
    rostest.rosrun(PKG, NAME, TestMapThrusters, sys.argv)