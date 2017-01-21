#!/usr/bin/env python

PKG = 'sub8_thruster_mapper'
NAME = 'test_map'

import sys
import unittest
import numpy as np
from sub8_msgs.msg import Thrust, ThrusterCmd
from sub8_msgs.srv import ThrusterInfo, ThrusterInfoResponse
from geometry_msgs.msg import WrenchStamped, Wrench, Vector3
from sub8_ros_tools import wait_for_subscriber
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
        thrust_service = rospy.Service('thrusters/thruster_range', ThrusterInfo, self.get_thruster_info)

    def thrust_callback(self, msg):
        self.got_msg = True
        self.test_data.append(msg)

    def get_thruster_info(self, srv):
        query_id = srv.thruster_id
        min_thrust = -100
        max_thrust = 90
        thruster_info = ThrusterInfoResponse(
            min_force=min_thrust,
            max_force=max_thrust
        )
        return thruster_info

    def test_map_good(self):
        '''Test desired wrenches that are known to be achievable
        '''
        target_node = 'thruster_mapper'
        target_topic = '/wrench'
        subscribed = wait_for_subscriber(target_node, target_topic)
        self.assertTrue(
            subscribed,
            "{} did not not come up in time".format(target_node)
        )

        thrust_pub = rospy.Publisher('/wrench', WrenchStamped, queue_size=1, latch=True)
        wrenches = [
            np.zeros(6),
            np.arange(6) * 5,
            np.arange(6)[::-1] * 5,
            np.arange(6) * -5,
            np.arange(6)[::-1] * -5,
            np.arange(6)[::-1] * -100,
            np.arange(6)[::-1] * 100,
        ]

        rospy.Subscriber("/thrusters/thrust", Thrust, self.thrust_callback)
        time.sleep(0.1)  # Wait for the Subscriber to 'wake up'

        for num, wrench in enumerate(wrenches):
            wrench_msg = WrenchStamped(
                wrench=Wrench(
                    force=Vector3(*wrench[:3]),
                    torque=Vector3(*wrench[3:])
                )
            )

            thrust_pub.publish(wrench_msg)
            timeout_t = time.time() + 0.01
            while not rospy.is_shutdown() and time.time() < timeout_t and not self.got_msg:
                time.sleep(0.01)

            self.assertEqual(len(self.test_data) - 1, num, msg="Could not compute wrench for " + str(wrench) + " within timeout")
            self.got_msg = False
            rospy.sleep(0.06)  # Wait the timeout period


if __name__ == '__main__':
    rospy.init_node(NAME, anonymous=True)
    rostest.rosrun(PKG, NAME, TestMapThrusters, sys.argv)
