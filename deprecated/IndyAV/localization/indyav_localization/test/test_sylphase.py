#!/usr/bin/env python
import os
import unittest
from time import sleep

import rospy
import rostest
import tf
from nav_msgs.msg import Odometry


class SylphaseTestCase(unittest.TestCase):
    sylphase_pub = False
    sylphase_hz_rate = 0
    high_hz = False

    def callback(self, data):
        self.sylphase_hz_rate += 1

    def test_ins_odom_frequency(self):
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        rospy.Subscriber("/ins_odom", Odometry, self.callback)
        sleep(1)

        # tests if ins_odom is being published
        if self.sylphase_hz_rate > 0:
            self.sylphase_pub = True
        self.assertTrue(self.sylphase_pub)

        # tests if ins_odom is being published at a rate above 25 hz
        if self.sylphase_hz_rate > 25:
            self.high_hz = True
        self.assertTrue(self.high_hz)

    def test_base_link_enu_tf(self):
        # wait till the robot is spawned in gazebo
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        tf_listener = tf.TransformListener()
        tf_listener.waitForTransform(
            "base_link", "enu", rospy.Time(0), rospy.Duration(5.0)
        )


if __name__ == "__main__":
    rospy.init_node("test_sylphase")
    import rostest

    rostest.rosrun("test_package", "test_name", SylphaseTestCase)
    os.system("killgazebo")
