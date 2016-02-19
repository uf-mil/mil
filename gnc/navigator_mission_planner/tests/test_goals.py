#!/usr/bin/env python

import unittest
import rospy
from navigator_msgs.msg import Waypoint
from navigator_mission_planner import goal_proxy

rospy.init_node("goal_client_test", anonymous=True)


class TestROSTools(unittest.TestCase):

    def test_down(self):
        boat = goal_proxy(testing=True)
        to_send = Waypoint()
        to_send.pose.position.z -= 10
        test = boat.move_down(10)
        self.assertEqual(test, to_send, "Wsaypoint down test failed")

    def test_up(self):
        boat = goal_proxy(testing=True)
        to_send = Waypoint()
        to_send.pose.position.z += 10
        test = boat.move_up(10)
        self.assertEqual(test, to_send, "Waypoint up test failed")

    def test_left(self):
        boat = goal_proxy(testing=True)
        to_send = Waypoint()
        to_send.pose.position.y -= 10
        test = boat.move_left(10)
        self.assertEqual(test, to_send, "Waypoint left test failed")

    def test_right(self):
        boat = goal_proxy(testing=True)
        to_send = Waypoint()
        to_send.pose.position.y += 10
        test = boat.move_right(10)
        self.assertEqual(test, to_send, "Waypoint right test failed")

    def test_forward(self):
        boat = goal_proxy(testing=True)
        to_send = Waypoint()
        to_send.pose.position.x += 10
        test = boat.move_forward(10)
        self.assertEqual(test, to_send, "Waypoint forward test failed")

    def test_back(self):
        boat = goal_proxy(testing=True)
        to_send = Waypoint()
        to_send.pose.position.x -= 10
        test = boat.move_back(10)
        self.assertEqual(test, to_send, "Waypoint back test failed")


if __name__ == '__main__':
    
    suite = unittest.TestLoader().loadTestsFromTestCase(TestROSTools)
    unittest.TextTestRunner(verbosity=2).run(suite)
