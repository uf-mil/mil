#!/usr/bin/env python3
import random
import sys
import unittest

import rospy
import rostest
from ros_alarms import AlarmBroadcaster, AlarmListener

PKG = "ros_alarms"
NAME = "server_tester"


class ClientTester(unittest.TestCase):
    def __init__(self, *args):
        rospy.init_node(NAME, anonymous=True)
        super().__init__(*args)

    """ Tests alarm client operations
    Creates some listeners and some broadcasters and tests various raising and clearing conditions
    Also tests various combination of parameters
    """

    def test_broadcaster_and_listener(self):
        """Simple broadcaster and listener tests, with arguments"""
        ab_a = AlarmBroadcaster("alarm_a")
        al_a = AlarmListener("alarm_a")

        al_b = AlarmListener("alarm_b")
        ab_b = AlarmBroadcaster("alarm_b")

        al_c = AlarmListener("alarm_c")
        ab_c = AlarmBroadcaster("alarm_c")

        # Make sure all the alarm start off clear
        self.assertFalse(al_a.is_raised())
        self.assertFalse(al_b.is_raised())
        self.assertFalse(al_b.is_raised())
        # Same as above
        self.assertTrue(al_a.is_cleared())
        self.assertTrue(al_b.is_cleared())
        self.assertTrue(al_c.is_cleared())

        # Some args to pass in
        _severity = 3
        _blank_params = ""
        _full_params = {"test": 1, "test2": 2}
        _problem_desc = "This is a test"

        # Raise them all, with some arguments
        ab_a.raise_alarm(severity=_severity)
        ab_b.raise_alarm(severity=_severity, parameters=_blank_params)
        ab_c.raise_alarm(problem_description=_problem_desc, parameters=_full_params)

        # Make sure all the alarm start off clear
        self.assertTrue(al_a.is_raised())
        self.assertTrue(al_b.is_raised())
        self.assertTrue(al_c.is_raised())

        # Make sure alarm values are correct
        self.assertEqual(al_a.get_alarm().severity, _severity)
        self.assertEqual(al_b.get_alarm().severity, _severity)
        self.assertEqual(al_b.get_alarm().parameters, _blank_params)
        self.assertEqual(al_c.get_alarm().problem_description, _problem_desc)
        self.assertEqual(al_c.get_alarm().parameters, _full_params)

        # Now clear the alarms, some again with arguments
        ab_a.clear_alarm()
        ab_b.clear_alarm(parameters=_full_params)
        ab_c.clear_alarm(parameters=_blank_params)

        # Make sure all alarms are cleared
        self.assertTrue(al_a.is_cleared())
        self.assertTrue(al_b.is_cleared())
        self.assertTrue(al_c.is_cleared())

        # Make sure arguments were passed correctly
        self.assertEqual(al_b.get_alarm().parameters, _full_params)
        self.assertEqual(al_c.get_alarm().parameters, _blank_params)

    def test_stress(self):
        """Stress test the server, lots of raises and clears"""
        ab_a = AlarmBroadcaster("alarm_a")
        al_a = AlarmListener("alarm_a")
        ab_b = AlarmBroadcaster("alarm_b")
        al_b = AlarmListener("alarm_b")
        al_c = AlarmListener("alarm_c")
        ab_c = AlarmBroadcaster("alarm_c")

        actions = [
            ab_a.raise_alarm,
            ab_b.raise_alarm,
            ab_c.raise_alarm,
            ab_a.clear_alarm,
            ab_b.clear_alarm,
            ab_c.clear_alarm,
        ]

        for i in range(100):
            random.choice(actions)()
            rospy.sleep(0.01)

        # Clear them all as a find state
        ab_a.clear_alarm()
        ab_b.raise_alarm()
        ab_c.raise_alarm()

        # Make sure all alarms are correct
        self.assertTrue(al_a.is_cleared())
        self.assertFalse(al_b.is_cleared())
        self.assertFalse(al_c.is_cleared())

        # Set everything cleared
        ab_b.clear_alarm()
        ab_c.clear_alarm()

        # Make sure of that
        self.assertTrue(al_b.is_cleared())
        self.assertTrue(al_c.is_cleared())

    def test_callbacks(self):
        al_a = AlarmListener("alarm_d")
        ab_a = AlarmBroadcaster("alarm_d")

        self.raised = False
        self.cleared = False
        self.both = False

        ab_a.clear_alarm()
        al_a.add_callback(self.raised_cb, call_when_cleared=False)
        al_a.add_callback(self.cleared_cb, call_when_raised=False)
        al_a.add_callback(self.both_cb)

        rospy.sleep(0.5)

        # Make sure callbacks are called on creation
        self.assertFalse(self.raised)
        self.assertTrue(self.cleared)
        self.assertTrue(self.both)

        self.raised = False
        self.cleared = False
        self.both = False

        # Make sure callbacks run when they're supposed to
        ab_a.raise_alarm()
        rospy.sleep(0.5)

        self.assertTrue(self.raised)
        self.assertFalse(self.cleared)
        self.assertTrue(self.both)

        self.raised = False
        self.cleared = False
        self.both = False

    def raised_cb(self, alarm):
        self.raised = True

    def cleared_cb(self, alarm):
        self.cleared = True

    def both_cb(self, alarm):
        self.both = True


if __name__ == "__main__":
    rostest.rosrun(PKG, NAME, ClientTester, sys.argv)
