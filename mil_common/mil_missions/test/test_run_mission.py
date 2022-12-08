#!/usr/bin/env python3
import unittest

import rospy
from actionlib import TerminalState
from mil_missions_core import MissionClient


class RunMissionTest(unittest.TestCase):
    def __init__(self, *args):
        self.client = MissionClient()
        self.client.wait_for_server()
        super().__init__(*args)

    def test_run_mission(self):
        self.client.run_mission("PrintAndWait")
        self.client.wait_for_result()
        result = self.client.get_result()
        state = self.client.get_state()
        self.assertEqual(state, TerminalState.SUCCEEDED)
        self.assertTrue(result.success)
        self.assertEqual(result.parameters, "")
        self.assertEqual(result.result, "The darkness isn't so scary")

    def test_failing_mission(self):
        self.client.run_mission("FailingMission")
        self.client.wait_for_result()
        result = self.client.get_result()
        state = self.client.get_state()
        self.assertEqual(state, TerminalState.ABORTED)
        self.assertFalse(result.success)
        self.assertTrue(result.parameters)
        self.assertRegex(
            result.result, r"^In submission \<.*\>: This is an example error\!$"
        )


if __name__ == "__main__":
    import rostest

    rospy.init_node("run_mission_test")
    rostest.rosrun("mil_missions", "run_mission", RunMissionTest)
