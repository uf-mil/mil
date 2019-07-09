#!/usr/bin/env python
import rospy
import unittest
from actionlib import TerminalState
from mil_missions_core import MissionClient


class RunMissionTest(unittest.TestCase):
    def __init__(self, *args):
        self.client = MissionClient()
        self.client.wait_for_server()
        super(RunMissionTest, self).__init__(*args)

    def test_run_mission(self):
        self.client.run_mission('PrintAndWait')
        self.client.wait_for_result()
        result = self.client.get_result()
        state = self.client.get_state()
        self.assertEqual(state, TerminalState.SUCCEEDED)
        self.assertTrue(result.success)
        self.assertEqual(result.parameters, '')
        self.assertEqual(result.result, 'The darkness isnt so scary')

if __name__ == '__main__':
    import rostest
    rospy.init_node('run_mission_test')
    rostest.rosrun('mil_missions', 'run_mission', RunMissionTest)
