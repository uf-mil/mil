#!/usr/bin/env python
import unittest
import numpy as np
from sub8_ros_tools import rosmsg_to_numpy
from sub8_alarm import AlarmBroadcaster


class TestAlarmSystem(unittest.TestCase):
    def test_instantiate_alarm_broadcaster(self):
        '''Ensure that the alarm broadcaster instantiates without errors'''
        broadcaster = AlarmBroadcaster()
        self.assertIsNotNone(broadcaster)

    def test_add_alarm(self):
        '''Ensure that adding an alarm succeeds without errors'''
        broadcaster = AlarmBroadcaster()
        alarm = broadcaster.add_alarm(
            name='wake-up',
            action_required=True,
            severity=1,
            problem_description='This is a problem',
            parameters={"concern": ["a", "b", "c"]}
        )
        self.assertIsNotNone(alarm)


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestAlarmSystem)
    unittest.TextTestRunner(verbosity=2).run(suite)
