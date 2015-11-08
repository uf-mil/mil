#!/usr/bin/env python

PKG = 'sub8_montecarlo'
NAME = 'test_controller'

import sys
import unittest
import rospy
import rostest
from sub8_montecarlo_tools import VerifyController


class TestController(unittest.TestCase):
    def setUp(self, *args):
        '''TODO:
            - Assert that wrenches within bounds are close to the unbounded least-squares estimate
            - Make waiting for node functionality into a test_helpers util
            - Test random wrenches and assert valid behavior
        '''
        self.num_runs = rospy.get_param('runs', 4)
        self.time_limit = rospy.get_param('run_time', 10)

    def test_pd_controller(self):
        # pretty loose for a crappy controller
        criteria = {
            'min_envelope': 0.7,
            'max_enevelope': 1.1,
        }
        # Of course we don't want to plot!
        vc = VerifyController(self.num_runs, self.time_limit, do_plot=False)
        vc.run_loop()
        averages = vc.analyze_stability()
        avg_min, avg_max_envelope = averages[0], averages[1]
        self.assertLess(
            avg_min, criteria['min_envelope'],
            msg='PD Controller did not satisy bounds for minimum, wanted {}, got {}'.format(
                criteria['min_envelope'],
                avg_min
            )
        )
        self.assertLess(
            avg_max_envelope, criteria['max_enevelope'],
            msg='PD Controller did not satisy bounds for maximum convergence error, wanted {}, got {}'.format(
                criteria['max_enevelope'],
                avg_max_envelope
            )
        )


if __name__ == '__main__':
    rospy.init_node(NAME, anonymous=True)
    rostest.rosrun(PKG, NAME, TestController, sys.argv)
