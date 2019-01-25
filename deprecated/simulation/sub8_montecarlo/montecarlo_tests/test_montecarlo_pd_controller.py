#!/usr/bin/env python

import sys
import unittest
import rospy
import rostest
from sub8_montecarlo_tools import VerifyController

PKG = 'sub8_montecarlo'
NAME = 'test_controller'


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
        '''Verify that the controller_verification tool *works* and produces useful results
            Note: This is *absolutely not* intended to prove that the demo PD controller works or is useful in any way
                It is *entirely* to prove that the monte-carlo tool works

        Behavior:
            1. Run N tests (Settable via parameter) using montecarlo controller verification
            2. Verify that it fails on some ridiculous requirements
            3. And succeeds on some super easy ones
        '''

        criteria = {
            'impossible_min_envelope': 0.2,
            'impossible_max_enevelope': 0.3,
            'achievable_min_envelope': 1.5,
            'achievable_max_enevelope': 2.0,
        }
        # Of course we don't want to plot!
        vc = VerifyController(self.num_runs, self.time_limit, do_plot=False)
        vc.run_loop()
        averages = vc.analyze_stability()
        avg_min, avg_max_envelope = averages[0], averages[1]

        # Verify that the controller fails when given ridiculous requirements
        self.assertGreater(
            avg_min, criteria['impossible_min_envelope'],
            msg='VerifyController reported nonsense performance, expected {}, got {}'.format(
                criteria['impossible_min_envelope'],
                avg_min
            )
        )
        # Verify again that the controller fails
        self.assertGreater(
            avg_max_envelope, criteria['impossible_max_enevelope'],
            msg='VerifyController reported nonsense performance, expected {}, got {}'.format(
                criteria['impossible_max_enevelope'],
                avg_max_envelope
            )
        )

        # Now check that the controller could satisfy super easy bounds
        self.assertLess(
            avg_min, criteria['achievable_min_envelope'],
            msg='PD Controller did not satisy bounds for minimum, wanted {}, got {}'.format(
                criteria['achievable_min_envelope'],
                avg_min
            )
        )

        # And again
        self.assertLess(
            avg_max_envelope, criteria['achievable_max_enevelope'],
            msg='PD Controller did not satisy bounds for maximum convergence error, wanted {}, got {}'.format(
                criteria['achievable_max_enevelope'],
                avg_max_envelope
            )
        )


if __name__ == '__main__':
    rospy.init_node(NAME, anonymous=True)
    rostest.rosrun(PKG, NAME, TestController, sys.argv)
