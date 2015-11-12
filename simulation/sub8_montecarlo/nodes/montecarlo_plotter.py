#!/usr/bin/env python
'''Node for quickly plotting monte-carlo results
'''
import rospy
from sub8_montecarlo_tools import VerifyController


if __name__ == '__main__':
    rospy.init_node('verify_pd')

    num_runs = rospy.get_param('runs', 4)
    time_limit = rospy.get_param('run_time', 10)
    do_plot = rospy.get_param('plot_results', False)

    verifier = VerifyController(num_runs, time_limit, do_plot=do_plot)
    verifier.run_loop()
    verifier.analyze_stability()
