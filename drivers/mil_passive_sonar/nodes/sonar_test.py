#!/usr/bin/env python
from __future__ import division
import numpy as np
import numpy.linalg as la
from scipy import optimize
import rospy
import rosparam
import random
from multilateration import Multilaterator, ReceiverArraySim, Pulse
import sys


if __name__ == '__main__':
    def print_green(str):
        print '\x1b[32m' +  str + '\x1b[0m'

    def error(obs, exp):
        # Interesting, faster, but not as accurate
        # alpha = np.arccos(np.clip(np.dot(obs/la.norm(obs),exp/la.norm(exp)),-1,1))*180/np.pi
        alpha = 2*np.arctan2(la.norm(la.norm(exp)*obs-la.norm(obs)*exp),la.norm(la.norm(exp)*obs+la.norm(obs)*exp))
        mag_error = 100 * (la.norm(obs) - la.norm(exp)) / la.norm(exp)
        return ('\x1b[31m' if (mag_error == -100) else "") + ("Errors:  directional=" + str(alpha) + "deg").ljust(42) \
            + ("magnitude=" + str(mag_error) + "%").ljust(20)

    def delete_last_lines(n=0):
        CURSOR_UP_ONE = '\x1b[1A'
        ERASE_LINE = '\x1b[2K'
        for _ in range(n):
            sys.stdout.write(CURSOR_UP_ONE)
            sys.stdout.write(ERASE_LINE)

    c = 1.484  # millimeters/microsecond
    hydrophone_locations = rospy.get_param('~/sonar_test/hydrophones')
    hydrophone_array = ReceiverArraySim(hydrophone_locations, c)
    sonar = Multilaterator(hydrophone_locations, c, 'LS')

    # # Simulate individual pulses (Debugging Jakes Board)
    # pulse = Pulse(-5251, -7620, 1470, 0)
    # tstamps = hydrophone_array.listen(pulse)
    # tstamps = tstamps - tstamps[0]
    # print_green(pulse.__repr__())
    # print "Perfect timestamps: (microseconds)\n\t", tstamps
    # res_msg = sonar.getPulseLocation(np.array(tstamps))
    # res = np.array([res_msg.x, res_msg.y, res_msg.z])
    # print "\t\x1b[33m".ljust(22) + error(res, pulse.position()) + "\x1b[0m"

    # pulses will be generated with inside a cube with side-length $(pulse_range) (mm)
    try:
        for h in range(3,8):
            # smallest cube will be a meter wide, largest will be 10 km wide
            pulse_range = 10**h  # in mm
            rand_args = [-pulse_range, pulse_range + 1]
            num_pulses = 10
            print "\n\x1b[1mGenerating " + str(num_pulses) + " pulses within a " \
                + str(2*pulse_range/1000) + " meters wide cube\x1b[0m\n"

            for i in range(num_pulses):
                pulse = Pulse(random.randrange(*rand_args),
                              random.randrange(*rand_args),
                              random.randrange(*rand_args), 0)
                tstamps = hydrophone_array.listen(pulse)
                tstamps = tstamps - tstamps[0]
                print_green(str(i).ljust(2) + str(pulse))
                print "Perfect timestamps: (microseconds)\n\t", tstamps
                res_msg = sonar.getPulseLocation(np.array(tstamps))
                delete_last_lines(4)  # more concise output
                res = np.array([res_msg.x, res_msg.y, res_msg.z])
                print "\t\x1b[33m".ljust(22) + error(res, pulse.position()) + "\x1b[0m"
                print "Progressively adding noise to timestamps..."

                for j in range(-5, 2):
                    sigma = 10**j
                    noisy_tstamps = [x + np.random.normal(0, sigma) for x in tstamps]
                    noisy_tstamps[0] = 0
                    print "Noisy timestamps:\n\t", noisy_tstamps
                    res_msg = sonar.getPulseLocation(np.array(noisy_tstamps))
                    res = np.array([res_msg.x, res_msg.y, res_msg.z])
                    delete_last_lines(4)  # more concise output
                    print "\t\x1b[33m" + ("sigma: " +  str(sigma)).ljust(16) \
                        + error(res, pulse.position()) + "\x1b[0m"
    except KeyboardInterrupt:
        print "\nAborting mutilateration tests prematurely"
