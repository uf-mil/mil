#!/usr/bin/env python
import numpy as np
import rospy
import rosparam
import random
from sonar_driver import EchoLocator

class SonarSim(object):
    def __init__(self, hydrophone_locations, wave_propagation_speed_mps):
        self.c = wave_propagation_speed_mps
        self.hydrophone_locations = np.array([0, 0, 0])
        for key in hydrophone_locations:
            sensor_location = np.array([hydrophone_locations[key]['x'], hydrophone_locations[key]['y'], hydrophone_locations[key]['z']])
            self.hydrophone_locations = np.vstack((self.hydrophone_locations, sensor_location))
        self.hydrophone_locations = self.hydrophone_locations[1:]

    def listen(self, pulse):
        timestamps = []
        for idx in range(4):
            src_range = np.sqrt(sum(np.square(pulse.position() - self.hydrophone_locations[idx])))
            timestamps += [pulse.t + src_range / self.c]
        return np.array(timestamps)

class Pulse(object):
    def __init__(self, x, y, z, t):
        self.x = x
        self.y = y
        self.z = z
        self.t = t

    def position(self):
        return np.array([self.x, self.y, self.z])

    def __repr__(self):
        return "Pulse:\t" + "x: " + str(self.x) + " y: " + str(self.y) + " z: " + str(self.z)


if __name__ == '__main__':
    def print_red(obj):
        print '\x1b[31m' +  obj.__repr__() + '\x1b[0m'

    hydrophone_locations = rospy.get_param('~/sonar_test/hydrophones')
    sonar = SonarSim(hydrophone_locations, 1484)
    locator = EchoLocator(hydrophone_locations, 1484)
    # pulses will be generated with x, y, z in range [-pulse_range, pulse_range + 1]
    pulse_range = 10 
    rand_args = [-pulse_range, pulse_range + 1]
    for i in range(10):
        pulse = Pulse(random.randrange(*rand_args),
                      random.randrange(*rand_args),
                      random.randrange(*rand_args), 0)
        tstamps = sonar.listen(pulse)
        tstamps = tstamps - tstamps[0]
        print_red(pulse)
        print "Simulated timestamps:\n\t", tstamps
        response = locator.getPulseLocation(np.array(tstamps))
        print "Reconstructed Pulse:\n\t" + "x: " + str(response.x) + " y: " + str(response.y) + " z: " + str(response.z)