#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import GetModelState, ApplyBodyWrenchRequest, ApplyBodyWrench, ApplyBodyWrenchResponse
from sub8_gazebo.srv import SetTurbulence
from sub8_ros_tools import msg_helpers

import numpy as np


class Turbulizor():
    def __init__(self, mag, freq):
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        self.set_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.turbulence_mag = mag
        self.turbulence_freq = freq

        self.reset_srv = rospy.Service('gazebo/set_turbulence', SetTurbulence, self.set_turbulence)

        # Wait for all the models and such to spawn.
        rospy.sleep(3)

        rospy.loginfo("Starting Turbulence.")

        self.turbuloop()

    def set_turbulence(self, srv):
        self.turbulence_mag = srv.magnitude
        self.turbulence_freq = srv.frequency
        return ApplyBodyWrenchResponse()

    def turbuloop(self):
        '''
        The idea is to create a smooth application of force to emulate underwater motion.
        The Turbuloop applies a wrench with a magnitude that varies like a squared function with zeros on both sides
            so that there are no sudden changes in the force.
        '''

        model_name = 'sub8::base_link'

        # Used to gently apply a force on the sub, time_step times per 1 / freq
        time_step = 5.0

        while not rospy.is_shutdown():
            turbulence_mag_step = self.turbulence_mag / time_step
            sleep_step = 1 / (self.turbulence_freq * time_step)

            # Random unit vector scaled by the desired magnitude
            f = np.random.uniform(-1, 1, size=3) * self.turbulence_mag
            r = np.random.uniform(-1, 1, size=3)
            r[:2] = 0  # C3 doesn't like variation in x or y rotation :(

            for i in range(int(time_step)):
                # Square function: -(x - a/2)^2 + (a/2)^2
                mag_multiplier = -((i - time_step / 2) ** 2 - (time_step / 2) ** 2 - 1) * turbulence_mag_step

                # Create service call
                body_wrench = ApplyBodyWrenchRequest()
                body_wrench.body_name = model_name
                body_wrench.reference_frame = model_name
                body_wrench.wrench = msg_helpers.make_wrench_stamped(f * mag_multiplier, r * mag_multiplier).wrench
                body_wrench.start_time = rospy.Time()
                body_wrench.duration = rospy.Duration(sleep_step)

                #rospy.loginfo("{}: Wrench Applied: {}".format(i, body_wrench.wrench))

                self.set_wrench(body_wrench)

                rospy.sleep(sleep_step)

if __name__ == '__main__':
    rospy.init_node('turbulator')
    t = Turbulizor(5, .5)
    rospy.spin()
