#!/usr/bin/env python
import rospy
import rospkg

from gazebo_msgs.msg import ContactsState, ModelStates, ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from geometry_msgs.msg import Pose, Twist
from sub8_actuator_driver.srv import SetValve

from sub8_ros_tools import msg_helpers

import os
import numpy as np

# Replace this with launch file parameter.
rospack = rospkg.RosPack()
torpedo_path = os.path.join(rospack.get_path('sub8_gazebo'), 'models/torpedo/torpedo.sdf')


class TorpedoLauncher():
    def __init__(self):
        f = open(torpedo_path, 'r')
        self.sdf_f = f.read()

        self.launched = False

        rospy.Service('/actuator_driver/actuate', SetValve, self.launch_torpedo)
        self.set_torpedo = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        rospy.Subscriber('/contact_bumper/', ContactsState, self.check_contact, queue_size=1)

    def check_contact(self, msg):
        #print "------------------------------------------------------------------------------------------------"

        if not self.launched:
            return

        if len(msg.states) == 0:
            # If there is no impact don't worry about it.
            self.launched = True
            return

        torpedo_name = "torpedo::body::bodycol"
        board_name = "ground_plane::link::collision"

        for state in msg.states:
            if state.collision1_name == torpedo_name:
                break

        print state

        # Now our torpedo friend has impacted something, we gotta check what.
        rospy.loginfo('Impact detected!')
        self.launched = False

        if state.collision2_name == board_name:
            rospy.loginfo('Correct!')
        else:
            rospy.logwarn('Wrong!')

    def launch_torpedo(self, srv):
        '''
        Find position of sub and launch the torpedo from there.

        TODO:
            - Offset sub position so that it launches from the torpedo location instead of
              arbitrarily infront of the sub.
            - Fix rotation problem.
        '''
        sub_state = self.get_model(model_name='sub8')
        sub_pose = msg_helpers.pose_to_numpy(sub_state.pose)

        launch_twist_np = rotate_vect_by_quat(np.array([5, 0, 0]), sub_pose[1])
        print launch_twist_np

        launch_twist = Twist()
        launch_twist.linear.x = launch_twist_np[0]
        launch_twist.linear.y = -launch_twist_np[1]
        launch_twist.linear.z = launch_twist_np[2]

        model_state = ModelState()
        model_state.model_name = 'torpedo'
        model_state.pose = msg_helpers.numpy_quat_pair_to_pose(sub_pose[1], sub_pose[0] + np.array([.5, 0, 0]))
        model_state.twist = launch_twist
        print model_state
        self.set_torpedo(model_state)

        # We have to wait until the torpedo actually gets fired to start listening for impacts.
        rospy.sleep(.1)
        self.launched = True


def rotate_vect_by_quat(v, q):
    '''
    I'm sure there's a library function to do this.

    v should be [x,y,z]
    q should be [x,y,z,w]

    [1] http://www.mathworks.com/help/aerotbx/ug/quatrotate.html
        NOTE:
        q0 = q[3]
        q1 = q[0]
        q2 = q[1]
        q3 = q[2]
    '''
    # ew
    r1 = v[0] * (1 - 2 * q[1] ** 2 - 2 * q[2] ** 2) + \
        v[1] * 2 * (q[0] * q[1] + q[3] * q[2]) +  \
        v[2] * 2 * (q[0] * q[2] - q[3] * q[1])
    r2 = v[1] * (1 - 2 * q[0] ** 2 - 2 * q[2] ** 2) + \
        v[0] * 2 * (q[0] * q[1] - q[3] * q[2]) +  \
        v[2] * 2 * (q[1] * q[2] + q[3] * q[0])
    r3 = v[2] * (1 - 2 * q[0] ** 2 - 2 * q[1] ** 2) + \
        v[0] * 2 * (q[0] * q[2] + q[3] * q[1]) +  \
        v[1] * 2 * (q[1] * q[2] - q[3] * q[0])

    return np.array([r1, r2, r3])
if __name__ == '__main__':
    rospy.init_node('torpedo_manager')
    t = TorpedoLauncher()
    rospy.spin()