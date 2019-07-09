#!/usr/bin/env python
from __future__ import division

import rospy
import mil_tools
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from gazebo_msgs.msg import LinkStates
import numpy as np

from rawgps_common import gps


class GazeboInterface(object):
    def __init__(self, target='wamv::base_link'):
        self.target = target
        intial_lla = [29.534912, -82.303642, 0]  # In Wauhburg
        self.last_ecef = gps.ecef_from_latlongheight(*np.radians(intial_lla))
        self.last_enu = None
        self.last_odom = None
        self.position_offset = None

        self.state_sub_max_prd = rospy.Duration(1 / 100)  # s
        self.last_state_sub_time = rospy.Time.now()
        self.state_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.state_cb)
        self.state_pub = rospy.Publisher('model_odom', Odometry, queue_size=1)
        self.absstate_pub = rospy.Publisher('absodom', Odometry, queue_size=1)

        self.last_odom = None
        self.last_absodom = None

        rospy.Timer(rospy.Duration(1 / 100), self.publish_odom)

    def publish_odom(self, *args):
        if self.last_odom is not None:
            self.state_pub.publish(self.last_odom)
        if self.last_absodom is not None:
            self.absstate_pub.publish(self.last_absodom)

    def enu_to_ecef(self, enu):
        if self.last_enu is None:
            return self.last_ecef

        enu_vector = enu - self.last_enu[0]
        ecef_vector = gps.enu_from_ecef_tf(self.last_ecef).T.dot(enu_vector)
        ecef = ecef_vector + self.last_ecef

        return ecef

    def state_cb(self, msg):
        if rospy.Time.now() - self.last_state_sub_time < self.state_sub_max_prd:
            return
        self.last_state_sub_time = rospy.Time.now()

        if self.target in msg.name:
            target_index = msg.name.index(self.target)

            twist = msg.twist[target_index]
            enu_pose = mil_tools.pose_to_numpy(msg.pose[target_index])

            self.last_ecef = self.enu_to_ecef(enu_pose[0])
            self.last_enu = enu_pose

            self.last_odom = Odometry(
                header=mil_tools.make_header(frame='/enu'),
                child_frame_id='/measurement',
                pose=PoseWithCovariance(
                    pose=mil_tools.numpy_quat_pair_to_pose(*self.last_enu)
                ),
                twist=TwistWithCovariance(
                    twist=twist
                )
            )

            self.last_absodom = Odometry(
                header=mil_tools.make_header(frame='/ecef'),
                child_frame_id='/measurement',
                pose=PoseWithCovariance(
                    pose=mil_tools.numpy_quat_pair_to_pose(self.last_ecef, self.last_enu[1])
                ),
                twist=TwistWithCovariance(
                    twist=twist
                )
            )


if __name__ == '__main__':
    rospy.init_node('gazebo_interface')
    GI = GazeboInterface()
    rospy.spin()
