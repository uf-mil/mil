#!/usr/bin/env python
import rospy
import sub8_ros_tools as sub8_utils
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion
from gazebo_msgs.msg import LinkStates, ModelState
from uf_common.msg import Float64Stamped
import numpy as np
import os


class GazeboInterface(object):
    def __init__(self, target='wamv::base_link'):
        self.target = target

        self.last_odom = None
        self.position_offset = None
        self.state_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.state_cb)
        self.state_pub = rospy.Publisher('odom', Odometry, queue_size=1)

        rospy.Timer(rospy.Duration(0.03), self.publish_odom)

    def publish_odom(self, *args):
        if self.last_odom is None:
            return

        msg = self.last_odom
        if self.target in msg.name:
            header = sub8_utils.make_header(frame='/enu')

            target_index = msg.name.index(self.target)
            twist = msg.twist[target_index]

            twist = msg.twist[target_index]
            pose = msg.pose[target_index]
            self.state_pub.publish(
                header=header,
                child_frame_id='/base_link',
                pose=PoseWithCovariance(
                    pose=pose
                ),
                twist=TwistWithCovariance(
                    twist=twist
                )
            )

    def state_cb(self, msg):
        if self.target not in msg.name:
            return

        self.last_odom = msg

if __name__ == '__main__':
    rospy.init_node('gazebo_interface')
    GI = GazeboInterface()
    rospy.spin()
