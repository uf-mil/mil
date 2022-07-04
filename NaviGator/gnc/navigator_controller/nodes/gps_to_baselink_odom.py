#!/usr/bin/env python
# this is a cameron problem https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/
import math

import rospy
import tf
from nav_msgs.msg import Odometry


# Defines a service that returns the position of the acoustic beacon from odom and the range_bearing topic
class GPSToBaseLink:
    def __init__(self):
        rospy.init_node("gps_base_link_odom")
        self.odom = rospy.Subscriber("/odom_gps", Odometry, self.odometrySubscriber)
        self.pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        rospy.spin()

    # Requires both odom and range_bearing to be publishing data
    def odometrySubscriber(self, msg):

        base_link_odom = msg
        pose = [[0, 0, 0], [0, 0, 0, 0]]

        pose[0][0] = msg.pose.pose.position.x
        pose[0][1] = msg.pose.pose.position.y
        pose[0][2] = msg.pose.pose.position.z
        pose[1][0] = msg.pose.pose.orientation.x
        pose[1][1] = msg.pose.pose.orientation.y
        pose[1][2] = msg.pose.pose.orientation.z
        pose[1][3] = msg.pose.pose.orientation.w

        new_odom = self.get_base_link_odom(pose)

        base_link_odom.pose.pose.position.x = new_odom[0][0]
        base_link_odom.pose.pose.position.y = new_odom[0][1]
        base_link_odom.pose.pose.position.z = new_odom[0][2]
        base_link_odom.pose.pose.orientation.x = new_odom[1][0]
        base_link_odom.pose.pose.orientation.y = new_odom[1][1]
        base_link_odom.pose.pose.orientation.z = new_odom[1][2]
        base_link_odom.pose.pose.orientation.w = new_odom[1][3]

        self.pub.publish(base_link_odom)

    def get_base_link_odom(self, gps_pose):
        """Provides odom with reference to base link instead of gps"""
        gps_offset = 0.85
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(gps_pose[1])
        dx = gps_offset * math.cos(yaw)
        dy = gps_offset * math.sin(yaw)

        base_link_pose = gps_pose
        base_link_pose[0][0] = gps_pose[0][0] + dx
        base_link_pose[0][1] = gps_pose[0][1] + dy
        return base_link_pose


if __name__ == "__main__":
    GPSToBaseLink()
