#!/usr/bin/env python

import rospy
import roslib
import numpy,math,tf,threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from navigator_tools import geometry_helper as gh

rospy.init_node('move_helper')

class move_helper(object):
    # Base class for whatever you are writing
    def __init__(self):

        self.odom = Odometry()
        self.pose_pub = rospy.Publisher("/set_desired_pose", Point, queue_size = 1)

        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/move_helper", Point, self.move_cb)
        rospy.spin()

    def odom_cb(self, msg):
        self.odom = msg

    def move_cb(self, msg):

        to_send = Point()
        theta = gh.quat_to_euler(self.odom.pose.pose.orientation)
        to_send.x = self.odom.pose.pose.position.x + msg.x
        to_send.y = self.odom.pose.pose.position.y + msg.y
        to_send.z = theta[2] + msg.z

        self.pose_pub.publish(to_send)



if __name__ == "__main__":
    #
    helper = move_helper()
