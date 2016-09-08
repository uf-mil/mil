#!/usr/bin/env python

import rospy
import roslib
import numpy,math,tf,threading
from tf import transformations as trns
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from navigator_tools import geometry_helpers as gh

rospy.init_node('move_helper')

class move_helper(object):

    def __init__(self):

        self.odom = Odometry()
        self.pose_pub = rospy.Publisher("/waypoint", PoseStamped, queue_size = 0)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/move_helper", Point, self.move_cb)
        rospy.spin()

    def odom_cb(self, msg):
        self.odom = msg

    def move_cb(self, msg):

        to_send = PoseStamped()
        to_send.header.frame_id = "/enu"

        R = trns.quaternion_matrix([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])[:2, :2]
        theta = gh.quat_to_euler(self.odom.pose.pose.orientation)

        current = numpy.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, numpy.rad2deg(theta[2])])
        shift = numpy.concatenate((R.dot([msg.x, msg.y]), [msg.z]))
        desired = current + shift
        desired_quaternion = trns.quaternion_from_euler(0, 0, numpy.deg2rad(desired[2]))

        to_send.pose.position.x = desired[0]
        to_send.pose.position.y = desired[1]
        to_send.pose.orientation.x, to_send.pose.orientation.y, to_send.pose.orientation.z, to_send.pose.orientation.w  = desired_quaternion

        self.pose_pub.publish(to_send)



if __name__ == "__main__":

    helper = move_helper()
