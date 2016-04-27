#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def pose_est_server():
    rospy.init_node('torpedo_board_pose_est')
    pose_est = rospy.Service()

if __name__ == '__main__':
    rospy.init_node('torpedo_board_pose_est')
