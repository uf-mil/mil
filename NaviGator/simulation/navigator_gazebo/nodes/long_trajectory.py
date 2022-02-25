#!/usr/bin/env python
import rospy
from twisted.internet import defer
from nav_msgs.msg import Odometry
from navigator_msgs.srv import MoveToWaypoint, MoveToWaypointResponse
from mil_tools import rosmsg_to_numpy
from robot_localization.srv import FromLL
import numpy as np
import math

#Defines a service that returns the position of the acoustic beacon from odom and the range_bearing topic
class LongTrajectory():
    def __init__(self):
        rospy.init_node("long_trajectory_setter")
        self.odom = rospy.Subscriber('/odom', Odometry, self.odometrySubscriber)
        self.pub = rospy.Publisher('/trajectory_long/cmd', Odometry, queue_size=10)
        self.serv = rospy.Service('/set_long_waypoint', MoveToWaypoint, self.handler)
        self.from_lla = rospy.ServiceProxy("/fromLL", FromLL)
        self.boat_pos = np.array([])
        self.boat_ori = np.array([])
        rospy.spin()

    #Requires both odom and range_bearing to be publishing data
    def odometrySubscriber(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.boat_pos = rosmsg_to_numpy(pos)
        self.boat_ori = rosmsg_to_numpy(ori)

    def handler(self, req):
        
        #define messages
        print("Message received")
        res = MoveToWaypointResponse()
        traj = Odometry()
        rate = rospy.Rate(10)

        while (self.boat_pos.size == 0):
            rate.sleep()

        pos = req.target_p.position
        ori = req.target_p.orientation

        #create boat trajectory
        traj.header.frame_id = "enu"
        traj.child_frame_id = "wamv/base_link"
        traj.pose.pose.position.x = pos.x
        traj.pose.pose.position.y = pos.y
        traj.pose.pose.orientation.x = ori.x
        traj.pose.pose.orientation.y = ori.y
        traj.pose.pose.orientation.z = ori.z
        traj.pose.pose.orientation.w = ori.w
        print("sending trajectory")
        self.pub.publish(traj)

        #go to waypoint
        while ( (abs(self.boat_pos[0] - pos.x) > 0.5) ) or ( (abs(self.boat_pos[1] - pos.y) > 0.5) ):
            rate.sleep()


        print("Arrived")
        self.boat_pos = np.array([])
        res.success = True
        return res

if __name__ == '__main__':
    LongTrajectory()
