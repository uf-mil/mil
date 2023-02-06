#!/usr/bin/env python3

import numpy as np
import rospy
import tf.transformations as trns
from mil_tools import rosmsg_to_numpy
from nav_msgs.msg import Odometry
from navigator_msgs.srv import (
    MoveToWaypoint,
    MoveToWaypointRequest,
    MoveToWaypointResponse,
)
from robot_localization.srv import FromLL
from vrx_gazebo.msg import Task


# Defines a service that returns the position of the acoustic beacon from odom and the range_bearing topic
class LongTrajectory:
    def __init__(self):
        rospy.init_node("long_trajectory_setter")
        self.odom = rospy.Subscriber("/odom", Odometry, self.odometrySubscriber)
        self.pub = rospy.Publisher("/trajectory_long/cmd", Odometry, queue_size=10)
        self.serv = rospy.Service("/set_long_waypoint", MoveToWaypoint, self.handler)
        self.from_lla = rospy.ServiceProxy("/fromLL", FromLL)
        self.task_info_sub = rospy.Subscriber(
            "/vrx/task/info", Task, self.taskinfoSubscriber
        )
        self.boat_pos = np.array([])
        self.boat_ori = np.array([])
        # X error threshold for long trajectory to be satisfied
        self.x_thresh = rospy.get_param("x_thresh", 0.5)
        # Y error threshold for long trajectory to be satisfied
        self.y_thresh = rospy.get_param("y_thresh", 0.5)
        # Yaw error threshold for long trajectory to be satisfied
        self.yaw_thresh = rospy.get_param("yaw_thresh", 0.02)
        self.thresholds_changed = False
        rospy.spin()

    def taskinfoSubscriber(self, msg: Task) -> None:
        if not self.thresholds_changed and msg.name == "wayfinding":
            self.x_thresh = 0.15
            self.y_thresh = 0.15
            self.yaw_thresh = 0.005
            self.thresholds_changed = True

    # Requires both odom and range_bearing to be publishing data
    def odometrySubscriber(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.boat_pos = rosmsg_to_numpy(pos)
        self.boat_ori = rosmsg_to_numpy(ori)

    def handler(self, req: MoveToWaypointRequest) -> MoveToWaypointResponse:
        # define messages
        print("Message received")
        res = MoveToWaypointResponse()
        traj = Odometry()
        rate = rospy.Rate(10)

        while self.boat_pos.size == 0:
            rate.sleep()

        pos = req.target_p.position
        ori = req.target_p.orientation

        # create boat trajectory
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

        # go to waypoint
        # TODO: Account for orientation to determine when we are finished

        x_err = 100
        y_err = 100
        yaw_err = 100
        start_force_end = False
        start_force_end_time = None
        self.orientation = np.array([ori.x, ori.y, ori.z, ori.w])

        while (
            (x_err > self.x_thresh)
            or (y_err > self.y_thresh)
            or (yaw_err > self.yaw_thresh)
        ):
            yaw_err = trns.euler_from_quaternion(
                trns.quaternion_multiply(
                    self.boat_ori, trns.quaternion_inverse(self.orientation)
                )
            )[2]
            x_err = abs(self.boat_pos[0] - pos.x)
            y_err = abs(self.boat_pos[1] - pos.y)

            if (x_err < 1) and (y_err < 1) and (yaw_err < 0.02) and not start_force_end:
                start_force_end = True
                start_force_end_time = rospy.get_rostime()

            if start_force_end and (
                (rospy.get_rostime() - start_force_end_time).to_sec() > 10
            ):
                # Exit out of while loop after 10 seconds of being in same point
                break

            rate.sleep()

        print("Arrived")
        self.boat_pos = np.array([])
        res.success = True
        return res


if __name__ == "__main__":
    LongTrajectory()
