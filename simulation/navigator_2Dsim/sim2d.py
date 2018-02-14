#!/usr/bin/env python

from __future__ import division
import numpy as np
import rospy
import matplotlib.pyplot as plt
import sys
from sys import argv
import tf.transformations as trns
import ast
import mil_tools

from geometry_msgs.msg import WrenchStamped, PoseStamped, PointStamped, PoseWithCovariance, TwistWithCovariance
from roboteq_msgs.msg import Command
from nav_msgs.msg import Odometry, OccupancyGrid
from rawgps_common import gps

import rosbag
from std_msgs.msg import Int32, String

# Ros params that are now handled in launch.
# Uncomment if you need to run this without the launch file.
# rospy.set_param('inertia', ([1, 1, 2]))
# rospy.set_param('drag', ([.5, 1, 1]))
# rospy.set_param('mass', 544)

# Twist seems off, also no covariance matrices? Pose looks fine tho.


class Navsim():

    def __init__(self, t=0, pose0=np.array([0, 0, 0]), twist0=np.array([0, 0, 0]), wind=None):

        rospy.init_node('2Dsim')
        self.des_force = np.array([0, 0, 0])
        self.gui = rospy.get_param("gui")
        self.world_frame_id = "/enu"
        self.body_frame_id = "/base_link"
        self.state = None
        self.get_ref = None
        self.last_odom = None
        self.last_absodom = None
        initial_lla = ast.literal_eval(rospy.get_param("start_pos"))
        # print(intial_lla)
        # self.last_ecef = gps.ecef_from_latlongheight(*np.radians(initial_lla))
        self.last_enu = None
        self.state_sub_max_prd = rospy.Duration(1 / 100)
        self.last_state_sub_time = rospy.Time.now()

        self.subscriber = rospy.Subscriber(
            "/wrench/cmd", WrenchStamped, self.wrench_cb)
        self.odompublisher = rospy.Publisher(
            "/odom", Odometry, queue_size=1)
        self.poses = []
        self.wrenches = []
        self.times = []
        self.twists = []
        # X, Y, and angle in world frame
        self.pose = np.float64(initial_lla)
        # X, Y, and rate of angle change in boat frame (Surge, Sway, yaw rate)
        self.twist = np.float64(twist0)
        # kg, kg, kg*m^2
        self.inertia = np.array(ast.literal_eval(
            rospy.get_param('inertia')), dtype=np.float64)
        # Boat is longer than shorter, hence .5, 1, 1.
        self.drag = np.array(ast.literal_eval(
            rospy.get_param('drag')), dtype=np.float64)
        # Time in seconds
        self.t = t
        # Wind is a function of time that returns a wrench
        if wind:
            self.wind = wind
        else:
            self.wind = lambda t: np.zeros(3, dtype=np.float64)

        rospy.Timer(rospy.Duration(1 / 100), self.publish_odom)

    def step(self, dt, wrench):
        s = np.sin(self.pose[2])
        c = np.cos(self.pose[2])
        # Rotation Matrix converts body to world by default
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        wrench = np.array(wrench)
        posedot, twistdot = self.state_deriv(np.float64(wrench), R)
        self.pose = self.pose + posedot * dt + 0.5 * R.dot(twistdot) * dt**2
        self.twist = self.twist + twistdot * dt
        self.t = self.t + dt

    def state_deriv(self, wrench, R):
        posedot = R.dot(self.twist)
        twistdot = (1 / self.inertia) * (wrench - self.drag *
                                         self.twist + R.T.dot(self.wind(self.t)))
        return posedot, twistdot

    def wrench_cb(self, msg):
        ''' Grab new wrench
                This is the 'b' in Ax = b
        '''
        self.force = msg.wrench.force
        self.torque = msg.wrench.torque
        # Set desired force and torque as numpy array
        self.des_force = np.array((self.force.x, self.force.y, self.torque.z))

    def publish_odom(self, *args):
        if self.last_odom is not None:
            self.odompublisher.publish(self.last_odom)
        if self.last_absodom is not None:
            self.absodompublisher.publish(self.last_absodom)

    # def enu_to_ecef(self, enu):
    #     if self.last_enu is None:
    #         return self.last_ecef

    #     enu_vector = enu - self.last_enu[0]
    #     ecef_vector = gps.enu_from_ecef_tf(self.last_ecef).T.dot(enu_vector)
    #     ecef = ecef_vector + self.last_ecef

        # return ecef

    def pack_odom(self, pose, twist):
        """
        Converts a state vector into an Odometry message
        with a given header timestamp.

        """
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.world_frame_id
        msg.child_frame_id = self.body_frame_id
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = pose
        # print(msg.pose.pose.position.z)
        quat = trns.quaternion_from_euler(0, 0, pose[2])
        msg.pose.pose.orientation = mil_tools.numpy_to_quaternion(quat)
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = twist
        msg.twist.twist.angular.z = twist[2]
        return msg

    def state_cb(self):
        if rospy.Time.now() - self.last_state_sub_time < self.state_sub_max_prd:
            return

        self.last_state_sub_time = rospy.Time.now()

        # self.last_ecef = self.enu_to_ecef(self.pose)
        self.last_enu = self.pose
        # print(self.last_enu)
        self.last_odom = self.pack_odom(self.pose, self.twist)

        # self.last_absodom = self.pack_odom(self.last_ecef, self.twist)

if __name__ == '__main__':

    # Check to see if additional command line argument was passed.
    # Allows 2D sim to play bags on its own, bag must be in the same
    # directory. Fix this later.

    # coordinate conversion server
    # Prep

    # duration =

    navsim = Navsim(pose0=np.array(
        [0, 0, np.pi / 2]))

    if navsim.gui == True:
        plt.ion()

        fig1 = plt.figure()
        fig1.suptitle('State Evolution', fontsize=20)
        fig1rows = 2
        fig1cols = 4
    # print(navsim.gui)
    dt = .05

    while not rospy.is_shutdown():
        navsim.poses.append(navsim.pose)
        navsim.twists.append(navsim.twist)
        navsim.times.append(navsim.t)
        # Increment sim
        # wrench = controller.compute_wrench(navsim.pose, navsim.twist, goal_pose,
        # goal_twist)
        navsim.state_cb()
        navsim.publish_odom()
        navsim.wrenches.append(navsim.des_force)
        navsim.step(dt, navsim.wrenches[-1])
        poses = np.asarray(navsim.poses)
        twists = np.asarray(navsim.twists)
        wrenches = np.asarray(navsim.wrenches)
        times = np.asarray(navsim.times)

        # Figure for individual results
        if navsim.gui == True:
            # Plot x position
            ax1 = fig1.add_subplot(fig1rows, fig1cols, 1)
            ax1.set_title('East Position (m)', fontsize=16)
            ax1.plot(times, poses[:, 0], 'k')
            ax1.grid(True)

            # Plot y position
            ax1 = fig1.add_subplot(fig1rows, fig1cols, 2)
            ax1.set_title('North Position (m)', fontsize=16)
            ax1.plot(times, poses[:, 1], 'k')
            ax1.grid(True)

            # Plot yaw position
            ax1 = fig1.add_subplot(fig1rows, fig1cols, 3)
            ax1.set_title('Heading (deg)', fontsize=16)
            ax1.plot(times, np.rad2deg(poses[:, 2]), 'k')
            ax1.grid(True)

            # Plot control efforts
            ax1 = fig1.add_subplot(fig1rows, fig1cols, 4)
            ax1.set_title('Wrench (N, N, N*m)', fontsize=16)
            ax1.plot(times, wrenches[:, 0], 'b',
                     times, wrenches[:, 1], 'g',
                     times, wrenches[:, 2], 'r')
            ax1.grid(True)

            # Plot x velocity
            ax1 = fig1.add_subplot(fig1rows, fig1cols, 5)
            ax1.set_title('Surge (m/s)', fontsize=16)
            ax1.plot(times, twists[:, 0], 'k')
            ax1.set_xlabel('Time (s)')
            ax1.grid(True)

            # Plot y velocity
            ax1 = fig1.add_subplot(fig1rows, fig1cols, 6)
            ax1.set_title('Sway (m/s)', fontsize=16)
            ax1.plot(times, twists[:, 1], 'k')
            ax1.set_xlabel('Time (s)')
            ax1.grid(True)

            # Plot yaw velocity
            ax1 = fig1.add_subplot(fig1rows, fig1cols, 7)
            ax1.set_title('Yaw (deg/s)', fontsize=16)
            ax1.plot(times, np.rad2deg(twists[:, 2]), 'k')
            ax1.set_xlabel('Time (s)')
            ax1.grid(True)

            # Plot parametric
            ax1 = fig1.add_subplot(fig1rows, fig1cols, 8)
            ax1.set_title('Position (deg/s)', fontsize=16)
            ax1.scatter(poses[0, 0], poses[0, 1])
            ax1.plot(poses[:, 0], poses[:, 1], 'k')
            ax1.set_xlabel('Eastness (m)')
            ax1.set_ylabel('Northness (m)')
            ax1.grid(True)

            plt.pause(.1)
    # print(wrenches)

    rospy.spin()
