#!/usr/bin/env python
from __future__ import division
import numpy as np
import rospy
import tf.transformations as trns
from mil_tools import numpy_to_quaternion
from nav_msgs.msg import Odometry
from roboteq_msgs.msg import Command
from navigator_thrust_mapper import ThrusterMap


class Navsim():
    '''
    A simple 2D simulation of the kinematics of NaviGator.
    '''
    def __init__(self, pose0=np.array([0, 0, 0]), twist0=np.array([0, 0, 0])):
        # Used to publish current state
        self.odom_publisher = rospy.Publisher("/odom", Odometry, queue_size=1)

        # Set initial state from constructor
        self.pose = np.float64(pose0)
        self.twist = np.float64(twist0)
        self.wrench = np.float64([0, 0, 0])
        self.thrusts = np.zeros(4, dtype=np.float64)

        # Get other contants from ROS params
        self.get_params()

        # Get URDF description from thrusters
        urdf = rospy.get_param('robot_description', default=None)
        if urdf is None or len(urdf) == 0:
            raise Exception('robot description not set or empty')
        self.thrust_map = ThrusterMap.from_urdf(urdf)

        # Subscribe to thrusters so we can simulate their forces
        for i, motor in enumerate(self.thrust_map.names):
            rospy.Subscriber('/{}_motor/cmd'.format(motor), Command, self.thruster_cb, queue_size=3, callback_args=i)

        # Start timer to run simulator
        rospy.Timer(rospy.Duration(self.update_period), self.timer_cb)

    def get_params(self):
        '''
        Load important configurable constants from ROS params
        '''
        mass = rospy.get_param('~mass')
        rotational_inertia = rospy.get_param('~rotational_inertia')
        self.inertia = np.float64([mass, mass, rotational_inertia])
        self.drag = np.float64(rospy.get_param('~drag'))
        self.wind = np.float64(rospy.get_param('~wind', [0, 0, 0]))
        self.update_period = rospy.get_param('~update_period', 0.1)
        self.world_frame = rospy.get_param('~world_frame', 'enu')
        self.body_frame = rospy.get_param('~body_frame', 'base_link')

    def thruster_cb(self, msg, index):
        self.thrusts[index] = msg.setpoint
        self.wrench = self.thrust_map.thrusts_to_wrench(self.thrusts)

    def timer_cb(self, timer_event):
        '''
        Each time timer triggers, update state and publish odometry
        '''
        self.step(self.update_period, self.wrench)
        self.publish_odom()

    def step(self, dt, wrench):
        '''
        Simulate new pose and twist given a time delta and a force/torque applied to NaviGator
        '''
        s = np.sin(self.pose[2])
        c = np.cos(self.pose[2])
        # Rotation Matrix converts body to world by default
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        wrench = np.array(wrench)
        posedot, twistdot = self.state_deriv(np.float64(wrench), R)
        self.pose = self.pose + posedot * dt + 0.5 * R.dot(twistdot) * dt**2
        self.twist = self.twist + twistdot * dt

    def state_deriv(self, wrench, R):
        posedot = R.dot(self.twist)
        twistdot = (1 / self.inertia) * (wrench - self.drag *
                                         np.sign(self.twist) * self.twist * self.twist + R.T.dot(self.wind))
        return posedot, twistdot

    def publish_odom(self):
        '''
        Publish to odometry with latest pose and twist
        '''
        odom = self.pack_odom(self.pose, self.twist)
        self.odom_publisher.publish(odom)

    def pack_odom(self, pose, twist):
        """
        Converts pose and twist into an Odometry message
        """
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.world_frame
        msg.child_frame_id = self.body_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose[0], pose[1]
        quat = trns.quaternion_from_euler(0, 0, pose[2])
        msg.pose.pose.orientation = numpy_to_quaternion(quat)
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = twist[0:2]
        msg.twist.twist.angular.z = twist[2]
        return msg


if __name__ == '__main__':
    rospy.init_node('navigator_sim2D')
    navsim = Navsim(pose0=np.array([0, 0, np.pi / 2]))
    rospy.spin()
