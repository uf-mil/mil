#!/usr/bin/env python
'''
Title: Thrust Mapper
Start Date: 10-15-15

Author: Zach Goins
Author email: zach.a.goins@gmail.com

CODE DETAILS --------------------------------------------------------------------

inputs: /wrench
output: /motors

This program recieves a wrench from the controller or xbox_node and solves for
the optimal thrust vector configurations. Thruster center of gravity offset are
located in the the launch file.

There are two classes in this file: Thruster and Mapper

Thrusters are added to the solver by creating new Thruster obects and adding
them to the Mapper class on instantiation.

This mapper is a simple Ax = b linear least squares solver. The A matrix is built on every
iteration and remapping, allowing us to one day detect thruster failure and adapt the mapper
on the fly. For right now though we are solving for 4 thrusters offset at +-45 degree angles to the vehicle.

Thruster configurations are in the gnc.launch file. They contain the (x,y,theta) offset for each thruster.
The naming conventions below are used throughout the entire system

BL = Back Left
BR = Back Right
FL = Front Left
FR = Front Right

Bibliography:
    [1] Christiaan De Wit
        "Optimal Thrust Allocation Methods for Dynamic Positioning of Ships"
        see: http://repository.tudelft.nl/assets/uuid:4c9685ac-3f76-41c0-bae5-a2a96f4d757e/DP_Report_FINAL.pdf

'''

import rospy
import roslib
import numpy as np
import numpy.linalg
import math,tf,threading
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray, Bool
from roboteq_msgs.msg import *

class Thruster(object):

    def __init__(self, cog, theda_constraint):
        self.thruster_cog = np.array(([cog[0], cog[1]]))
        self.angle_constrain = theda_constraint

class Mapper(object):

    def __init__(self, positions, thrust_limit):


        self.boat_cog = np.array(([0.0,0.0]))
        self.des_force = np.array(([0,0,0])).astype(np.float32)

        # list of thruster positions (x,y,theda) in the body frame
        self.positions = positions
        # Thrust limit used in the clip_thrust() function
        self.thrust_limit = thrust_limit

        # ROS data
        self.BL_pub = rospy.Publisher("/BL_motor/cmd" , Command, queue_size = 1)
        self.BR_pub = rospy.Publisher("/BR_motor/cmd" , Command, queue_size = 1)
        self.FL_pub = rospy.Publisher("/FL_motor/cmd" , Command, queue_size = 1)
        self.FR_pub = rospy.Publisher("/FR_motor/cmd" , Command, queue_size = 1)
        rospy.Subscriber("/wrench/cmd", WrenchStamped, self.wrench_cb)

    def wrench_cb(self, msg):
        ''' Grab new wrench
            This is the 'b' in Ax = b
        '''

        force = msg.wrench.force
        torque = msg.wrench.torque
        # Set desired force and torque as numpy array
        self.des_force = np.array((force.x, force.y, torque.z))

    def thrust_matrix(self, angles):
        ''' Iterate through thruster positions and create thruster trans matrix'''

        thruster_matrix = []
        count = 0
        # loop through all sub positions and compute collumns of A
        for thruster, position in enumerate(self.positions):
            # l_x and l_y are the offset for the center of gravity
            l_x, l_y = np.subtract(position, self.boat_cog)
            # sin and cos of the angle of the thrusters
            cos = np.cos(angles[count])
            sin = np.sin(angles[count])
            torque_effect = np.cross((l_x, l_y), (cos, -sin))
            # must transpose to stack correctly
            thruster_column = np.transpose(np.array([[ cos, -sin, torque_effect]]))
            thruster_matrix.append(thruster_column)
            count += 1

        # returns a matrix made of the thruster collumns
        return np.hstack(thruster_matrix)

    def allocate(self, angles):
        ''' Solve for thrust vectors after creating trans matrix '''

        def clip_thrust(thrust):
            new_thrust = thrust
            if thrust > self.thrust_limit:
                new_thrust = self.thrust_limit
            return new_thrust

        # create thrust matrix
        A = self.thrust_matrix(angles)
        b = self.des_force

        # solve Ax = b
        # solutions are given respectively by thruster pose
        br, bl, fl, fr = np.linalg.lstsq(A, b)[0]

        # Temporarily sending the left thruster command to the motor driver
        BL_msg, BR_msg, FL_msg, FR_msg = Command(), Command(), Command(), Command()

        # Clip thrust and assign ro ROS messages
        BL_msg.setpoint = clip_thrust(bl)
        BR_msg.setpoint = clip_thrust(br)
        FL_msg.setpoint = clip_thrust(fl)
        FR_msg.setpoint = clip_thrust(fr)

        self.BL_pub.publish(BL_msg)
        self.BR_pub.publish(BR_msg)
        self.FL_pub.publish(FL_msg)
        self.FR_pub.publish(FR_msg)

if __name__ == "__main__":

    rospy.init_node('primitive_driver')

    # Grab params from launch file
    thruster_BR_cog = rospy.get_param('~thruster_BR_cog')
    thruster_BL_cog = rospy.get_param('~thruster_BL_cog')
    thruster_FR_cog = rospy.get_param('~thruster_FR_cog')
    thruster_FL_cog = rospy.get_param('~thruster_FL_cog')
    thruster_BR_theta = rospy.get_param('~thruster_BR_theta')
    thruster_BL_theta = rospy.get_param('~thruster_BL_theta')
    thruster_FR_theta = rospy.get_param('~thruster_FR_theta')
    thruster_FL_theta = rospy.get_param('~thruster_FL_theta')

    rate = rospy.Rate(50)

    # Create four thruster objects
    BL = Thruster(thruster_BR_cog, thruster_BR_theta)
    BR = Thruster(thruster_BL_cog, thruster_BL_theta)
    FL = Thruster(thruster_FR_cog, thruster_FR_theta)
    FR = Thruster(thruster_FL_cog, thruster_FL_theta)

    # Set the thrust limit
    thrust_limit = 600

    # Pass current thruster data and thrust limit to mapper
    mapper = Mapper([BR.thruster_cog, BL.thruster_cog, FL.thruster_cog, FR.thruster_cog], thrust_limit)

    while not rospy.is_shutdown():
        # map thruster at 50hz
        mapper.allocate([thruster_BR_theta, thruster_BL_theta, thruster_FL_theta, thruster_FR_theta])
        rate.sleep()
