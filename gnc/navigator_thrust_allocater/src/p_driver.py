#!/usr/bin/env python
'''

This source is written for use in the Machine Intelligence Lab in the MAE
Department at the University of Florida.
It is writen for use on the UF RobotX robot
It is released under the BSD license and is intended for university use
This code is provided "as is" and relies on specific hardware, use at your own risk

Title: Thruster Mapper
Start Date: 10-15-15

Author: Zach Goins
Author email: zach.a.goins@gmail.com

Co-author:
Co-author email:

CODE DETAILS --------------------------------------------------------------------

inputs: /wrench
output: /motors

This program recieves a wrench from the controller or xbox_node and solves for
the optimal thrust vector configurations. Thruster center of gravity offset are
located in the the launch file.

Thruters are added to the solver by creating new THRUSTER obects and adding
them to the P_DRIVER class. A transition matrix is created based on the number of thrusters
and then a linear least squares solution is used to minimize Ax=b

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

class P_Driver(object):

    def __init__(self, positions, thrust_limit):

        self.boat_cog = np.array(([0.0,0.0]))
        self.des_force = np.array(([0,0,0])).astype(np.float32)
        self.kill = True

        self.thrust_limit = thrust_limit

        # ROS data
        self.BL_pub = rospy.Publisher("/BL_motor/cmd" , Command, queue_size = 1)
        self.BR_pub = rospy.Publisher("/BR_motor/cmd" , Command, queue_size = 1)
        self.FL_pub = rospy.Publisher("/FL_motor/cmd" , Command, queue_size = 1)
        self.FR_pub = rospy.Publisher("/FR_motor/cmd" , Command, queue_size = 1)

        rospy.Subscriber("/wrench/cmd", WrenchStamped, self.wrench_cb)
        rospy.Subscriber("/killed", Bool, self.kill_cb)

        # list of  thruster positions center of gravity offsets
        self.positions = positions

    def wrench_cb(self, msg):
        """ grab new wrench """
        force = msg.wrench.force
        torque = msg.wrench.torque
        self.des_force = np.array((force.x, force.y, torque.z))

    def kill_cb(self, msg):
        """ grab new wrench """
        self.kill = msg.data

    def thrust_matrix(self, angles):
        """ Iterate through thruster positions and create thruster trans matrix"""

        thruster_matrix = []
        ghetto_count = 0
        # loop through all sub positions and compute collumns of A
        for thruster, position in enumerate(self.positions):
            # l_x and l_y are the offset for the center of gravity
            l_x, l_y = np.subtract(position, self.boat_cog)
            # sin and cos of the angle of the thrusters
            cos = np.cos(angles[ghetto_count])
            sin = np.sin(angles[ghetto_count])
            torque_effect = np.cross((l_x, l_y), (cos, -sin))
            # must transpose to stack correctly
            thruster_column = np.transpose(np.array([[ cos, -sin, torque_effect]]))
            thruster_matrix.append(thruster_column)
            ghetto_count += 1

        # returns a matrix made of the thruster collumns
        return np.hstack(thruster_matrix)

    def allocate(self, angles):
        """ Solve for thrust vectors after creating trans matrix """

        def clip_thrust(thrust):
            if thrust > self.thrust_limit:
                new_thrust = self.thrust_limit
            return new_thrust

        # create thrust matrix
        A = self.thrust_matrix(angles)
        b = self.des_force

        # solve Ax = b
        # solutions are given respectively by one, two, three, n...
        one, two, three, four = np.linalg.lstsq(A, b)[0]

        # Temporarily sending the left thruster command to the motor driver
        BL_msg, BR_msg, FL_msg, FR_msg = Command(), Command(), Command(), Command()

        BL_msg.setpoint = clip_thrust(two)
        BR_msg.setpoint = clip_thrust(one)
        FL_msg.setpoint = clip_thrust(three)
        FR_msg.setpoint = clip_thrust(four)

        #print ""
        #print ""
        #print "FL: " + str(three) + "   FR: ", str(two)
        #print ""
        #print "BL: " + str(one) + "   BR: ", str(four)
        #print ""
        #print ""

        self.BL_pub.publish(BL_msg)
        self.BR_pub.publish(BR_msg)
        self.FL_pub.publish(FL_msg)
        self.FR_pub.publish(FR_msg)

if __name__ == "__main__":

    rospy.init_node('primitive_driver')

    thruster_BR_cog = rospy.get_param('~thruster_BR_cog')
    thruster_BL_cog = rospy.get_param('~thruster_BL_cog')
    thruster_FR_cog = rospy.get_param('~thruster_FR_cog')
    thruster_FL_cog = rospy.get_param('~thruster_FL_cog')
    thruster_BR_theta = rospy.get_param('~thruster_BR_theta')
    thruster_BL_theta = rospy.get_param('~thruster_BL_theta')
    thruster_FR_theta = rospy.get_param('~thruster_FR_theta')
    thruster_FL_theta = rospy.get_param('~thruster_FL_theta')

    rate = rospy.Rate(50)
    # Create two thrusters
    BL = Thruster(thruster_BR_cog, thruster_BR_theta)
    BR = Thruster(thruster_BL_cog, thruster_BL_theta)
    FL = Thruster(thruster_FR_cog, thruster_FR_theta)
    FR = Thruster(thruster_FL_cog, thruster_FL_theta)

    thrust_limit = 600

    # Pass current thruster data to mapper
    mapper = P_Driver([BR.thruster_cog, BL.thruster_cog, FL.thruster_cog, FR.thruster_cog], thrust_limit)

    while not rospy.is_shutdown():
        # map thruster
        mapper.allocate([thruster_BR_theta, thruster_BL_theta, thruster_FL_theta, thruster_FR_theta])
        rate.sleep()
