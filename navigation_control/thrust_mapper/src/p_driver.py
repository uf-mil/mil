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
from std_msgs.msg import Float32MultiArray
from roboteq_msgs.msg import *

rospy.init_node('primitive_driver')
thruster_one_cog = rospy.get_param('~thruster_one_cog')
thruster_two_cog = rospy.get_param('~thruster_two_cog')

B_L_LAT_COG = thruster_one_cog
B_R_LAT_COG = thruster_two_cog
B_L_THEDA = 0
B_R_THEDA = 0

class Thruster(object):

    def __init__(self, cog, theda_constraint):
        self.thruster_cog = np.array(([cog[0], cog[1]]))
        self.angle_constrain = theda_constraint

class P_Driver(object):

    def __init__(self, positions):

        self.boat_cog = np.array(([0.0,0.0]))
        self.des_force = np.array(([0,0,0])).astype(np.float32)
        
        # ROS data
        self.left_pub = rospy.Publisher("/left_motor/cmd" , Command, queue_size = 1)
        self.right_pub = rospy.Publisher("/right_motor/cmd" , Command, queue_size = 1)
        rospy.Subscriber("/wrench", WrenchStamped, self.wrench_cb)

        # list of  thruster positions center of gravity offsets
        self.positions = positions

    def wrench_cb(self, msg):
        """ grab new wrench """
        force = msg.wrench.force
        torque = msg.wrench.torque
        self.des_force = np.array((force.x, force.y, torque.z))

    def thrust_matrix(self, angle):
        """ Iterate through thruster positions and create thruster trans matrix"""

        thruster_matrix = []
        # loop through all sub positions and compute collumns of A
        for thruster, position in enumerate(self.positions):
            # l_x and l_y are the offset for the center of gravity
            l_x, l_y = np.subtract(position, self.boat_cog)
            # sin and cos of the angle of the thrusters 
            cos = np.cos(angle)
            sin = np.sin(angle)
            torque_effect = np.cross((l_x, l_y), (cos, -sin))
            # must transpose to stack correctly
            thruster_column = np.transpose(np.array([[ cos, -sin, torque_effect]]))
            thruster_matrix.append(thruster_column)

        # returns a matrix made of the thruster collumns
        return np.hstack(thruster_matrix)

    def allocate(self, angle):
        """ Solve for thrust vectors after creating trans matrix """

        # create thrust matrix
        A = self.thrust_matrix(angle)
        b = self.des_force
        # solve Ax = b
        # solutions are given respectively by one, two, three, n...
        one, two = np.linalg.lstsq(A, b)[0]

        # Temporarily sending the left thruster command to the motor driver
        right_msg = Command()
        left_msg = Command()


        right_msg.setpoint = one
        left_msg.setpoint = two

        self.right_pub.publish(right_msg)
        self.left_pub.publish(left_msg)
        
if __name__ == "__main__":
    
    
    rate = rospy.Rate(10)
    # Create two thrusters
    BL_lateral = Thruster(B_L_LAT_COG, B_L_THEDA)
    BR_lateral = Thruster(B_R_LAT_COG, B_R_THEDA)

    # Pass current thruster data to mapper
    mapper = P_Driver([BR_lateral.thruster_cog, BL_lateral.thruster_cog])

    while not rospy.is_shutdown():
        # map thruster
        mapper.allocate(BR_lateral.angle_constrain)
        rate.sleep()
