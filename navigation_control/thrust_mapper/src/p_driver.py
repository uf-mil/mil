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

Please include inputs, outputs, and fill with a pseudo-code or description of the source to follow

inputs: /topic
output: /topic

1. Step 1
2. Step 2
3. Step 3
N. Step N

'''

import rospy
import roslib
import numpy as np 
import numpy.linalg
import scipy.linalg
import math,tf,threading
from scipy import optimize
from kill_handling.listener import KillListener
from kill_handling.broadcaster import KillBroadcaster
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray

rospy.init_node('primitive_driver')

thruster_one_cog = rospy.get_param('~thruster_one_cog')
thruster_two_cog = rospy.get_param('~thruster_two_cog')

B_L_LAT_COG = thruster_one_cog
B_R_LAT_COG = thruster_two_cog
BOAT_COG = (0.0,0.0)
B_L_THEDA = 0
B_R_THEDA = 0

class THRUSTER(object):

    def __init__(self, cog, theda_constraint):
        self.thruster_cog = np.array(([cog[0], cog[1]]))
        self.angle_constrain = theda_constraint

class P_Driver(object):

    def __init__(self, positions):
        self.boat_cog = np.array(([0.0,0.0]))
        self.des_force = np.array(([0,0,0])).astype(np.float32)
        rospy.Subscriber("/wrench", WrenchStamped, self.wrench_cb)
        self.pub = rospy.Publisher("/motors" , Float32MultiArray, queue_size = 1)
        self.positions = positions

    def wrench_cb(self, msg):
        force = msg.wrench.force
        torque = msg.wrench.torque
        self.des_force = np.array((force.x, force.y, torque.z))

    def thrust_matrix(self, angle):
        thruster_matrix = []
        for thruster, position in enumerate(self.positions):
            l_x, l_y = np.subtract(position, self.boat_cog)
            cos = np.cos(angle)
            sin = np.sin(angle)
            thruster_column = np.transpose(
                np.array([[ cos, -sin, np.cross((l_x, l_y), (cos, -sin))
                ]]))
            thruster_matrix.append(thruster_column)

        return np.hstack(thruster_matrix)

    def allocate(self, angle):
        A = self.thrust_matrix(angle)
        b = self.des_force
        one, two = np.linalg.lstsq(A, b)[0]
        msg = Float32MultiArray()
        msg.data = [one,two]
        self.pub.publish(msg)
        
if __name__ == "__main__":
    
    BL_lateral = THRUSTER(B_L_LAT_COG, B_L_THEDA)
    BR_lateral = THRUSTER(B_R_LAT_COG, B_R_THEDA)
    mapper = P_Driver([BR_lateral.thruster_cog, BL_lateral.thruster_cog])
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        mapper.allocate(BR_lateral.angle_constrain)
        rate.sleep()
