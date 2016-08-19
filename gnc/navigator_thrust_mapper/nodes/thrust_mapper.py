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

^ in that order (top to bottom), it's also alphabetical if you forget

Bibliography:
    [1] Christiaan De Wit
        "Optimal Thrust Allocation Methods for Dynamic Positioning of Ships"
        see: http://repository.tudelft.nl/assets/uuid:4c9685ac-3f76-41c0-bae5-a2a96f4d757e/DP_Report_FINAL.pdf

'''
import rospy
import roslib
import numpy as np
import numpy.linalg
import math
import tf
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray, Bool
from roboteq_msgs.msg import *
from sub8_alarm import AlarmListener


class Thruster(object):

    def __init__(self, cog, angle):
        self.cog = np.array([cog[0], cog[1]])
        self.angle = angle

class Mapper(object):

    def __init__(self, thrusters, effort_ratio, effort_limit):

        self.boat_cog = np.zeros(2)
        self.des_force = np.zeros(3)

        self.positions = []
        self.angles = []

        for thruster in thrusters:
            self.positions.append(np.array(([thruster.cog[0], thruster.cog[1]])))
            self.angles.append(thruster.angle)

        # conversion from newtons to firmware units, measured during thruster testing
        self.effort_ratio = effort_ratio
        # limit for the value sent to motor driver firmware
        self.effort_limit = effort_limit

        self.kill = False
        self.docking_alarm = False

        self.kill_listener = AlarmListener('kill', self.kill_cb)
        self.docking_alarm_listener = AlarmListener('docking', self.docking_alarm_cb)

        # ROS data
        self.BL_pub = rospy.Publisher("/BL_motor/cmd" , Command, queue_size=1)
        self.BR_pub = rospy.Publisher("/BR_motor/cmd" , Command, queue_size=1)
        self.FL_pub = rospy.Publisher("/FL_motor/cmd" , Command, queue_size=1)
        self.FR_pub = rospy.Publisher("/FR_motor/cmd" , Command, queue_size=1)
        rospy.Subscriber("/wrench/cmd", WrenchStamped, self.wrench_cb)

    def kill_cb(self, alarm):
        self.kill = not alarm.clear

    def docking_alarm_cb(self, alarm):
        self.docking_alarm = not alarm.clear

    def wrench_cb(self, msg):
        ''' Grab new wrench
            This is the 'b' in Ax = b
        '''
        force = msg.wrench.force
        torque = msg.wrench.torque
        # Set desired force and torque as numpy array
        self.des_force = np.array((force.x, force.y, torque.z))

    def thrust_matrix(self):
        ''' Iterate through thruster positions and create thruster trans matrix'''
        thruster_matrix = []
        # loop through all sub positions and compute collumns of A
        for thruster_number, position in enumerate(self.positions):
            # l_x and l_y are the offset for the center of gravity
            l_x, l_y = np.subtract(position, self.boat_cog)
            # sin and cos of the angle of the thrusters
            cos = np.cos(self.angles[thruster_number])
            sin = np.sin(self.angles[thruster_number])
            torque_effect = np.cross((l_x, l_y), (cos, sin))
            # must transpose to stack correctly
            thruster_column = np.transpose(np.array([[cos, sin, torque_effect]]))
            thruster_matrix.append(thruster_column)

        # returns a matrix made of the thruster collumns
        return np.hstack(thruster_matrix)

    def allocate(self):
        ''' Solve for thrust vectors after creating trans matrix '''

        # create thrust matrix
        A = self.thrust_matrix()
        b = self.des_force

        # solve Ax = b
        # solutions are given respectively by thruster pose
        bl, br, fl, fr = np.linalg.lstsq(A, b)[0]

        # Temporarily sending the left thruster command to the motor driver
        BL_msg, BR_msg, FL_msg, FR_msg = Command(), Command(), Command(), Command()

        # convert thrusts (in newtons) to effort values (in firmware units)
        # clip them to firmware limits
        # assign to ROS messages
        BL_msg.setpoint = np.clip(bl * self.effort_ratio, -self.effort_limit, self.effort_limit)
        BR_msg.setpoint = np.clip(br * self.effort_ratio, -self.effort_limit, self.effort_limit)
        FL_msg.setpoint = np.clip(fl * self.effort_ratio, -self.effort_limit, self.effort_limit)
        FR_msg.setpoint = np.clip(fr * self.effort_ratio, -self.effort_limit, self.effort_limit)

        # publish ROS messages
        if self.kill is True:
            self.BL_pub.publish(Command(setpoint=0))
            self.BR_pub.publish(Command(setpoint=0))
            self.FL_pub.publish(Command(setpoint=0))
            self.FR_pub.publish(Command(setpoint=0))
        elif self.docking_alarm is True:
            self.BL_pub.publish(BL_msg)
            self.BR_pub.publish(BR_msg)
            self.FL_pub.publish(Command(setpoint=0))
            self.FR_pub.publish(Command(setpoint=0))
        else:
            self.BL_pub.publish(BL_msg)
            self.BR_pub.publish(BR_msg)
            self.FL_pub.publish(FL_msg)
            self.FR_pub.publish(FR_msg)


if __name__ == "__main__":

    rospy.init_node('thruster_mapper')

    # Grab params from launch file
    thruster_BL_cog = rospy.get_param('~thruster_BL_cog')
    thruster_BR_cog = rospy.get_param('~thruster_BR_cog')
    thruster_FL_cog = rospy.get_param('~thruster_FL_cog')
    thruster_FR_cog = rospy.get_param('~thruster_FR_cog')
    thruster_BL_theta = rospy.get_param('~thruster_BL_theta')
    thruster_BR_theta = rospy.get_param('~thruster_BR_theta')
    thruster_FL_theta = rospy.get_param('~thruster_FL_theta')
    thruster_FR_theta = rospy.get_param('~thruster_FR_theta')
    effort_ratio = rospy.get_param('~effort_ratio')
    effort_limit = rospy.get_param('~effort_limit')

    rate = rospy.Rate(50)

    # Create four thruster objects
    BL = Thruster(thruster_BL_cog, thruster_BL_theta)
    BR = Thruster(thruster_BR_cog, thruster_BR_theta)
    FL = Thruster(thruster_FL_cog, thruster_FL_theta)
    FR = Thruster(thruster_FR_cog, thruster_FR_theta)

    # Put the thrusters in a list and give them to the mapper
    thrusters = [BL, BR, FL, FR]
    mapper = Mapper(thrusters, effort_ratio, effort_limit)

    # Allocate for the given wrench and thruster locations
    while not rospy.is_shutdown():
        # map thruster at 50hz
        mapper.allocate()
        rate.sleep()
