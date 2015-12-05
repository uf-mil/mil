#!/usr/bin/env python
'''

This source is written for use in the Machine Intelligence Lab in the MAE
Department at the University of Florida. 
It is writen for use on the UF RobotX robot
It is released under the BSD license and is intended for university use
This code is provided "as is" and relies on specific hardware, use at your own risk

Title: Control Arbiter
Start Date: 10-22-15

Author: Zach Goins
Author email: zach.a.goins@gmail.com

Co-author:
Co-author email:

CODE DETAILS --------------------------------------------------------------------

inputs: /wrench/rc, /wrench/autonomous
serivces: change_wrench
output: /wrench

This code will be used as an arbiter for any control sytems that need changing 
during operation of the boat. Services are used because it may be beneficial to
have completion feedback one day when we create the controller. 

Currently there is only a service to change the system we take in wrenchs from to 
send to the primitive driver 

Add an arbiter:
    1. Create a service in the class
    2. Create a callback for that service and set a variable depending on what that service polls
    3. create a publish function to call from the main funtion at the bottom
       that publishes depending on that service variable

Don't forget about edge cases in case a service returns false and 
the control variable you tried to change does not actually change

'''

import rospy
import roslib
import numpy,math,tf,threading
from geometry_msgs.msg import WrenchStamped
from navigator_msg_multiplexer.srv import wrench_arbiter

rospy.init_node('wrench_arbiter')

class control_arb(object):
    # Base class for whatever you are writing
    def __init__(self):

        
        self.rc_wrench = WrenchStamped()
        self.autonomous_wrench = WrenchStamped()
        self.control = "kill"

        self.wrench_pub = rospy.Publisher("/wrench/cmd", WrenchStamped, queue_size = 1)
        rospy.Service('change_wrench', wrench_arbiter, self.change_wrench)
        rospy.Subscriber("/wrench/rc", WrenchStamped, self.rc_cb)
        rospy.Subscriber("/wrench/autonomous", WrenchStamped, self.autonomous_cb)
       
    def rc_cb(self, msg):
        self.rc_wrench = msg

    def autonomous_cb(self, msg):
        self.autonomous_wrench = msg

    def change_wrench(self,req):
        rospy.loginfo("Server recieved request for wrench control change - " + req.str)
        if req.str == "rc":
            self.control = "rc"
            return True
        if req.str == "autonomous":
            self.control = "autonomous"
            return True
        if req.str == "kill":
            self.control = "kill"
            return True
        else:
            self.control = "null"
            return False
        
    def publish_wrench(self):
        if self.control == "rc": 
            self.wrench_pub.publish(self.rc_wrench)
        if self.control == "autonomous": 
            self.wrench_pub.publish(self.autonomous_wrench)
        if self.control == "null" or self.control == "kill":
            blank_wrench = WrenchStamped()
            self.wrench_pub.publish(blank_wrench)
  
if __name__ == "__main__":
    
    arb = control_arb()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        arb.publish_wrench()
        rate.sleep()
    rospy.spin()
