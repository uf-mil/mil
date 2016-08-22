#!/usr/bin/env python
'''

Title: Control Arbiter
Start Date: 10-22-15

Author: Zach Goins
Author email: zach.a.goins@gmail.com

CODE DETAILS --------------------------------------------------------------------

inputs: /wrench/rc, /wrench/autonomous /wrench/gui
serivces: change_wrench
output: /wrench/cmd

This code will be used as an arbiter for any systems outputting a wrench during operation of the boat.
No movement commands go to the motors except through the thurster mapper. Because of this all command sources
that move the vessel should output a wrench. Right now the controller, remote control, and the gui all are able to
send commands to the boat meaning they all output a wrench. However, only one wrench can be sent to the mapper.

This program is used to control from which command source the boat is taking commands.

All the functions in this class are ROS callbacks except for one: The one that publishes the correct wrench
That way wrenches are always publishing and the settings change on the fly.

Don't forget about edge cases in case a service returns false and
the control variable you tried to change does not actually change

'''

import rospy
import roslib
import numpy,math,tf,threading
from geometry_msgs.msg import WrenchStamped
from navigator_msgs.srv import WrenchSelect
from std_msgs.msg import Bool

rospy.init_node('wrench_arbiter')

class control_arb(object):

    def __init__(self):

        # Set the three recieving variables
        self.rc_wrench = WrenchStamped()
        self.autonomous_wrench = WrenchStamped()
        self.gui_wrench = WrenchStamped()

        # Variable that is set when the control source is changed
        # It is what is used to determine what should be output
        # SET TO NONE ON DEFAULT MEANING THE BOAT IS STARTED KILLED
        self.control = None

        # ROS stuff - Wrench changing service, final output wrench, and the three input sources
        rospy.Service('change_wrench', WrenchSelect, self.change_wrench)
        self.wrench_pub = rospy.Publisher("/wrench/cmd", WrenchStamped, queue_size = 1)
        rospy.Subscriber("/wrench/rc", WrenchStamped, self.rc_cb)
        rospy.Subscriber("/wrench/autonomous", WrenchStamped, self.autonomous_cb)
        rospy.Subscriber("/wrench/gui", WrenchStamped, self.gui_cb)

    # These functions sit here and spin and collect the new messages

    def rc_cb(self, msg):
        self.rc_wrench = msg
        self.rc_wrench.header.frame_id = "/base_link"

    def autonomous_cb(self, msg):
        self.autonomous_wrench = msg
        self.autonomous_wrench.header.frame_id = "/base_link"

    def gui_cb(self, msg):
        self.gui_wrench = msg
        self.gui_wrench.header.frame_id = "/base_link"

    # When the change wrench service is called...
    def change_wrench(self, req):
        '''
            When the service is called the calling program either sends 'rc', 'autonomous', or 'gui'.
            This sets the wrench output to the correct source by setting the 'control' variable
            to the right source.

        '''

        rospy.loginfo("Server recieved request for wrench control change - " + req.str)

        if req.str == "rc":
            self.control = "rc"
            return True
        if req.str == "autonomous":
            self.control = "autonomous"
            return True
        if req.str == "gui":
            self.control = "gui"
            return True
        else:
            return False

    def publish_wrench(self):
        '''
            Publishes whichever wrench the control variable is set to
        '''

        if self.control == "rc":
            self.wrench_pub.publish(self.rc_wrench)

        if self.control == "autonomous":
            self.autonomous_wrench.header.frame_id = "/base_link"
            self.wrench_pub.publish(self.autonomous_wrench)

        if self.control == "gui":
            self.wrench_pub.publish(self.gui_wrench)


if __name__ == "__main__":

    arb = control_arb()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        arb.publish_wrench()
        rate.sleep()
    rospy.spin()
