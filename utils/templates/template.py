#!/usr/bin/env python
'''

This source is written for use in the Machine Intelligence Lab in the MAE
Department at the University of Florida. 
It is writen for use on the UF ___________ robot
It is released under the BSD license and is intended for university use
This code is provided "as is" and relies on specific hardware, use at your own risk

Title: This is your title
Start Date: Date

Author: 
Author email: 

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
import numpy,math,tf,threading
from kill_handling.listener import KillListener
from kill_handling.broadcaster import KillBroadcaster

rospy.init_node('NODE_NAME')

class CLASS_NAME(object):
	# Base class for whatever you are writing
	def __init__(self):
		# Initilization code
		self.killed = False
		self.kill_listener = KillListener(self.set_kill, self.clear_kill)

    def set_kill(self):
        self.killed = True

    def clear_kill(self):
        self.killed = False

	def state_status(self):
		# State info to go here

if __name__ == "__main__":
	#
	CLASS_INSTANCE = CLASS_NAME()
    rospy.Timer(rospy.Duration(1), CLASS_INSTANCE.state_status)
    rospy.spin()
