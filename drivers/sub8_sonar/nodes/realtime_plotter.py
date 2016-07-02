#!/usr/bin/env python
import rospy
import rosparam

import numpy as np
from scipy import optimize

from sub8_msgs.srv import Sonar, SonarResponse

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

rospy.wait_for_service('~/sonar/get_pinger_pulse')
sonar =  rospy.ServiceProxy('~/sonar/get_pinger_pulse', Sonar)
print sonar, "serv proxy established"
i = 0
while(True):
	i += 1
	print "i:", i
	pinger_pose = sonar()
	print pinger_pose
	ax.scatter(pinger_pose.x, pinger_pose.y, pinger_pose.z)
	plt.draw()
	plt.pause(0.5)