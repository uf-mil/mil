#!/usr/bin/env python
import rospy
import numpy as np
import tf
import sub8_ros_tools
from sensor_msgs.msg import PointCloud


rospy.init_node("ground_finder")

pub = rospy.Publisher("ground_est", PointCloud, queue_size=1)

listener = tf.TransformListener()
pc = PointCloud()
pc.header = sub8_ros_tools.make_header(frame="map")
pc.points = []

rate = rospy.Rate(1.0)
while not rospy.is_shutdown():
    try:
        t = listener.waitForTransform('/map', '/ground', rospy.Time.now(), rospy.Duration(1))
        (trans,rot) = listener.lookupTransform('/map', '/ground', rospy.Time(0))
    except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("TF waitForTransform timeout")
        continue
    
    pc.points.append(sub8_ros_tools.numpy_to_point(np.array(trans)))
    pub.publish(pc)

    rate.sleep()
