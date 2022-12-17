#!/usr/bin/env python3
import mil_ros_tools
import numpy as np
import rospy
import tf
from sensor_msgs.msg import PointCloud

rospy.init_node("ground_finder")

pub = rospy.Publisher("ground_est", PointCloud, queue_size=1)

listener = tf.TransformListener()
pc = PointCloud()
pc.header = mil_ros_tools.make_header(frame="map")
pc.points = []

rate = rospy.Rate(1.0)
while not rospy.is_shutdown():
    try:
        t = listener.waitForTransform(
            "map", "/ground", rospy.Time.now(), rospy.Duration(1)
        )
        (trans, rot) = listener.lookupTransform("map", "/ground", rospy.Time(0))
    except (
        tf.Exception,
        tf.LookupException,
        tf.ConnectivityException,
        tf.ExtrapolationException,
    ):
        rospy.logwarn("TF waitForTransform timeout")
        continue

    pc.points.append(mil_ros_tools.numpy_to_point(np.array(trans)))
    pub.publish(pc)

    rate.sleep()
