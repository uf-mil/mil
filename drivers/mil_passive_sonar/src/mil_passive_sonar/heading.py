#!/usr/bin/env python
from __future__ import division

import numpy as np
from mil_passive_sonar.msg import ProcessedPing
import rospy
from geometry_msgs.msg import Vector3Stamped
import mil_ros_tools
import collections
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_geometry_msgs import Vector3Stamped, PointStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Vector3
from mil_passive_sonar.srv import FindPinger, FindPingerResponse
from std_srvs.srv import SetBool, SetBoolResponse


class Heading(object):
    def __init__(self):
        self.processed_ping_sub = rospy.Subscriber('/hydrophones/processed', ProcessedPing, self.processed_ping_cb, queue_size=10)

        self.heading_pub = rospy.Publisher('~heading', Vector3Stamped, queue_size=5)

	self.find_pinger_srv = rospy.Service('~find_pinger', FindPinger, self.get_intersection_cb)
	self.record_pinger_srv = rospy.Service('~record_pinger', SetBool, self.record_pinger_cb)
 

        self.frequency = 30000

        self.vec_samples = collections.deque(maxlen=20)


        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

	

	self.recording = False

    def record_pinger_cb(self, req):
	self.recording = req.data
	return SetBoolResponse(True, "ping ping")


    def processed_ping_cb(self, p_message):
	if not self.recording:
	    return
        p_position = mil_ros_tools.rosmsg_to_numpy(p_message.position)
        vec = p_position / np.linalg.norm(p_position)

        if np.isnan(vec).any():
            return
	
	vec_stamped = Vector3Stamped(Header(stamp=rospy.Time.now(), frame_id='hydrophones'), Vector3(*vec))
	origin_stamped = Vector3Stamped(Header(stamp=rospy.Time.now(), frame_id='hydrophones'), Point(0,0,0))
	transformed_vec = self.tfBuffer.transform(vec_stamped, "base_link", rospy.Duration(2))
	transformed_origin = self.tfBuffer.transform(origin_stamped, "base_link", rospy.Duration(2))
	self.vec_samples.append((transformed_origin, transformed_vec))

    def get_intersection_cb(self, req):
	start = np.array([mil_ros_tools.rosmsg_to_numpy(p[0].vector)[:2] for p in self.vec_samples])
	end = np.array([mil_ros_tools.rosmsg_to_numpy(p[1].vector)[:2] for p in self.vec_samples])
	
	where = self.ls_line_intersection2d(start, end)
	return FindPingerResponse(Point(where[0], where[1], 0), len(self.vec_samples))
		
	
    def ls_line_intersection2d(self, start, end):
        """
        Find the intersection of lines in the least-squares sense.
        start - Nx3 numpy array of start points
        end - Nx3 numpy array of end points
        https://en.wikipedia.org/wiki/Line-line_intersection#In_two_dimensions_2
        """
        if len(start) != len(end):
            raise RuntimeError('Dimension mismatch')
        if len(start) < 2:
            raise RuntimeError('Insufficient line count')
        dir_vecs = end - start
        lengths = np.linalg.norm(dir_vecs).reshape((-1,1))
        dir_vecs = dir_vecs / lengths
        Rl_90 = np.array([[0, -1], [1, 0]])  # rotates right 90deg
        perp_unit_vecs = Rl_90.dot(dir_vecs.T).T
        A_sum = np.zeros((2, 2))
        Ap_sum = np.zeros((2, 1))
    
        for x, y in zip(start, perp_unit_vecs):
            A = y.reshape(2, 1).dot(y.reshape(1, 2))
            Ap = A.dot(x.reshape(2, 1))
            A_sum += A
            Ap_sum += Ap
    
	return np.linalg.lstsq(A, Ap_sum, rcond=None)[0]

if __name__ == "__main__":
    rospy.init_node("passive_sonar_driver")
    ping_ping_motherfucker = Heading()
    rospy.spin()
    
        



