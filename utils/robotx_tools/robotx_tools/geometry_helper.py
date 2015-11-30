from __future__ import division
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Quaternion

'''
    A file to assist with some math that is commonly used in robotics

    Some of the functions here are shared with the UF Machine
    Intelligence Lab's SubjuGator robot. All hail Jacob Panikulum,
    may he reclaim his honor.

'''


def normalize(vector):
    return vector / np.linalg.norm(vector)


def compose_transformation(R, t):
    '''Compose a transformation from a rotation matrix and a translation matrix'''
    transformation = np.zeros((4, 4))
    transformation[:3, :3] = R
    transformation[3, :3] = t
    transformation[3, 3] = 1.0
    return transformation

def quat_to_euler(q):
    ''' Approximate a quaternion as a euler rotation vector'''

    euler_rot_vec = euler_from_quaternion([q.x, q.y, q.z, q.w])
    final = np.array(([euler_rot_vec[0], euler_rot_vec[1], euler_rot_vec[2]]))
    return final


def euler_to_quat(rotvec):
    ''' convert a euler rotation vector into a ROS quaternion '''

    quat = quaternion_from_euler(rotvec[0], rotvec[1], rotvec[2])
    return Quaternion(quat[0], quat[1], quat[2], quat[3])

 
