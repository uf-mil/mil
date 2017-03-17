#!/usr/bin/env python
from __future__ import division

import numpy as np
import rospy
import visualization_msgs.msg as visualization_msgs

from geometry_msgs.msg import Pose, Vector3, Point
from std_msgs.msg import ColorRGBA
from uf_common.msg import Float64Stamped  # This needs to be deprecated

import navigator_tools

rviz_pub = rospy.Publisher("visualization", visualization_msgs.Marker, queue_size=3)

def draw_sphere(position, color, scaling=(0.11, 0.11, 0.11), m_id=4, frame='/base_link'):
    pose = Pose(
        position=navigator_tools.numpy_to_point(position),
        orientation=navigator_tools.numpy_to_quaternion([0.0, 0.0, 0.0, 1.0])
    )

    marker = visualization_msgs.Marker(
        ns='wamv',
        id=m_id,
        header=navigator_tools.make_header(frame=frame),
        type=visualization_msgs.Marker.SPHERE,
        action=visualization_msgs.Marker.ADD,
        pose=pose,
        color=ColorRGBA(*color),
        scale=Vector3(*scaling),
        lifetime=rospy.Duration(),
    )
    rviz_pub.publish(marker)

def draw_ray_3d(pix_coords, camera_model, color, frame='/stereo_front', m_id=0, length=35):
    marker = make_ray(
        base=np.array([0.0, 0.0, 0.0]),
        direction=np.array(camera_model.projectPixelTo3dRay(pix_coords)),
        length=length,
        color=color,
        frame=frame,
        m_id=m_id
    )

    rviz_pub.publish(marker)

def make_ray(base, direction, length, color, frame='/base_link', m_id=0, **kwargs):
    '''Handle the frustration that Rviz cylinders are designated by their center, not base'''
    marker = visualization_msgs.Marker(
        ns='wamv',
        id=m_id,
        header=navigator_tools.make_header(frame=frame),
        type=visualization_msgs.Marker.LINE_STRIP,
        action=visualization_msgs.Marker.ADD,
        color=ColorRGBA(*color),
        scale=Vector3(0.05, 0.05, 0.05),
        points=map(lambda o: Point(*o), [base, direction * length]),
        lifetime=rospy.Duration(),
        **kwargs
    )
    return marker