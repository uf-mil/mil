#!/usr/bin/env python
from __future__ import division

import numpy as np
import rospy
import visualization_msgs.msg as visualization_msgs
from geometry_msgs.msg import Pose, Vector3, Point
from std_msgs.msg import ColorRGBA
import sub8_ros_tools as sub8_utils


class RvizVisualizer(object):
    '''Cute tool for drawing both depth and height-from-bottom in RVIZ
    '''

    def __init__(self):
        self.rviz_pub = rospy.Publisher("visualization/projection", visualization_msgs.Marker, queue_size=3)

    def draw_sphere(self, position, color, scaling=(0.11, 0.11, 0.11), _id=4, frame='/stereo_front'):
        pose = Pose(
            position=sub8_utils.numpy_to_point(position),
            orientation=sub8_utils.numpy_to_quaternion([0.0, 0.0, 0.0, 1.0])
        )

        marker = visualization_msgs.Marker(
            ns='sub',
            id=_id,
            header=sub8_utils.make_header(frame=frame),
            type=visualization_msgs.Marker.SPHERE,
            action=visualization_msgs.Marker.ADD,
            pose=pose,
            color=ColorRGBA(*color),
            scale=Vector3(*scaling),
            lifetime=rospy.Duration(),
        )
        self.rviz_pub.publish(marker)

    def draw_ray_3d(self, pix_coords, camera_model, color, frame='/stereo_front', _id=100):
        '''Handle range data grabbed from dvl'''
        # future: should be /base_link/dvl, no?
        marker = self.make_ray(
            base=np.array([0.0, 0.0, 0.0]),
            direction=np.array(camera_model.projectPixelTo3dRay(pix_coords)),
            length=35.0,
            color=color,
            frame=frame,
            _id=_id
        )

        self.rviz_pub.publish(marker)

    def make_ray(self, base, direction, length, color, frame='/base_link', _id=100, **kwargs):
        '''Handle the frustration that Rviz cylinders are designated by their center, not base'''
        marker = visualization_msgs.Marker(
            ns='sub',
            id=_id,
            header=sub8_utils.make_header(frame=frame),
            type=visualization_msgs.Marker.LINE_STRIP,
            action=visualization_msgs.Marker.ADD,
            color=ColorRGBA(*color),
            scale=Vector3(0.05, 0.0, 0.0),
            points=map(lambda o: Point(*o), [base, direction * length]),
            lifetime=rospy.Duration(),
            **kwargs
        )
        return marker
