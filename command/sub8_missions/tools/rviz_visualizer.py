#!/usr/bin/env python
from __future__ import division

import numpy as np
import rospy
import visualization_msgs.msg as visualization_msgs

from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import ColorRGBA
from uf_common.msg import Float64Stamped  # This needs to be deprecated

import sub8_ros_tools as sub8_utils


class RvizVisualizer(object):
    '''Cute tool for drawing both depth and height-from-bottom in RVIZ
    '''

    def __init__(self):
        rospy.init_node('revisualizer')
        self.rviz_pub = rospy.Publisher("visualization/state", visualization_msgs.Marker, queue_size=2)
        self.rviz_pub_t = rospy.Publisher("visualization/state_t", visualization_msgs.Marker, queue_size=2)

        # text marker
        # TODO: Clean this up, there should be a way to set all of this inline
        self.surface_marker = visualization_msgs.Marker()
        self.surface_marker.type = self.surface_marker.TEXT_VIEW_FACING
        self.surface_marker.color = ColorRGBA(1, 1, 1, 1)
        self.surface_marker.scale.z = 0.1

        self.depth_marker = visualization_msgs.Marker()
        self.depth_marker.type = self.depth_marker.TEXT_VIEW_FACING
        self.depth_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        self.depth_marker.scale.z = 0.1

        # distance to bottom
        self.range_sub = rospy.Subscriber("dvl/range", Float64Stamped, self.range_callback)
        # distance to surface
        self.depth_sub = rospy.Subscriber("depth", Float64Stamped, self.depth_callback)

    def depth_callback(self, msg):
        '''Handle depth data sent from depth sensor'''
        frame = '/depth'
        distance = msg.data
        marker = self.make_cylinder_marker(
            np.array([0.0, 0.0, 0.0]),  # place at origin
            length=distance,
            color=(0.0, 1.0, 0.2, 0.7),  # green,
            frame=frame,
            id=0  # Keep these guys from overwriting eachother
        )
        self.surface_marker.ns='sub'
        self.surface_marker.header = sub8_utils.make_header(frame='/depth')
        self.surface_marker.pose = marker.pose
        self.surface_marker.text = str(round(distance, 3)) + 'm'
        self.surface_marker.id = 0

        self.rviz_pub.publish(marker)
        self.rviz_pub_t.publish(self.depth_marker)

    def range_callback(self, msg):
        '''Handle range data grabbed from dvl'''
        # future: should be /base_link/dvl, no?
        frame = '/dvl'
        distance = msg.data

        # Color a sharper red if we're in danger
        if distance < 1.0:
            color = (1.0, 0.1, 0.0, 0.9)
        else:
            color = (0.2, 0.8, 0.0, 0.7)

        marker = self.make_cylinder_marker(
            np.array([0.0, 0.0, 0.0]),  # place at origin
            length=distance,
            color=color,  # red,
            frame=frame,
            up_vector=np.array([0.0, 0.0, -1.0]),  # up is down in range world
            id=1  # Keep these guys from overwriting eachother
        )
        self.depth_marker.ns='sub'
        self.depth_marker.header = sub8_utils.make_header(frame='/dvl')
        self.depth_marker.pose = marker.pose
        self.depth_marker.text = str(round(distance, 3)) + 'm'
        self.depth_marker.id = 1

        self.rviz_pub_t.publish(self.depth_marker)
        self.rviz_pub.publish(marker)

    def make_cylinder_marker(self, base, length, color, frame='/base_link', up_vector=np.array([0.0, 0.0, 1.0]), **kwargs):
        '''Handle the frustration that Rviz cylinders are designated by their center, not base'''

        center = base + (up_vector * (length / 2))

        pose = Pose(
            position=sub8_utils.numpy_to_point(center),
            orientation=sub8_utils.numpy_to_quaternion([0.0, 0.0, 0.0, 1.0])
        )

        marker = visualization_msgs.Marker(
            ns='sub',
            header=sub8_utils.make_header(frame=frame),
            type=visualization_msgs.Marker.CYLINDER,
            action=visualization_msgs.Marker.ADD,
            pose=pose,
            color=ColorRGBA(*color),
            scale=Vector3(0.2, 0.2, length),
            lifetime=rospy.Duration(),
            **kwargs
        )
        return marker


if __name__ == '__main__':
    vizualizer = RvizVisualizer()
    rospy.spin()
