#!/usr/bin/env python3

import mil_ros_tools
import numpy as np
import rospy
import visualization_msgs.msg as visualization_msgs
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import ColorRGBA


class RvizVisualizer:
    """Cute tool for drawing both depth and height-from-bottom in RVIZ"""

    def __init__(self, topic="visualization/markers"):
        self.rviz_pub = rospy.Publisher(topic, visualization_msgs.Marker, queue_size=3)

    def draw_sphere(
        self,
        position,
        color,
        scaling=(0.11, 0.11, 0.11),
        _id=4,
        frame="/front_stereo",
    ):
        pose = Pose(
            position=mil_ros_tools.numpy_to_point(position),
            orientation=mil_ros_tools.numpy_to_quaternion([0.0, 0.0, 0.0, 1.0]),
        )

        marker = visualization_msgs.Marker(
            ns="sub",
            id=_id,
            header=mil_ros_tools.make_header(frame=frame),
            type=visualization_msgs.Marker.SPHERE,
            action=visualization_msgs.Marker.ADD,
            pose=pose,
            color=ColorRGBA(*color),
            scale=Vector3(*scaling),
            lifetime=rospy.Duration(),
        )
        self.rviz_pub.publish(marker)

    def draw_ray_3d(
        self,
        pix_coords,
        camera_model,
        color,
        frame="/front_stereo",
        _id=100,
        length=35,
        timestamp=None,
    ):
        """Handle range data grabbed from dvl"""
        # future: should be /base_link/dvl, no?
        marker = self.make_ray(
            base=np.array([0.0, 0.0, 0.0]),
            direction=np.array(camera_model.projectPixelTo3dRay(pix_coords)),
            length=length,
            color=color,
            frame=frame,
            timestamp=timestamp,
            _id=_id,
        )

        self.rviz_pub.publish(marker)

    def make_ray(
        self,
        base,
        direction,
        length,
        color,
        frame="/base_link",
        _id=100,
        timestamp=None,
        **kwargs,
    ):
        """Handle the frustration that Rviz cylinders are designated by their center, not base"""
        marker = visualization_msgs.Marker(
            ns="sub",
            id=_id,
            header=mil_ros_tools.make_header(frame=frame, stamp=timestamp),
            type=visualization_msgs.Marker.LINE_STRIP,
            action=visualization_msgs.Marker.ADD,
            color=ColorRGBA(*color),
            scale=Vector3(0.05, 0.05, 0.05),
            points=(Point(*o) for o in [base, direction * length]),
            lifetime=rospy.Duration(),
            **kwargs,
        )
        return marker
