#!/usr/bin/env python3
import argparse
import sys
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from mil_tools import numpy_to_colorRGBA, numpy_to_point, rosmsg_to_numpy
from visualization_msgs.msg import Marker

__author__ = "Kevin Allen"


class VectorToMarker:
    """
    Node to subscribe to a Vector3Stamped topic and publish a rviz marker
    with the same content. Used to get around the fact that rviz cannot display
    Vector3Stamped messages.

    Can be used from the command line through :mod:`argparse` support.

    Args:
        vector_topic (str): Topic of vector3stamped to subscribe to.
        marker_topic (str): Topic of marker to publish.
        length (float): Length to scale vector to. If ``None``, leave original scale.
        color (List[float]): The color of the marker. The list should contain the ``r``,
            ``g``, ``b``, and ``a`` values of the color.

    Attributes:
        length (Optional[float]): The length to scale the marker to upon receival.
        pub (rospy.Publisher): The publisher responsible for publishing the :class:`Marker`
            messages.
    """

    def __init__(
        self,
        vector_topic: str,
        marker_topic: str,
        length: Optional[float] = 1.0,
        color: List[float] = [0, 0, 1, 1],
    ):
        self.length = length
        self.pub = self.create_publisher(Marker, marker_topic, 1)
        self.color = numpy_to_colorRGBA(color)
        self.create_subscription(Vector3Stamped, vector_topic, self.publish)

    def publish(self, vec):
        rclpy.logdebug("vector received")
        marker = Marker()
        marker.header = vec.header
        marker.type = 0
        marker.color = self.color
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = 0.1
        marker.scale.y = 0.15
        marker.scale.z = 0.5
        marker.points.append(numpy_to_point([0, 0, 0]))
        vec = rosmsg_to_numpy(vec.vector)
        if self.length is not None:
            norm = np.linalg.norm(vec)
            if norm == 0:
                self.get_logger().warn("Zero vector received, skipping")
                return
            vec = (self.length / norm) * vec
        marker.points.append(numpy_to_point(vec))
        self.pub.publish(marker)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Publishes a rviz marker")
    parser.add_argument(
        "vector_topic",
        type=str,
        help="Topic of vector to subscribe to",
    )
    parser.add_argument(
        "marker_topic",
        type=str,
        nargs="?",
        default="marker",
        help="Topic to publish marker to",
    )
    parser.add_argument(
        "--length",
        "-l",
        type=float,
        default=None,
        help="Length of vector to output. Default: do not scale",
    )
    parser.add_argument(
        "--color",
        type=float,
        nargs=4,
        default=[0, 0, 1, 1],
        metavar=("R", "G", "B", "A"),
        help="Color of vector to publish as floats RGBA 0 to 1",
    )
    args = rclpy.myargv()
    args = parser.parse_args(args[1:])
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("vector_to_marker")

    VectorToMarker(
        args.vector_topic,
        args.marker_topic,
        length=args.length,
        color=args.color,
    )
    rclpy.spin()
