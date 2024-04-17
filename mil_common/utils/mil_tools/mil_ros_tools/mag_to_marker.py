#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys

import numpy as np
import rclpy
from mil_tools import numpy_to_point, rosmsg_to_numpy
from sensor_msgs.msg import MagneticField
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

__author__ = "Kevin Allen"


class MagToMarker:
    """
    Node to subscribe to a MagneticField topic and publish a rviz marker with
    the same content. Used to get around the fact that rviz cannot display
    MagneticField messages.
    """

    color: ColorRGBA
    length: float

    def __init__(
        self,
        mag_topic: str,
        marker_topic: str,
        length: float = 1.0,
        color: list[int] | None = None,
    ):
        """
        Args:
            mag_topic (str): Topic of MagneticField to subscribe to.
            marker_topic (str): Topic of marker to publish.
            length (float): Length to scale vector to. Defaults to 1.0
            color (Optional[list[int]]): The color of the marker. Defaults to
                standard red.
        """
        if color is None:
            color = [0, 0, 1, 1]
        self.length = length
        self.pub = self.create_publisher(Marker, marker_topic, 1)
        self.color = ColorRGBA(*color)
        self.create_subscription(MagneticField, mag_topic, self.publish)

    def publish(self, vec: MagneticField):
        rclpy.logdebug("mag received")
        marker = Marker()
        marker.header = vec.header
        marker.type = 0
        marker.color = self.color
        marker.scale.x = 0.1
        marker.scale.y = 0.15
        marker.scale.z = 0.5
        marker.points.append(numpy_to_point([0, 0, 0]))
        vec = rosmsg_to_numpy(vec.magnetic_field)
        if self.length is not None:
            norm = np.linalg.norm(vec)
            if norm == 0:
                self.get_logger().warn("Zero vector received, skipping")
                return
            vec = (self.length / norm) * vec
        marker.points.append(numpy_to_point(vec))
        self.pub.publish(marker)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Publishes a rviz marker representing a MagneticField message",
    )
    parser.add_argument(
        "mag_topic",
        type=str,
        help="Topic of magnetic field to subscribe to",
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
    node = rclpy.create_node("mag_to_marker")
    MagToMarker(args.mag_topic, args.marker_topic, length=args.length, color=args.color)
    rclpy.spin()
