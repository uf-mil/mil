#!/usr/bin/env python3

import csv
import datetime
import sys
from typing import Dict, Union

import rclpy
from geometry_msgs.msg import PointStamped


class ClickedPointRecorder:
    """
    Listens to RVIZ clicked points, storing points in a CSV file. Support for running
    through `rosrun`.
    """

    def __init__(self):
        self.points = []
        self.point_sub = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.point_cb,
        )

    def point_to_dict(self, point: PointStamped) -> Dict[str, Union[str, int, float]]:
        """
        Returns a dictionary representing a point, where the values for the
        dict are obtained from the PointStamped message accepted.

        Args:
            point (PointStamped): The point message to accept.

        Returns:
            Dict[str, Union[str, int, float]]: Dictionary representing point and stamp.
        """
        return {
            "seq": point.header.seq,
            "secs": point.header.stamp.secs,
            "nsecs": point.header.stamp.nsecs,
            "frame_id": point.header.frame_id,
            "x": point.point.x,
            "y": point.point.y,
            "z": point.point.z,
        }

    def write_file(self) -> None:
        """
        Writes the points to a CSV file.
        """
        time = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        filename = f"clickedpoints{time}.csv"
        with open(filename, "wx") as csvfile:
            fieldnames = ["seq", "secs", "nsecs", "frame_id", "x", "y", "z"]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for p in self.points:
                d = self.point_to_dict(p)
                writer.writerow(d)
        self.get_logger().info(f"Writing points to {filename}")

    def point_cb(self, point: PointStamped) -> None:
        """
        Serves as a callback to the point subscriber. Accepts PointStamped
        messages and adds the messages to the internal points array.

        Args:
            point (PointStamped): The message input to the callback.
        """
        self.get_logger().info(f"Received new point: {point}")
        self.points.append(point)


if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("clicked_point_recorder")
    recorder = ClickedPointRecorder()

    def shutdown_cb():
        recorder.write_file()

    rclpy.on_shutdown(shutdown_cb)
    rclpy.spin()
