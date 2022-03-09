#!/usr/bin/env python3
"""
Listens to RVIZ clicked points, storing points in a csv file

Usage: rosrun mil_tools clicked_point_recorder.py
"""

import rospy
import datetime
import csv
from geometry_msgs.msg import PointStamped
from typing import Dict, Union


class ClickedPointRecorder:
    def __init__(self):
        self.points = []
        self.point_sub = rospy.Subscriber("/clicked_point", PointStamped, self.point_cb)

    def point_to_dict(self, point: PointStamped) -> Dict[str, Union[str, int, float]]:
        """
        Returns a dictionary representing a point, where the values for the
        dict are obtained from the PointStamped message accepted.

        Args:
            point: PointStamped - The point message to accept.
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
        filename = "clickedpoints{}.csv".format(time)
        with open(filename, "wx") as csvfile:
            fieldnames = ["seq", "secs", "nsecs", "frame_id", "x", "y", "z"]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for p in self.points:
                d = self.point_to_dict(p)
                writer.writerow(d)
        rospy.loginfo("Writing points to {}".format(filename))

    def point_cb(self, point: PointStamped) -> None:
        """
        Serves as a callback to the point subscriber. Accepts PointStamped
        messages and adds the messages to the internal points array.

        Args:
            point: PointStamped - The message
        """
        rospy.loginfo("Received new point: {}".format(point))
        self.points.append(point)


if __name__ == "__main__":
    rospy.init_node("clicked_point_recorder")
    recorder = ClickedPointRecorder()

    def shutdown_cb():
        recorder.write_file()

    rospy.on_shutdown(shutdown_cb)
    rospy.spin()
