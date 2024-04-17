#!/usr/bin/env python3
"""
Publishes a Header message with the current time stamp
at a fixed interval. Useful for monitoring network loss
or for a safety network heartbeat.
"""
import sys

import rclpy
from std_msgs.msg import Header


class NetworkBroadcaster:
    """
    Serves as the main class broadcasting information through the network. Useful
    for monitoring network loss or for a safety network heartbeat. Can be run through
    `rosrun`.

    Attributes:
        msg (Header): The Header message which is recurrently published
        num_connections (int): The number of nodes connected to the publisher
        pub (rospy.Publisher): The Publisher publishing information, with a queue
          size of 1.
    """

    def __init__(self):
        hz = self.declare_parameter("~hz", 20)
        topic = self.declare_parameter("~topic", "network")

        self.get_logger().info(f"NETWORK BROADCASTER: publishing to {topic} at {hz}hz")
        self.msg = Header()
        self.msg.seq = 0
        self.num_connections = -1
        self.pub = self.create_publisher(Header, topic, 1, tcp_nodelay=True)
        rclpy.Timer(rclpy.Duration(1 / hz), self._publish)

    def _publish(self, *args):
        connections = self.pub.get_num_connections()
        if connections != self.num_connections:
            if connections == 0:
                node.get_logger().info("NETWORK BROADCASTER: no connections")
            else:
                node.get_logger().info(
                    f"NETWORK BROADCASTER: connected to {connections} nodes",
                )
            self.num_connections = connections
        self.msg.stamp = rclpy.Time.now()
        self.pub.publish(self.msg)
        self.msg.seq += 1


if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("network_broadcaster")

    NetworkBroadcaster()
    rclpy.spin()
