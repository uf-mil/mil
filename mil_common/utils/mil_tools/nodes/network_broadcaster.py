#!/usr/bin/env python3
"""
Publishes a Header message with the current time stamp
at a fixed interval. Useful for monitoring network loss
or for a safety network heartbeat.
"""
import rospy
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
        hz = rospy.get_param("~hz", 20)
        topic = rospy.get_param("~topic", "network")

        rospy.loginfo(f"NETWORK BROADCASTER: publishing to {topic} at {hz}hz")
        self.msg = Header()
        self.msg.seq = 0
        self.num_connections = -1
        self.pub = rospy.Publisher(topic, Header, queue_size=1, tcp_nodelay=True)
        rospy.Timer(rospy.Duration(1 / hz), self._publish)

    def _publish(self, *args):
        connections = self.pub.get_num_connections()
        if connections != self.num_connections:
            if connections == 0:
                rospy.loginfo("NETWORK BROADCASTER: no connections")
            else:
                rospy.loginfo(f"NETWORK BROADCASTER: connected to {connections} nodes")
            self.num_connections = connections
        self.msg.stamp = rospy.Time.now()
        self.pub.publish(self.msg)
        self.msg.seq += 1


if __name__ == "__main__":
    rospy.init_node("network_broadcaster", anonymous=True)
    NetworkBroadcaster()
    rospy.spin()
