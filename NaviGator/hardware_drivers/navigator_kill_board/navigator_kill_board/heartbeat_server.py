#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header


class HeartbeatServer:
    def __init__(self, topic: str, period: float):
        self.pub = rospy.Publisher(topic, Header, queue_size=10)

        self.period = period
        rospy.Timer(rospy.Duration(period / 2), self.publish)

    def publish(self, *args):
        self.pub.publish(Header())


if __name__ == "__main__":
    rospy.init_node("heartbeat_server")
    server = HeartbeatServer("/network", 1.0)
    rospy.spin()
