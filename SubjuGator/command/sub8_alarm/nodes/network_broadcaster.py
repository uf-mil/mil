#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header


class NetworkBroadcaster:
    """
    Class to push header messages to the network topic. Heartbeat monitors check
    for recurring messages in other classes.
    """

    def __init__(self):
        self.pub = rospy.Publisher("/network", Header, queue_size=1)

        rospy.Timer(rospy.Duration(0.05), self._publish)

    def _publish(self, *args):
        h = Header()
        h.stamp = rospy.Time.now()
        self.pub.publish(h)


if __name__ == "__main__":
    rospy.init_node("network_broadcaster")
    nb = NetworkBroadcaster()
    rospy.spin()
