#!/usr/bin/env python
import rospy
from std_msgs.msg import Header


class NetworkBroadcaster(object):

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
