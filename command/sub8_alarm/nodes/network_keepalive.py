#!/usr/bin/env python
import rospy
from std_msgs.msg import String


class KeepAlive(object):
    def __init__(self):
        self.pub = rospy.Publisher('/keep_alive', String, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.keepalive_pub)

    def keepalive_pub(self, *args):
        self.pub.publish(String('Nothing'))


if __name__ == '__main__':
    rospy.init_node('network_keepalive')
    ka = KeepAlive()
    rospy.spin()
