#!/usr/bin/env python
import rospy
from std_msgs.msg import String


class KeepAlive(object):
    def __init__(self, auto):
        self.pub = rospy.Publisher('/keep_alive', String, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.keepalive_pub)

    def keepalive_pub(self, *args):
        self.auto_mode = KeepAlive(rospy.get_param('/autonomous'))
        if self.auto_mode:
            self.pub.publish(String('auto'))
        else:
            self.pub.publish(String('keep_alive'))


if __name__ == '__main__':
    rospy.init_node('network_keepalive')
    ka = KeepAlive()
    rospy.spin()
