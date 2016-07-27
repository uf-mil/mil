#!/usr/bin/env python
import rospy
from std_msgs.msg import String


class KeepAlive(object):
    def __init__(self):
        self.pub = rospy.Publisher('/keep_alive', String, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.keepalive_pub)

    def keepalive_pub(self, *args):
        if rospy.get_param('/autonomous'):
            self.pub.publish(String('auto'))
        else:
            self.pub.publish(String('keep_alive'))


if __name__ == '__main__':
    rospy.init_node('network_keepalive')
    ka = KeepAlive()
    rospy.spin()
