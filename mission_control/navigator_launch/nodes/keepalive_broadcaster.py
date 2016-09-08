#!/usr/bin/env python
import rospy
from std_msgs.msg import String


class KeepAliveBroadcaster(object):
    '''
    Meant to run from any number of remote computers.
    Make sure the wifi link is somewhere between the computers running the keepalive
        script and the boat (or else there is no point!)
    AKA, don't run this on the boat.
    '''
    def __init__(self):
        self.pub = rospy.Publisher('/keep_alive', String, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.keepalive_pub)

    def keepalive_pub(self, *args):
        self.pub.publish(String('keep_alive'))


if __name__ == '__main__':
    rospy.init_node('network_keepalive')
    KeepAliveBroadcaster()
    rospy.spin()
