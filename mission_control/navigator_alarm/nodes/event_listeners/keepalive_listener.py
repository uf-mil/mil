#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from navigator_alarm import single_alarm
import time


class KeepAliveListener(object):
    '''
    Meant to only run on the boat. When the network is dropped, it triggers a kill.
    '''
    def __init__(self, timeout=5.0):
        self.timeout = rospy.Duration(timeout)
        self.last_msg = None 
        self.sub = rospy.Subscriber('/keep_alive', Header, self.got_network_msg, queue_size=1)

        self.alarm_broadcaster, self.alarm = single_alarm('network_loss', severity=3, problem_description="Network loss")
        rospy.Timer(rospy.Duration(0.1), self.check)

    def check(self, *args):
        if self.need_kill() and self.last_msg != '':
            self.alarm.raise_alarm()
            rospy.logwarn("NETWORK LOSS!")
        else:
            self.alarm.clear_alarm()

    def got_network_msg(self, msg):
        self.last_msg = msg

    def need_kill(self):
        if self.last_msg is None:
            return False
        return ((rospy.Time.now() - self.last_msg.stamp) > self.timeout)


if __name__ == '__main__':
    rospy.init_node('network_kill')
    KeepAliveListener()
    rospy.spin()
