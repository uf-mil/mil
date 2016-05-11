#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sub8_alarm import single_alarm


class KeepAlive(object):
    def __init__(self, timeout=5.0):
        self.timeout = rospy.Duration(timeout)
        self.last_time = rospy.Time.now()
        self.sub = rospy.Subscriber('/keep_alive', String, self.got_keepalive, queue_size=1)
        self.alarm_broadcaster, self.alarm = single_alarm('network-timeout', severity=1)
        rospy.Timer(rospy.Duration(0.01), self.check)

    def check(self, *args):
        if self.need_kill():
            self.alarm.raise_alarm()
        else:
            self.alarm.clear_alarm()

    def got_keepalive(self, msg):
        self.last_time = rospy.Time.now()

    def need_kill(self):
        if (rospy.Time.now() - self.last_time) > self.timeout:
            return True
        else:
            return False


if __name__ == '__main__':
    rospy.init_node('network_kill')
    ka = KeepAlive(timeout=0.1)
    rospy.spin()
