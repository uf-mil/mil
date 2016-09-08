#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from navigator_alarm import single_alarm
from twisted.internet import reactor
import time


class OdomKillListener(object):
    '''
    Will kill the boat when it stops hearing odometry messages or when there is a large
    discontinuity in state estimation. Only meant to run on the boad as a safety measure.
    '''
    def __init__(self, timeout=0.5):
        self.timeout = rospy.Duration(timeout)
        self.last_time = rospy.Time.now()
        self.last_msg = None
        self.sub = rospy.Subscriber('/odom', String, self.got_network_msg, queue_size=1)

        self.alarm_broadcaster, self.alarm = single_alarm('kill', severity=3,
                                                          problem_description="Fatal state estimation error")
        rospy.Timer(rospy.Duration(0.1), self.check)

    def check(self, *args):
        if self.need_kill() and self.last_msg != None:
            self.alarm.raise_alarm()
            rospy.logerr("STATE ESTIMATION LOSS: KILLING BOAT")
        else:
            self.alarm.clear_alarm()

    def got_odom_msg(self, msg):
        self.last_msg = msg.data
        self.last_time = rospy.Time.now()

    def need_kill(self):
        return ((rospy.Time.now() - self.last_time) > self.timeout)


if __name__ == '__main__':
    rospy.init_node('odom_kill')
    mattfucious = OdomKillListener()
    rospy.spin()