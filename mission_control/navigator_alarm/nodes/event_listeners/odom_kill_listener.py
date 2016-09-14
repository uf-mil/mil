#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from navigator_alarm import single_alarm
from twisted.internet import reactor
import time
import nav_msgs.msg import Odometry


class OdomKillListener(object):
    '''
    Will kill the boat when it stops hearing odometry messages or when there is a large
    discontinuity in state estimation. Only meant to run on the boad as a safety measure.
    '''
    def __init__(self, timeout=0.5):
        self.timeout = rospy.Duration(timeout)
        self.last_time = rospy.Time.now()
        self.last_position = None
        self.max_jump = 1.0
        self.odom_discontinuity = False
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
        self.odom_discontinuity = not check_continuity(msg)
        self.last_position = msg.pose.pose.position
        self.last_time = rospy.Time.now()

    def need_kill(self):
        odom_loss =  ((rospy.Time.now() - self.last_time) > self.timeout)
        return odom_loss or self.odom_discontinuity

    def check_continuity(new_odom_msg):
        this_p = new_odom_msg.pose.pose.position
        last_p = self.last_position
        d_sq = (this_p.x - last_p.x) ** 2 + (this_p.y - last_p.y) ** 2
        return d_sq ** 0.5 > self.max_jump
       
        
if __name__ == '__main__':
    rospy.init_node('odom_kill')
    mattfucious = OdomKillListener()
    rospy.spin()
