#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from navigator_alarm import single_alarm, AlarmListener
from twisted.internet import reactor
import time


class OdomKillListener(object):
    '''
    Will kill the boat when it stops hearing odometry messages or when there is a large
    discontinuity in state estimation. Only meant to run on the boat as a safety measure.
    '''
    def __init__(self, timeout=0.5):
        self.timeout = rospy.Duration(timeout)
        self.last_time = rospy.Time.now()
        self.last_position = None
        self.check_count = 0
        self.max_jump = 1.0
        self.odom_discontinuity = False
        self.killed = False
        self.odom_listener = rospy.Subscriber('/odom', Odometry, self.got_odom_msg, queue_size=1)
        self.alarm_broadcaster, self.alarm = single_alarm('odom_loss', severity=3,
                                                          problem_description="Killing wamv due to unreliable state estimation")
        rospy.Timer(rospy.Duration(0.1), self.check)

    def check(self, *args):
        self.check_count += 1
        if not self.killed and self.need_kill():
            if self.last_position is None:
                if self.check_count < 10: # Probably just havent received odom yet
                    pass
                else:  # Odom is probably not publishing
                    self.killed = True
                    self.alarm.raise_alarm()
                    rospy.logerr('STATE ESTIMATION NOT AVAILABLE: KILLING BOAT')
            else:
                self.killed = True
                self.alarm.raise_alarm()
                rospy.logerr("STATE ESTIMATION LOSS: KILLING BOAT")
    
    def got_odom_msg(self, msg):
        if self.last_position is not None:
            self.check_continuity(msg)
        self.last_position = msg.pose.pose.position
        self.last_time = rospy.Time.now()

    def check_continuity(self, new_odom_msg):  # True if 'continuous'
        if self.odom_discontinuity:
            return

        this_p = new_odom_msg.pose.pose.position
        last_p = self.last_position
        jump = ((this_p.x - last_p.x) ** 2 + (this_p.y - last_p.y) ** 2) ** 0.5
        if jump > self.max_jump:
            rospy.logerr('ODOM DISCONTINUITY DETECTED')
            self.odom_discontinuity = True
            return False

    def need_kill(self):  
        odom_loss =  rospy.Time.now() - self.last_time > self.timeout
        if odom_loss:
            rospy.logerr('LOST ODOM FOR {} SECONDS'.format((rospy.Time.now() - self.last_time).to_sec()))
        return odom_loss or self.odom_discontinuity

    def clear_kill(self, alarm):
        msg = ""
        if alarm.clear:
            if self.killed:
                self.killed = False
                self.odom_discontinuity = False
                msg = "Odom kill successfully cleared!"
            else:
                msg = "Attempted to clear odom kill, but was already not killed."
            rospy.logwarn(msg)
       
        
if __name__ == '__main__':
    rospy.init_node('odom_kill')
    mattfucious = OdomKillListener()
    rospy.spin()
