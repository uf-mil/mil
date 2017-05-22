#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from ros_alarms import AlarmBroadcaster, HandlerBase
__author__ = 'David Soto'


class OdomKill(HandlerBase):

    '''
    Will kill the sub when it stops hearing odometry messages or when there is a large
    discontinuity in state estimation. Only meant to run on the sub as a safety measure.
    '''

    def __init__(self, timeout=0.5):

        self.timeout = rospy.Duration(timeout)
        self.last_time = rospy.Time.now()
        self.last_position = None
        self.check_count = 0
        self.max_jump = 1.0
        self.odom_discontinuity = False
        self._killed = False
        self.odom_listener = rospy.Subscriber('/odom', Odometry, self.got_odom_msg, queue_size=1)
        self.ab = AlarmBroadcaster('odom-kill', node_name='odom-kill')
        rospy.Timer(rospy.Duration(0.1), self.check)

    def check(self, *args):
        self.check_count += 1
        if not self._killed and self.need_kill():
            if self.last_position is None:
                if self.check_count < 10:  # Probably just havent received odom yet
                    pass
                else:  # Odom is probably not publishing
                    self._killed = True
                    self.ab.raise_alarm(problem_description='STATE ESTIMATION NOT AVAILABLE: KILLING SUB',
                                        severity=5
                                        )
                    rospy.logerr('STATE ESTIMATION NOT AVAILABLE: KILLING SUB')
            else:
                self._killed = True
                self.ab.raise_alarm(problem_description='STATE ESTIMATION LOSS: KILLING SUB',
                                    severity=5
                                    )
                rospy.logerr("STATE ESTIMATION LOSS: KILLING SUB")

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
        jump = ((this_p.x - last_p.x) ** 2 + (this_p.y - last_p.y) ** 2 + (this_p.z - last_p.z) ** 2) ** 0.5
        if jump > self.max_jump:
            rospy.logerr('ODOM DISCONTINUITY DETECTED')
            self.ab.raise_alarm(problem_description='ODOM DISCONTINUITY DETECTED JUMPED {} METERS'.format(jump),
                                severity=5)
            self.odom_discontinuity = True
            return False

    def need_kill(self):
        odom_loss = rospy.Time.now() - self.last_time > self.timeout
        if odom_loss:
            rospy.logerr('LOST ODOM FOR {} SECONDS'.format((rospy.Time.now() - self.last_time).to_sec()))
            self.ab.raise_alarm(
                problem_description='LOST ODOM FOR {} SECONDS'.format((rospy.Time.now() - self.last_time).to_sec()),
                severity=5
            )
        return odom_loss or self.odom_discontinuity

    def clear_kill(self, alarm):
        msg = ""
        if alarm.clear:
            if self._killed:
                self._killed = False
                self.odom_discontinuity = False
                msg = "Odom kill successfully cleared!"
            else:
                msg = "Attempted to clear odom kill, but was already not killed."
            rospy.logwarn(msg)

    def raised(self, alarm):
        self._killed = True

    def cleared(self, alarm):
        self._killed = False
