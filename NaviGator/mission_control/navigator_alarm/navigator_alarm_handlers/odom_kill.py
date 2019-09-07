#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from ros_alarms import AlarmBroadcaster, HandlerBase, HeartbeatMonitor
import numpy as np
from mil_ros_tools import rosmsg_to_numpy
__author__ = 'David Soto'


class OdomKill(HandlerBase):
    '''
    Alarm raised when either Odometry has not been published in a while or the euclidean distance
    between the positions in the last two messages is too large, indicating a sensor error or other
    state estimation issue that will cause unsafe behavior
    '''

    alarm_name = 'odom-kill'
    TIMEOUT_SECONDS = 1.0  # Max time delta between Odometry messages before alarm is raised
    MAX_DISTANCE_METERS = 0.5  # Max distance in position between Odometry messages before alarm is raised

    def __init__(self):
        self.hm = HeartbeatMonitor(self.alarm_name, "/odom", Odometry,
                                   node_name="alarm_server", prd=self.TIMEOUT_SECONDS)
        self.MAX_JUMP = 0.5
        self.launch_time = rospy.Time.now()
        self.last_time = self.launch_time
        self.last_position = None
        self._raised = False
        self.ab = AlarmBroadcaster('odom-kill', node_name='odom-kill')
        rospy.Subscriber('/odom', Odometry, self.check_continuity, queue_size=5)

    def check_continuity(self, odom):
        '''
        On new odom message, find distance in position between this message and the last one.
        If the distance is > MAX_JUMP, raise the alarm
        '''
        position = rosmsg_to_numpy(odom.pose.pose.position)
        if self.last_position is not None:
            jump = np.linalg.norm(position - self.last_position)
            if jump > self.MAX_JUMP and not self._raised:
                self._raised = True  # Avoid raising multiple times
                rospy.logwarn('ODOM DISCONTINUITY DETECTED')
                self.ab.raise_alarm(problem_description='ODOM DISCONTINUITY DETECTED. JUMPED {} METERS'.format(jump),
                                    severity=5)
        self.last_position = position
        self.last_time = odom.header.stamp

    def raised(self, alarm):
        self._raised = True

    def cleared(self, alarm):
        self._raised = False
