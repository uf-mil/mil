#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient, TerminalState
from lqrrt_ros.msg import MoveAction, MoveGoal
from navigator_msgs.srv import WrenchSelect, WrenchSelectRequest
from ros_alarms import HandlerBase

class StationHold(HandlerBase):
    alarm_name = 'station-hold'

    def __init__(self):
        self.move_client = SimpleActionClient('/move_to', MoveAction)
        self.change_wrench = rospy.ServiceProxy('/change_wrench', WrenchSelect)

    def _client_cb(self, terminal_state, result):
        if terminal_state != 3:
            rospy.logwarn('Hold goal failed (Status={}, Failure Reason={}'.format(TerminalState.to_string(status), result.failure_reason))
            return
        try:
            self.change_wrench('autonomous')
        except rospy.ServiceException as e:
            rospy.logwarn('Error changing wrench: {}'.format(e))

    def raised(self, alarm):
        station_hold_goal = MoveGoal()
        station_hold_goal.move_type = 'hold'
        self.move_client.send_goal(station_hold_goal, done_cb=self._client_cb)

    def cleared(self, alarm):
        # When cleared, do nothing and just wait for new goal / custom wrench
        pass


