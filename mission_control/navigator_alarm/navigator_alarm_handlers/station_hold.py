#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient, TerminalState
from navigator_path_planner.msg import MoveAction, MoveGoal
from topic_tools.srv import MuxSelect
from ros_alarms import HandlerBase, AlarmBroadcaster


class StationHold(HandlerBase):
    alarm_name = 'station-hold'

    def __init__(self):
        self.move_client = SimpleActionClient('/move_to', MoveAction)
        self.change_wrench = rospy.ServiceProxy('/wrench/select', MuxSelect)
        self.broadcaster = AlarmBroadcaster(self.alarm_name)

    def _client_cb(self, terminal_state, result):
        if terminal_state != 3:
            rospy.logwarn('Station hold goal failed (Status={}, Result={})'.format(
                TerminalState.to_string(terminal_state), result))
            return
        try:
            self.change_wrench('autonomous')
            rospy.loginfo('Now station holding')
            self.broadcaster.clear_alarm()
        except rospy.ServiceException as e:
            rospy.logwarn('Station hold changing wrench failed: {}'.format(e))

    def raised(self, alarm):
        rospy.loginfo("Attempting to station hold")
        station_hold_goal = MoveGoal()
        station_hold_goal.move_type = 'hold'
        self.move_client.send_goal(station_hold_goal, done_cb=self._client_cb)

    def cleared(self, alarm):
        # When cleared, do nothing and just wait for new goal / custom wrench
        pass
