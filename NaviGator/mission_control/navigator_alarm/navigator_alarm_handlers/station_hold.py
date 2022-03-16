#!/usr/bin/env python
import rospy
from actionlib import TerminalState
from mil_missions_core import MissionClient
from ros_alarms import HandlerBase, AlarmBroadcaster


class StationHold(HandlerBase):
    alarm_name = 'station-hold'

    def __init__(self):
        self.task_client = MissionClient()
        self.broadcaster = AlarmBroadcaster(self.alarm_name)

    def _client_cb(self, terminal_state, result):
        if terminal_state != 3:
            rospy.logwarn('Station hold goal failed (Status={}, Result={})'.format(
                TerminalState.to_string(terminal_state), result.result))
            return
        rospy.loginfo('Station holding!')
        self.broadcaster.clear_alarm()

    def raised(self, alarm):
        rospy.loginfo("Attempting to station hold")
        self.task_client.run_mission('StationHold', done_cb=self._client_cb)

    def cleared(self, alarm):
        # When cleared, do nothing and just wait for new goal / custom wrench
        pass
