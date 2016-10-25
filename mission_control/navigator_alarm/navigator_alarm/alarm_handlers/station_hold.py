#!/usr/bin/env python
import rospy
import actionlib
from _template import HandlerBase

from lqrrt_ros.msg import MoveAction, MoveGoal

class StationHoldHandler(HandlerBase):
    alarm_name = 'station_hold'

    def __init__(self):
        self.client = actionlib.SimpleActionClient('/move_to', MoveAction)
        self.station_hold_alarm = self.alarm_broadcaster.add_alarm('station_hold', node_name="StationHoldHandler")
        station_hold_goal = MoveGoal()
        station_hold_goal.move_type = 'hold'

        self.do_holding = lambda: self.client.send_goal(station_hold_goal)

    def handle(self, alarm, time_sent, parameters):
        self.do_holding()
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))
        self.station_hold_alarm.clear_alarm()

    def cancel(self, alarm, time_sent, parameters):
        # This alarm will get cleared immediatly after it gets called
        pass
