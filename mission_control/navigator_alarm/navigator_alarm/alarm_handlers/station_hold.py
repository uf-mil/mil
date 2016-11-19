#!/usr/bin/env python
import rospy
import actionlib
from _template import HandlerBase

from lqrrt_ros.msg import MoveAction, MoveGoal

class StationHoldHandler(HandlerBase):
    alarm_name = 'station_hold'

    def __init__(self):
        self.station_hold_alarm = self.alarm_broadcaster.add_alarm('station_hold', node_name="StationHoldHandler")

    def handle(self, alarm, time_sent, parameters):
        # Wait 1 second then clear, ../../nodes/event_listners/station_holder.py takes over once the alarm is raised
        rospy.sleep(1)
        self.station_hold_alarm.clear_alarm()