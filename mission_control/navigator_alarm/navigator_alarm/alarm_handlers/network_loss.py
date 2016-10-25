#!/usr/bin/env python
import rospy
from _template import HandlerBase

from lqrrt_ros.msg import MoveAction, MoveGoal

class NetworkLossHandler(HandlerBase):
    alarm_name = 'network_loss'

    def __init__(self):
        self.station_hold_alarm = self.alarm_broadcaster.add_alarm('station_hold', node_name="NetworkLossHandler")

    def handle(self, alarm, time_sent, parameters):
        self.station_hold_alarm.raise_alarm()

    def cancel(self, alarm, time_sent, parameters):
        pass
