#!/usr/bin/env python
import rospy
from _template import HandlerBase


class NetworkLossHandler(HandlerBase):
    alarm_name = 'network-loss'

    def __init__(self):
        pass

    def handle(self, alarm, time_sent, parameters):
        pass

    def cancel(self, alarm, time_sent, parameters):
        pass
