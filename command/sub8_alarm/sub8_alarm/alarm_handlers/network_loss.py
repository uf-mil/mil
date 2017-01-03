#!/usr/bin/env python
import rospy
from _template import HandlerBase
from navigator_msgs.srv import WrenchSelect

import kill_handling

class NetworkLossHandler(HandlerBase):
    alarm_name = 'network_loss'

    def __init__(self):
        pass

    def handle(self, alarm, time_sent, parameters):
        pass

    def cancel(self, alarm, time_sent, parameters):
        pass
