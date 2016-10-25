#!/usr/bin/env python
import rospy
from kill_handling.broadcaster import KillBroadcaster
from _template import HandlerBase

class KillHandler(HandlerBase):
    alarm_name = 'kill'

    # Multiple functions listen to the kill alarm so no need to do anything here.
    # I'm keeping this here just incase we want to add some kill functionality.

    def __init__(self):
        pass

    def handle(self, alarm, time_sent, parameters):
        pass

    def cancel(self, alarm, time_sent, parameters):
        pass
