#!/usr/bin/env python
import rospy
from _template import HandlerBase
from kill_handling.broadcaster import KillBroadcaster


class Handler(HandlerBase):
    alarm_name = 'kill'

    def __init__(self):
        # Keep some knowledge of which thrusters we have working
        self.kb = KillBroadcaster(id='alarm-kill', description='Kill by alarm')
        self.alarms = {}

    def handle(self, time_sent, parameters, alarm_name):
        self.alarms[alarm_name] = True
        self.kb.send(active=True)

    def cancel(self, time_sent, parameters, alarm_name):
        self.alarms[alarm_name] = False

        # Make sure that ALL alarms that caused a kill have been cleared
        if not any(self.alarms.values()):
            self.kb.send(active=False)
