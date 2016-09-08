#!/usr/bin/env python
import rospy
from kill_handling.broadcaster import KillBroadcaster
from _template import HandlerBase

class Handler(HandlerBase):
    alarm_name = 'kill'

    def __init__(self):
        self.kb = KillBroadcaster(id='alarm-kill', description='Kill by alarm')
        self.alarms = {}

    def handle(self, alarm, time_sent, parameters):
        self.alarms[alarm.alarm_name] = True
        self.kb.send(active=True)

    def cancel(self, alarm, time_sent, parameters):
        self.alarms[alarm.alarm_name] = False

        # Make sure that ALL alarms that caused a kill have been cleared
        if not any(self.alarms.values()):
            self.kb.send(active=False)
