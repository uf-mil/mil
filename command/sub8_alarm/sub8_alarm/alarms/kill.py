#!/usr/bin/env python
import rospy
from kill_handling.broadcaster import KillBroadcaster


class Handler(object):
    alarm_name = 'kill'

    def __init__(self):
        # Keep some knowledge of which thrusters we have working
        self.dropped_thrusters = []
        self.kb = KillBroadcaster(id='alarm-kill', description='Kill by alarm')

    def handle(self, time_sent, parameters):
        self.kb.send(active=True)

    def cancel(self, time_sent, parameters):
        self.kb.send(active=False)
