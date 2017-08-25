#!/usr/bin/env python
import rospy
from ros_alarms import HandlerBase, Alarm

class Kill(HandlerBase):
    alarm_name = 'kill'

    def __init__(self):
        self._killed = False
        self.initial_alarm = Alarm(self.alarm_name, True,
                                   node_name='alarm_server',
                                   problem_description='Initial kill')

    def raised(self, alarm):
        self._killed = True

    def cleared(self, alarm):
        self._killed = False

    def meta_predicate(self, meta_alarm, alarms):
        if self._killed:  # Stay killed until manually cleared
            return True

        ignore = []

        # Don't kill on low battery, only on critical
        if alarms['battery-voltage'].raised and alarms['battery-voltage'].severity < 2:
            ignore.append('battery-voltage')

        # Raised if any alarms besides the two above are raised
        return any([alarm.raised for name, alarm in alarms.items()
                    if name not in ignore])
