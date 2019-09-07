#!/usr/bin/env python
from navigator import Navigator


class Killed(Navigator):
    '''
    Run when Navigator is killed. Exsists mostly to
    cancel the current mission on kill and print
    this to the GUI.
    '''

    def run(self, parameters):
        alarm = self.kill_alarm
        if alarm.node_name != '':
            self.send_feedback('Killed by {}'.format(alarm.node_name))
        if alarm.problem_description == '':
            return 'Killed'
        else:
            return 'Killed: {}'.format(alarm.problem_description)
