#!/usr/bin/env python3
from .navigator import NaviGatorMission


class Killed(NaviGatorMission):
    """
    Run when NaviGatorMission is killed. Exists mostly to
    cancel the current mission on kill and print
    this to the GUI.
    """

    def run(self, parameters):
        alarm = self.kill_alarm
        if alarm.node_name != "":
            self.send_feedback(f"Killed by {alarm.node_name}")
        if alarm.problem_description == "":
            return "Killed"
        else:
            return f"Killed: {alarm.problem_description}"
