#!/usr/bin/env python
import rospy
import actionlib

from navigator_alarm import AlarmListener
from lqrrt_ros.msg import MoveAction, MoveGoal

from navigator_tools import fprint as _fprint

fprint = lambda *args, **kwargs: _fprint(title="STATION_HOLDER", time="", *args, **kwargs)

class StationHoldListener():
    def __init__(self):
        fprint("Starting station hold listener...")
        self.client = actionlib.SimpleActionClient('/move_to', MoveAction)
        fprint("Waiting for action server...")
        self.client.wait_for_server()

        fprint("Creating alarm listener...")
        self.station_hold_alarm = AlarmListener(alarm_name='station_hold', callback_funct=self.handle)

        fprint("Ready to go!", msg_color="green")

    def handle(self, alarm):
        if alarm.clear:
            # Not of intrest since it's being cleared
            return

        station_hold_goal = MoveGoal()
        station_hold_goal.move_type = 'hold'

        self.client.send_goal(station_hold_goal)
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))

        fprint("Station holding", msg_color="green")

if __name__ == "__main__":
    rospy.init_node("station_holder")
    s = StationHoldListener()
    rospy.spin()