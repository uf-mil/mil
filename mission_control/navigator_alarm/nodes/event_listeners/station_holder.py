#!/usr/bin/env python
import rospy
import actionlib

from navigator_alarm import AlarmListener
from lqrrt_ros.msg import MoveAction, MoveGoal
from navigator_msgs.srv import WrenchSelect, WrenchSelectRequest

from mil_misc_tools.text_effects import fprint as _fprint

fprint = lambda *args, **kwargs: _fprint(title="STATION_HOLDER", time="", *args, **kwargs)

class StationHoldListener():
    def __init__(self):
        fprint("Starting station hold listener...")
        self.client = actionlib.SimpleActionClient('/move_to', MoveAction)
        fprint("Waiting for action server...")
        self.client.wait_for_server()

        self.change_wrench = rospy.ServiceProxy("/change_wrench", WrenchSelect)
        self.change_wrench.wait_for_service()

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
        self.client.wait_for_result(rospy.Duration.from_sec(2.0))
        res = self.client.get_result()
        if res is None or res.failure_reason is not '':
            fprint("ERROR STATION HOLDING", msg_color="red")
            return

        self.change_wrench("autonomous")

        fprint("Station holding", msg_color="green")

if __name__ == "__main__":
    rospy.init_node("station_holder")
    s = StationHoldListener()
    rospy.spin()
