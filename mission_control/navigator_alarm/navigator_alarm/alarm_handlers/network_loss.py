#!/usr/bin/env python
import rospy
from _template import HandlerBase
from navigator_msgs.srv import WrenchSelect

#from lqrrt_ros.msg import MoveAction, MoveGoal

class NetworkLossHandler(HandlerBase):
    alarm_name = 'network_loss'

    def __init__(self):
        # To kill or to station hold. That is the question
        # TODO: Check if we can station hold? Listen to /move_to/feedback and check 'tracking'.
        #self.kill = self.alarm_broadcaster.add_alarm('network_loss', node_name="NetworkLossHandler")
        #self.station_hold_alarm = self.alarm_broadcaster.add_alarm('station_hold', node_name="NetworkLossHandler")
        self.wrench_changer = rospy.ServiceProxy("/change_wrench", WrenchSelect)

    def handle(self, alarm, time_sent, parameters):
        #self.station_hold_alarm.raise_alarm()
        #self.kill.raise_alarm()
        self.wrench_changer("rc")

    def cancel(self, alarm, time_sent, parameters):
        pass
