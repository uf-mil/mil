#!/usr/bin/env python
import rospy
from navigator_alarm import AlarmListener

rospy.init_node("alarm_test")

def kill(alarm):
    print "KILL cleared: {}".format(alarm.clear)

def rc_kill(alarm):
    print "RCKILL clear: {}".format(alarm.clear)

al = AlarmListener()
al.add_listener("kill", kill)
al.add_listener("rc_kill", rc_kill)

rospy.spin()
