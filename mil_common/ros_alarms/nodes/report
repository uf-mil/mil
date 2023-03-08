#!/usr/bin/env python3
import argparse

import rospy
from ros_alarms import AlarmListener

parser = argparse.ArgumentParser(description="Reports the status of an alarm")
parser.add_argument("alarm_name", help="the name of the alarm to report on")
args = parser.parse_args()


def print_alarm(alarm):
    rospy.loginfo(
        "Alarm: {} ({}) ({})".format(
            alarm.alarm_name, "raised" if alarm.raised else "clear", alarm.severity
        )
    )
    for name, value in alarm.parameters.items():
        rospy.loginfo(f"\t{name} = {value}")


rospy.init_node("alarm_report", anonymous=True)
al = AlarmListener(args.alarm_name)
al.wait_for_server()
alarm = al.get_alarm()
print_alarm(alarm)
rospy.sleep(0.1)
