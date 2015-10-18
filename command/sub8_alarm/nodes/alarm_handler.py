#!/usr/bin/env python
import rospy
import json
from sub8_msgs.msg import Alarm
from std_msgs.msg import Header
from sub8_alarm import alarms
import string


class AlarmHandler(object):
    def __init__(self):
        '''Alarm Handler
            Listen for alarms, call scenarios
        TODO:
            - Add alarm queue
            - Handle alarms in sequence (Don't get stuck in the callback)
            - bag (if set) EVERY single alarm received
        '''
        rospy.init_node('alarm_handler')
        # Queue size is large because you bet your ass we are addressing every alarm
        self.alarm_sub = rospy.Subscriber('/alarm', Alarm, self.alarm_callback, queue_size=100)

        self.scenarios = {}

        # Go through everything in the sub8_alarm.alarms package
        for candidate_alarm_name in dir(alarms):
            # Discard __* nonsense
            if not candidate_alarm_name.startswith('_'):
                # Verify that it is actually an alarm handler
                CandidateAlarm = getattr(alarms, candidate_alarm_name)
                if hasattr(CandidateAlarm, 'handle'):
                    self.scenarios[CandidateAlarm.alarm_name] = CandidateAlarm()


    def alarm_callback(self, alarm):
        time = alarm.header.stamp
        if alarm.action_required:
            rospy.logwarn(
                "{}: {}, severity {}, handling NOW".format(
                    alarm.node_name, alarm.alarm_name, alarm.severity
                )
            )

        rospy.logwarn(
            "{} raised alarm of type {} of severity {} at {}".format(
                alarm.node_name, alarm.alarm_name, alarm.severity, time
            )
        )

        scenario = self.scenarios.get(alarm.alarm_name)

        # Decode JSON
        parameters = json.loads(alarm.parameters)
        scenario.handle(time, parameters)


if __name__ == '__main__':
    alarm_handler = AlarmHandler()
    rospy.spin()
