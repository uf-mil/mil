#!/usr/bin/env python
import rospy
import json
from sub8_msgs.msg import Alarm
from sub8_alarm import alarms, meta_alarms
import datetime


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
        self.alarm_sub = rospy.Subscriber('/alarm_raise', Alarm, self.alarm_callback, queue_size=100)
        self.alarm_pub = rospy.Publisher('/alarm', Alarm, queue_size=100)
        self.alarms = {}
        self.alarm_republish_timer = rospy.Timer(rospy.Duration(0.1), self.republish_alarms)

        self.scenarios = {}

        # Go through everything in the sub8_alarm.alarms package
        for candidate_alarm_name in dir(alarms):
            # Discard __* nonsense
            if not candidate_alarm_name.startswith('_'):
                # Verify that it is actually an alarm handler
                CandidateScenarioModule = getattr(alarms, candidate_alarm_name)
                if hasattr(CandidateScenarioModule, 'Handler'):
                    rospy.loginfo("Registered scenario with name {}".format(CandidateScenarioModule.Handler.alarm_name))
                    self.scenarios[CandidateScenarioModule.Handler.alarm_name] = CandidateScenarioModule.Handler()

    def republish_alarms(self, *args):
        for alarm_name, alarm in self.alarms.items():
            self.alarm_pub.publish(alarm)

    def alarm_callback(self, alarm):

        # -- > We don't have to remove cleared alarms

        # if alarm.clear:
        #     rospy.logwarn("Clearing {}".format(alarm.alarm_name))
        #     if alarm.alarm_name in self.alarms.keys():
        #         self.alarms.pop(alarm.alarm_name)
        #     return

        self.alarms[alarm.alarm_name] = alarm

        time = alarm.header.stamp
        if alarm.action_required:
            rospy.logwarn(
                "{}: {}, severity {}, handling NOW".format(
                    alarm.node_name, alarm.alarm_name, alarm.severity
                )
            )

        alarm_epoch = alarm.header.stamp.to_time()
        actual_time = datetime.datetime.fromtimestamp(alarm_epoch).strftime('%I:%M:%S.%f')
        if alarm.clear:
            rospy.logwarn(
                "{} cleared alarm of type {} at {}".format(
                    alarm.node_name, alarm.alarm_name, actual_time
                )
            )
        else:
            rospy.logwarn(
                "{} raised alarm of type {} of severity {} at {}".format(
                    alarm.node_name, alarm.alarm_name, alarm.severity, actual_time
                )
            )

        scenario = self.scenarios.get(alarm.alarm_name)

        # Decode JSON
        if len(alarm.parameters) == 0:
            parameters = {}
        else:
            parameters = json.loads(alarm.parameters)
        if scenario is not None:
            if alarm.clear:
                scenario.cancel(time, parameters)
            else:
                scenario.handle(time, parameters)


        # Handle meta-alarms (See wiki)
        meta_alarm = meta_alarms.get(alarm.alarm_name, None)
        if meta_alarm is not None:
            self.alarms[meta_alarm] = alarm
            scenario = self.scenarios.get(meta_alarm, None)
            if scenario is not None:
                if alarm.clear:
                    scenario.cancel(time, parameters, alarm.alarm_name)
                else:
                    scenario.handle(time, parameters, alarm.alarm_name)



if __name__ == '__main__':
    alarm_handler = AlarmHandler()
    rospy.spin()
