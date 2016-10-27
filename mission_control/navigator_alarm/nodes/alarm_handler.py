#!/usr/bin/env python
import rospy
import json
from navigator_msgs.msg import Alarm
from navigator_alarm import alarm_handlers, meta_alarms, HandlerBase
import datetime
import inspect
import sys


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
        self.alarm_republish_timer = rospy.Timer(rospy.Duration(0.05), self.republish_alarms)

        self.scenarios = {}

        # Go through everything in the navigator_alarm.alarm_handlers package
        for candidate_alarm_name in dir(alarm_handlers):
            # Discard __* nonsense
            if not candidate_alarm_name.startswith('_'):
                # Verify that it is actually an alarm handler by checking if the class inherits from `HandlerBase`
                handlers = inspect.getmembers(getattr(alarm_handlers, candidate_alarm_name), inspect.isclass)
                handlers = [handler for handler in handlers if handler[0] != "HandlerBase" and \
                                                               issubclass(handler[1], HandlerBase)]
                for handler_name, handler_class in handlers:
                    rospy.loginfo("Registered scenario with name '{}'".format(handler_class.alarm_name))
                    self.scenarios[handler_class.alarm_name] = handler_class()

    def republish_alarms(self, *args):
        alarms = self.alarms
        for alarm_name, alarm in alarms.items():
            self.alarm_pub.publish(alarm)

            if alarm.clear:
                del self.alarms[alarm_name]

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
                scenario.cancel(alarm, time, parameters)
            else:
                scenario.handle(alarm, time, parameters)

        # Handle meta-alarms (See Sub8 wiki)
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
