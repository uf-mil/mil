from __future__ import division
import rospy
import rostopic

from ros_alarms.msg import Alarms
from ros_alarms.srv import AlarmSet, AlarmGet, AlarmSetRequest, AlarmGetRequest

import json


class AlarmBroadcaster(object):
    def __init__(self, name, node_name=None):
        self._alarm_name = name
        self._node_name = rospy.get_name() if node_name is None else node_name

        self._alarm_set = rospy.ServiceProxy("/alarm/set", AlarmSet)
        rospy.wait_for_service("/alarm/set")
        rospy.logdebug("Created alarm broadcaster for alarm {}".format(name))

    def _generate_request(self, raised, problem_description="", parameters={}, severity=5):
        request = AlarmSetRequest()
        request.alarm.alarm_name = self._alarm_name
        request.alarm.node_name = self._node_name

        request.alarm.raised = raised
        request.alarm.problem_description = problem_description
        request.alarm.parameters = json.dumps(parameters)
        request.alarm.severity = severity

        return request

    def raise_alarm(self, **kwargs):
        ''' Raises this alarm '''
        return self._alarm_set(self._generate_request(True, **kwargs))

    def clear_alarm(self, **kwargs):
        ''' Clears this alarm '''
        return self._alarm_set(self._generate_request(False, **kwargs))


class AlarmListener(object):
    def __init__(self, name, callback_funct=None, **kwargs):
        self._alarm_name = name

        self._alarm_get = rospy.ServiceProxy("/alarm/get", AlarmGet)
        rospy.wait_for_service("/alarm/get")
        
        # Data used to trigger callbacks
        self._last_alarm = None
        self._raised_cbs = []  # [(severity_for_cb1, cb1), (severity_for_cb2, cb2), ...]
        self._cleared_cbs = []
        rospy.Subscriber("/alarm/updates", Alarms, self._alarm_update)

        if callback_funct is not None:
            self.add_callback(callback_funct, **kwargs)
        
    def is_raised(self):
        ''' Returns whether this alarm is raised or not '''
        resp = self._alarm_get(AlarmGetRequest(alarm_name=self._alarm_name))
        return resp.alarm.raised

    def is_cleared(self):
        ''' Returns whether this alarm is cleared or not '''
        return not self.is_raised()

    def get_alarm(self):
        ''' Returns the alarm message 
        Also worth noting, the alarm this returns has it's `parameter` field 
            converted to a dictionary
        '''
        resp = self._alarm_get(AlarmGetRequest(alarm_name=self._alarm_name))
        params = resp.alarm.parameters
        resp.alarm.parameters = params if params == '' else json.loads(params)
        return resp.alarm 

    def _severity_cb_check(self, severity):
        if isinstance(severity, tuple):
            # If the severity is a tuple, it should be interpreted as a range
            if severity[1] == -1:
                # (X, -1)  Triggers for any alarms less severe then X
                return severity[0] < self._last_alarm.severity 

            # (-1 , X) or (Y, X)  Trigger for any alarms less or equally severe to Y but more severe then X
            return severity[0] <= self._last_alarm.severity < severity[1] 
        
        # Not a tuple, just an int. -1 for any severity, otherwise the severities much match
        return severity == -1 or self._last_alarm.severity == severity

    def add_callback(self, funct, call_when_raised=True, call_when_cleared=True,
                     severity_required=-1):
        ''' Deals with adding function callbacks
        The user can specify if the function should be run on a raise or clear of this alarm.

        Each callback can have a severity level associated with it such that different callbacks can 
            be triggered for different levels of severity.
        '''
        if call_when_raised:
            self._raised_cbs.append((severity_required, funct))

        if call_when_cleared:
            self._cleared_cbs.append((-1, funct))  # Clear callbacks always run

    def clear_callbacks(self):
        ''' Clears all callbacks '''
        self._raised_cbs = []
        self._cleared_cbs = []

    def _alarm_update(self, msg):
        alarm = [a for a in msg.alarms if a.alarm_name == self._alarm_name][0]

        # No change from the last update of this alarm
        if alarm == self._last_alarm:
            return
        self._last_alarm = alarm

        # Run the callbacks if severity conditions are met
        cb_list = self._raised_cbs if alarm.raised else self._cleared_cbs
        for severity, cb in cb_list:
            # If the cb severity is not valid for this alarm's severity, skip it
            if not self._severity_cb_check(severity):
                continue

            # Try to run the callback, absorbing any errors
            try:
                cb(self)
            except Exception as e:
                rospy.logwarn("A callback function for the alarm: {} threw an error!".format(self.alarm_name))
                rospy.logwarn(e)

class HeartbeatMonitor(AlarmBroadcaster):
    def __init__(self, alarm_name, topic_name, prd=0.2, predicate=None):
        ''' Used to trigger an alarm if a message on the topic `topic_name` isn't published
            atleast every `prd` seconds.

        An alarm won't be triggered if no messages are initally received
        '''
        self._predicate = predicate if predicate is not None else lambda *args: True
        self._last_msg_time = None
        self._prd = rospy.Duration(prd)
        self._killed = False
        
        super(HeartbeatMonitor, self).__init__(alarm_name)
        msg_class, _, _ = rostopic.get_topic_class(topic_name)
        rospy.Subscriber(topic_name, msg_class, self._got_msg)

        rospy.Timer(rospy.Duration(prd / 2), self._check_for_message)

    def _got_msg(self, msg):
        # If the predicate passes, store the message time
        if self._predicate(msg):
            self._last_msg_time = rospy.Time.now()
            
            # If it's killed, clear the kill
            if self._killed:
                self.clear_alarm()
                self._killed = False

    def _check_for_message(self, *args):
        if self._last_msg_time is None:
            return

        if rospy.Time.now() - self._last_msg_time > self._prd and not self._killed:
            self.raise_alarm()
            self._killed = True

