from __future__ import division
import txros
from twisted.internet import defer
import rospy  # TEMP

from ros_alarms.msg import Alarm
from ros_alarms.srv import AlarmSet, AlarmGet, AlarmSetRequest, AlarmGetRequest

import json


@txros.util.cancellableInlineCallbacks
def _check_for_alarm(nh, alarm_name, nowarn=False):
    if not nowarn and (yield nh.has_param("/known_alarms")) and \
       alarm_name not in (yield nh.get_param("/known_alarms")):
        msg = "'{}' is not in the list of known alarms (as defined in the /known_alarms rosparam)"
        print msg.format(alarm_name)

class TxAlarmBroadcaster(object):
    @classmethod
    @txros.util.cancellableInlineCallbacks
    def init(cls, nh, name, node_name=None, nowarn=False):
        yield _check_for_alarm(nh, name, nowarn)
        
        node_name = nh.get_name() if node_name is None else node_name

        defer.returnValue(cls(nh, name, node_name))

    def __init__(self, nh, name, node_name):
        ''' Don't invoke this function directly, use the `init` function above '''
        self._nh = nh
        self._alarm_name = name

        self._node_name = node_name
        self._alarm_set = self._nh.get_service_client("/alarm/set", AlarmSet)

        print "Created alarm broadcaster for alarm {}".format(name)

    def _generate_request(self, raised, problem_description="", parameters={}, severity=0):
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


class TxAlarmListener(object):
    @classmethod
    @txros.util.cancellableInlineCallbacks
    def init(cls, nh, name, callback_funct=None, nowarn=False, **kwargs):
        yield _check_for_alarm(nh, name, nowarn)

        alarm_client = nh.get_service_client("/alarm/get", AlarmGet)
        try:
            yield txros.util.wrap_timeout(alarm_client.wait_for_service(), 1)
        except txros.util.TimeoutError:
            print "No alarm sever found! Alarm behaviours will be unpredictable."

        defer.returnValue(cls(nh, name, alarm_client, callback_funct))

    def __init__(self, nh, name, alarm_client, callback_funct):
        ''' Don't invoke this function directly, use the `init` function above '''
        self.nh = nh
        self._alarm_name = name
        self._alarm_get = alarm_client
        
        # Data used to trigger callbacks
        self._raised_cbs = []  # [(severity_for_cb1, cb1), (severity_for_cb2, cb2), ...]
        self._cleared_cbs = []
        self.update_sub = self.nh.subscribe("/alarm/updates", Alarm, self._alarm_update)

        if callback_funct is not None:
            self.add_callback(callback_funct, **kwargs)
     
    @txros.util.cancellableInlineCallbacks
    def is_raised(self):
        ''' Returns whether this alarm is raised or not '''
        resp = yield self._alarm_get(AlarmGetRequest(alarm_name=self._alarm_name))
        defer.returnValue(resp.alarm.raised)

    @txros.util.cancellableInlineCallbacks
    def is_cleared(self):
        ''' Returns whether this alarm is raised or not '''
        resp = yield self._alarm_get(AlarmGetRequest(alarm_name=self._alarm_name))
        defer.returnValue(not resp.alarm.raised)

    @txros.util.cancellableInlineCallbacks
    def get_alarm(self):
        ''' Returns the alarm message 
        Also worth noting, the alarm this returns has it's `parameter` field 
            converted to a dictionary
        '''
        resp = yield self._alarm_get(AlarmGetRequest(alarm_name=self._alarm_name))

        params = resp.alarm.parameters
        resp.alarm.parameters = params if params == '' else json.loads(params)
        defer.returnValue(resp.alarm)

    def _severity_cb_check(self, severity):
        if isinstance(severity, tuple) or isinstance(severity, list):
            return severity[0] <= self._last_alarm.severity <= severity[1]

        # Not a tuple, just an int. The severities should match
        return self._last_alarm.severity == severity

    def add_callback(self, funct, call_when_raised=True, call_when_cleared=True,
                     severity_required=(0, 5)):
        ''' Deals with adding function callbacks
        The user can specify if the function should be run on a raise or clear of this alarm.

        Each callback can have a severity level associated with it such that different callbacks can 
            be triggered for different levels of severity.
        '''
        if call_when_raised:
            self._raised_cbs.append((severity_required, funct))

        if call_when_cleared:
            self._cleared_cbs.append(((0, 5), funct))  # Clear callbacks always run

    def clear_callbacks(self):
        ''' Clears all callbacks '''
        self._raised_cbs = []
        self._cleared_cbs = []
    
    @txros.util.cancellableInlineCallbacks
    def _alarm_update(self, alarm):
        if alarm.alarm_name == self._alarm_name:
            self._last_alarm = alarm

            # Run the callbacks if severity conditions are met
            cb_list = self._raised_cbs if alarm.raised else self._cleared_cbs
            for severity, cb in cb_list:
                # If the cb severity is not valid for this alarm's severity, skip it
                if not self._severity_cb_check(severity):
                    continue

                # Try to run the callback, absorbing any errors
                try:
                    yield cb(self.nh, alarm)
                except Exception as e:
                    print "A callback function for the alarm: {} threw an error!".format(self._alarm_name)
                    print e


class TxHeartbeatMonitor(TxAlarmBroadcaster):
    @txros.util.cancellableInlineCallbacks
    def _init(self, alarm_name, topic_name, msg_type, prd=0.2, predicate=None, nowarn=False, **kwargs):
        ''' Used to trigger an alarm if a message on the topic `topic_name` isn't published
            atleast every `prd` seconds.

        An alarm won't be triggered if no messages are initally received
        '''
        raise NotImplementedError
        self._predicate = predicate if predicate is not None else lambda *args: True
        self._last_msg_time = None
        self._prd = txros.Genpy.Duration(prd)
        self._dropped = False

        super(HeartbeatMonitor, self).__init__(alarm_name, nowarn=nowarn, **kwargs)

        rospy.Subscriber(topic_name, msg_class, self._got_msg)

        rospy.Timer(rospy.Duration(prd / 2), self._check_for_message)

    def _got_msg(self, msg):
        # If the predicate passes, store the message time
        if self._predicate(msg):
            self._last_msg_time = rospy.Time.now()
            
            # If it's dropped, clear the alarm and reset the dropped status
            if self._dropped:
                self.clear_alarm()
                self._dropped = False

    def _check_for_message(self, *args):
        if self._last_msg_time is None:
            return

        if rospy.Time.now() - self._last_msg_time > self._prd and not self._dropped:
            self.raise_alarm()
            self._dropped = True

