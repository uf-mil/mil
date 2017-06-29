from __future__ import division
import rospy

from ros_alarms.msg import Alarm as AlarmMsg
from ros_alarms.srv import AlarmSet, AlarmGet, AlarmSetRequest, AlarmGetRequest

import json
import traceback


def parse_json_str(json_str):
    parameters = ''
    try:
        parameters = '' if json_str is '' else json.loads(json_str)
    except ValueError:
        # User passed in a non JSON string
        parameters = {}
        parameters['data'] = json_str
    finally:
        return parameters


def _check_for_alarm(alarm_name, nowarn=False):
    if not nowarn and rospy.has_param("/known_alarms") and \
       alarm_name not in rospy.get_param("/known_alarms"):
        msg = "'{}' is not in the list of known alarms (as defined in the /known_alarms rosparam)"
        rospy.logwarn(msg.format(alarm_name))


def _check_for_valid_name(alarm_name, nowarn=False):
    if nowarn:
        return

    assert alarm_name.isalnum() or '_' in alarm_name or '-' in alarm_name, \
        "Alarm name '{}' is not valid!".format(alarm_name)


class Alarm(object):

    @classmethod
    def blank(cls, name):
        ''' Generate a general blank alarm that is cleared with a low severity '''
        return cls(name, raised=False, severity=0)

    @classmethod
    def from_msg(cls, msg):
        ''' Generate an alarm object from an Alarm message '''
        node_name = "unknown" if msg.node_name is "" else msg.node_name
        parameters = parse_json_str(msg.parameters)

        return cls(msg.alarm_name, msg.raised, node_name,
                   msg.problem_description, parameters, msg.severity)

    def __init__(self, alarm_name, raised, node_name="unknown",
                 problem_description="", parameters={}, severity=0):
        self.alarm_name = alarm_name
        self.raised = raised
        self.node_name = node_name
        self.problem_description = problem_description
        self.parameters = parameters
        self.severity = severity

        try:
            self.stamp = rospy.Time.now()
        except rospy.ROSInitException:
            self.stamp = rospy.Time(0)

        # Callbacks to run if the alarm is cleared or raised formatted as follows:
        #   [(severity_required, cb1), (severity_required, cb2), ...]
        self.raised_cbs = []
        self.cleared_cbs = []

    def __repr__(self):
        msg = self.as_msg()
        msg.parameters = parse_json_str(msg.parameters)
        return str(msg)
    __str__ = __repr__

    def _severity_cb_check(self, severity):
        if isinstance(severity, tuple) or isinstance(severity, list):
            return severity[0] <= self.severity <= severity[1]

        # Not a tuple, just an int. The severities should match
        return self.severity == severity

    def add_callback(self, funct, call_when_raised=True, call_when_cleared=True,
                     severity_required=(0, 5)):
        ''' Deals with adding handler function callbacks
        The user can specify if the function should be run on a raise or clear
        This will call the function when added if that condition is met.

        Each callback can have a severity level associated with it such that different callbacks can
            be triggered for different levels or ranges of severity.
        '''
        err_msg = "A {} callback for the alarm: {} threw an error!\n{}"
        if call_when_raised:
            self.raised_cbs.append((severity_required, funct))
            if self.raised and self._severity_cb_check(severity_required):
                # Try to run the callback, absorbing any errors
                try:
                    funct(self)
                except Exception as e:
                    rospy.logwarn(err_msg.format('raise', self.alarm_name, traceback.format_exc()))

        if call_when_cleared:
            self.cleared_cbs.append(((0, 5), funct))
            if not self.raised and self._severity_cb_check(severity_required):
                # Try to run the callback, absorbing any errors
                try:
                    funct(self)
                except Exception as e:
                    rospy.logwarn(err_msg.format('clear', self.alarm_name, traceback.format_exc()))

    def update(self, srv):
        ''' Updates this alarm with a new AlarmSet request.
        Also will call any required callbacks.
        '''
        self.stamp = rospy.Time.now()

        node_name = "unknown" if srv.node_name is "" else srv.node_name
        parameters = parse_json_str(srv.parameters)

        # Update all possible parameters
        self.raised = srv.raised
        self.node_name = node_name
        self.problem_description = srv.problem_description
        self.parameters = parameters
        self.severity = srv.severity

        rospy.loginfo("Updating alarm: {}, {}.".format(self.alarm_name, "raised" if self.raised else "cleared"))
        # Run the callbacks for that alarm
        cb_list = self.raised_cbs if srv.raised else self.cleared_cbs
        for severity, cb in cb_list:
            # If the cb severity is not valid for this alarm's severity, skip it
            if srv.raised and not self._severity_cb_check(severity):
                continue

            # Try to run the callback, absorbing any errors
            try:
                cb(self)
            except Exception as e:
                rospy.logwarn("A callback function for the alarm: {} threw an error!".format(self.alarm_name))
                rospy.logwarn(e)

    def as_msg(self):
        ''' Get this alarm as an Alarm message '''
        alarm = AlarmMsg()
        alarm.alarm_name = self.alarm_name
        alarm.raised = self.raised
        alarm.node_name = self.node_name
        alarm.problem_description = self.problem_description
        alarm.parameters = json.dumps(self.parameters)
        alarm.severity = self.severity
        return alarm

    def as_srv_resp(self):
        ''' Get this alarm as an AlarmGet response '''
        resp = AlarmGetResponse()
        resp.header.stamp = self.stamp
        resp.alarm = self.as_msg()
        return resp


class AlarmBroadcaster(object):

    def __init__(self, name, node_name=None, nowarn=False):
        self._alarm_name = name
        _check_for_valid_name(self._alarm_name, nowarn)
        _check_for_alarm(self._alarm_name, nowarn)

        self._node_name = rospy.get_name() if node_name is None else node_name

        self._alarm_set = rospy.ServiceProxy("/alarm/set", AlarmSet)
        try:
            rospy.wait_for_service("/alarm/set", timeout=1)
        except rospy.exceptions.ROSException:
            rospy.logerr("No alarm sever found! Alarm behaviours will be unpredictable.")

        rospy.logdebug("Created alarm broadcaster for alarm {}".format(name))

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
        try:
            return self._alarm_set(self._generate_request(True, **kwargs))
        except rospy.service.ServiceException:
            rospy.logerr("No alarm sever found! Can't raise alarm.")

    def clear_alarm(self, **kwargs):
        ''' Clears this alarm '''
        try:
            return self._alarm_set(self._generate_request(False, **kwargs))
        except rospy.service.ServiceException:
            rospy.logerr("No alarm sever found! Can't clear alarm.")


class AlarmListener(object):

    def __init__(self, name, callback_funct=None, nowarn=False, **kwargs):
        self._alarm_name = name
        self._last_alarm = None
        _check_for_valid_name(self._alarm_name, nowarn)
        _check_for_alarm(self._alarm_name, nowarn)

        self._alarm_get = rospy.ServiceProxy("/alarm/get", AlarmGet)
        try:
            rospy.wait_for_service("/alarm/get", timeout=1)
        except rospy.exceptions.ROSException:
            rospy.logerr("No alarm sever found! Alarm behaviours will be unpredictable.")

        # Data used to trigger callbacks
        self._raised_cbs = []  # [(severity_for_cb1, cb1), (severity_for_cb2, cb2), ...]
        self._cleared_cbs = []
        rospy.Subscriber("/alarm/updates", AlarmMsg, self._alarm_update)

        if callback_funct is not None:
            self.add_callback(callback_funct, **kwargs)

    def is_raised(self):
        ''' Returns whether this alarm is raised or not '''
        try:
            resp = self._alarm_get(AlarmGetRequest(alarm_name=self._alarm_name))
            return resp.alarm.raised
        except rospy.service.ServiceException:
            rospy.logerr("No alarm sever found!")
            return False

    def is_cleared(self):
        ''' Returns whether this alarm is cleared or not '''
        return not self.is_raised()

    def get_alarm(self):
        ''' Returns the alarm message
        Also worth noting, the alarm this returns has it's `parameter` field
            converted to a dictionary
        '''
        try:
            resp = self._alarm_get(AlarmGetRequest(alarm_name=self._alarm_name))
        except rospy.service.ServiceException:
            rospy.logerr("No alarm sever found!")
            return None

        resp.alarm.parameters = parse_json_str(resp.alarm.parameters)
        self._last_alarm = resp.alarm
        return resp.alarm

    def _severity_cb_check(self, severity):
        # In case _last alarm hasnt been declared yet
        if self._last_alarm is None:
            return False

        if isinstance(severity, tuple) or isinstance(severity, list):
            return severity[0] <= self._last_alarm.severity <= severity[1]

        # Not a tuple or list, just an int. The severities should match
        return self._last_alarm.severity == severity

    def add_callback(self, funct, call_when_raised=True, call_when_cleared=True,
                     severity_required=(0, 5)):
        ''' Deals with adding function callbacks
        The user can specify if the function should be run on a raise or clear of this alarm.

        Each callback can have a severity level associated with it such that different callbacks can
            be triggered for different levels of severity.
        '''
        alarm = self.get_alarm()
        if call_when_raised:
            self._raised_cbs.append((severity_required, funct))
            if alarm.raised and self._severity_cb_check(severity_required):
                # Try to run the callback, absorbing any errors
                try:
                    alarm.parameters = parse_json_str(alarm.parameters)
                    funct(alarm)
                except Exception as e:
                    rospy.logwarn("A callback function for the alarm: {} threw an error!".format(self._alarm_name))
                    rospy.logwarn(e)

        if call_when_cleared:
            self._cleared_cbs.append(((0, 5), funct))  # Clear callbacks always run
            if not alarm.raised:
                # Try to run the callback, absorbing any errors
                try:
                    alarm.parameters = parse_json_str(alarm.parameters)
                    funct(alarm)
                except Exception as e:
                    rospy.logwarn("A callback function for the alarm: {} threw an error!".format(self._alarm_name))
                    rospy.logwarn(e)

    def clear_callbacks(self):
        ''' Clears all callbacks '''
        self._raised_cbs = []
        self._cleared_cbs = []

    def _alarm_update(self, alarm):
        if alarm.alarm_name != self._alarm_name:
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
                alarm.parameters = parse_json_str(alarm.parameters)
                cb(alarm)
            except Exception as e:
                rospy.logwarn("A callback function for the alarm: {} threw an error!".format(self._alarm_name))
                rospy.logwarn(e)


class HeartbeatMonitor(AlarmBroadcaster):

    def __init__(self, alarm_name, topic_name, msg_class, prd=0.2, predicate=None, nowarn=False, **kwargs):
        ''' Used to trigger an alarm if a message on the topic `topic_name` isn't published
            atleast every `prd` seconds.

        An alarm won't be triggered if no messages are initally received
        '''
        self._predicate = predicate if predicate is not None else lambda *args: True
        self._last_msg_time = None
        self._prd = rospy.Duration(prd)
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
