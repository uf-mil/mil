#!/usr/bin/env python
import rospy
import json

from ros_alarms.srv import AlarmGet, AlarmGetResponse, AlarmSet, AlarmSetResponse

class Alarm(object):
    @classmethod
    def blank(cls, name):
        ''' Generate a general blank alarm that is cleared with a low severity '''
        return cls(name, False, severity=5)

    @classmethod
    def from_srv(cls, srv):
        ''' Generate an alarm object from an AlarmSet request '''
        node_name = "unknown" if srv.node_name is "" else srv.node_name
        parameters = json.loads(srv.parameters)

        return cls(srv.alarm_name, srv.raised, node_name, srv.action_required, 
                   srv.problem_description, parameters, srv.severity)

    def __init__(self, alarm_name, raised, node_name="unknown", action_required=False,
                 problem_desription="", parameters={}, severity=1):
        self.alarm_name = alarm_name
        self.raised = raised
        self.node_name = node_name
        self.action_required = action_required
        self.problem_description = problem_description
        self.parameters = parameters
        self.severity = severity

        self.stamp = rospy.Time.now()

        # Callbacks to run if the alarm is cleared or raised formatted as follows:
        #   [(severity_required, cb1), (severity_required, cb2), ...]
        self.rasied_cbs = []
        self.cleared_cbs = []
    
    def _severity_cb_check(self, severity):
        return severity == -1 or self.severity == severity

    def add_callback(self, funct, call_when_raised=True, call_when_cleared=True, severity_required=-1):
        ''' Deals with adding handler function callbacks
        The user can specify if the function should be run on a raise or clear
        This will call the function when added if that condition is met.

        Each callback can have a severity level associated with it such that different callbacks can 
            be triggered for different levels of severity.
        '''
        if call_when_raised:
            self.raised_cbs.appened((severity_required, funct))
            if self.raised and self._severity_cb_check(severity_required):
                funct(self)

        if call_when_cleared:
            self.cleared_cbs.append((severity_required, funct))
            if not self.raised and self._severity_cb_check(severity_required):
                funct(self)

    def update(self, srv):
        ''' Updates this alarm with a new AlarmSet request. 
        Also will call any required callbacks. 
        '''
        self.stamp = rospy.Time.now()

        node_name = "unknown" if srv.node_name is "" else srv.node_name
        parameters = json.loads(srv.parameters)

        # Update all possible parameters
        self.raised = srv.raised
        self.node_name = node_name
        self.action_required = srv.action_required
        self.problem_description = srv.problem_description
        self.parameters = parameters
        self.severity = srv.severity
        
        # Run the callbacks for that alarm
        cb_list = self.raised_cbs if srv.raised else self.cleared_cbs
        for severity, cb in cb_list:
            # If the cb severity is not valid for this alarm's severity, skip it
            if not self._severity_cb_check(severity_required):
                continue

            # Try to run the callback, absorbing any errors
            try:
                cb(self)
            except:
                rospy.logwarn("A callback function for the alarm: {} threw an error!".format(self.alarm_name))

    def as_srv(self):
        ''' Get this alarm as an AlarmGet response '''
        resp = AlarmGetResponse()
        resp.header.stamp = self.stamp
        resp.alarm_name = self.alarm_name
        resp.raised = self.raised
        resp.node_name = self.node_name
        resp.action_required = self.action_required
        resp.problem_description = problem_description
        resp.parameters = json.dumps(self.parameters)
        resp.severity = severity
        return resp


class AlarmServer(object):
    def __init__(self):
        # Maps alarm name to Alarm objects
        self.alarms = {}

        # Outside interface to the alarm system. Usually you don't want to 
        # interface with these directly.
        rospy.Service("/alarm/set", AlarmSet, self.set_alarm)
        rospy.Service("/alarm/get", AlarmGet, self.get_alarm)

    def set_alarm(self, srv):
        ''' Sets or updates the alarm
        Updating the alarm triggers all of the alarms callbacks
        '''
        if srv.alarm_name in self.alarms:
            self.alarms[srv.alarm_name].update(srv)
        else:
            self.alarms[srv.alarm_name] = Alarm.from_srv(srv)

    def get_alarm(self, srv):
        ''' Either returns the alarm request if it exists or a blank alarm '''
        return self.alarms.get(srv.alarm_name, Alarm.clear(srv.alarm_name))

if __name__ == "__main__":
    rospy.init_node("alarm_server")
    AlarmServer()
    rospy.spin()
