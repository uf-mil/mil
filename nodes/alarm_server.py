#!/usr/bin/env python
import rospy
import json

from std_msgs.msg import Header 
from ros_alarms.msg import Alarms
from ros_alarms.msg import Alarm as AlarmMsg
from ros_alarms.srv import AlarmGet, AlarmGetResponse, AlarmSet, AlarmSetResponse


class Alarm(object):
    @classmethod
    def blank(cls, name):
        ''' Generate a general blank alarm that is cleared with a low severity '''
        return cls(name, False, severity=5)

    @classmethod
    def from_msg(cls, msg):
        ''' Generate an alarm object from an Alarm message '''
        node_name = "unknown" if msg.node_name is "" else msg.node_name
        parameters = {} if msg.parameters is '' else json.loads(msg.parameters)
        return cls(msg.alarm_name, msg.raised, node_name, msg.action_required, 
                   msg.problem_description, parameters, msg.severity)

    def __init__(self, alarm_name, raised, node_name="unknown", action_required=False,
                 problem_description="", parameters={}, severity=1):
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
        self.raised_cbs = []
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
        parameters = '' if srv.parameters is '' else json.loads(srv.parameters)

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
            if not self._severity_cb_check(severity):
                continue

            # Try to run the callback, absorbing any errors
            try:
                cb(self)
            except:
                rospy.logwarn("A callback function for the alarm: {} threw an error!".format(self.alarm_name))

    def as_msg(self):
        ''' Get this alarm as an Alarm message '''
        alarm = AlarmMsg()
        alarm.alarm_name = self.alarm_name
        alarm.raised = self.raised
        alarm.node_name = self.node_name
        alarm.action_required = self.action_required
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


class AlarmServer(object):
    def __init__(self, handler_path):
        # Maps alarm name to Alarm objects
        self.alarms = {}

        # Outside interface to the alarm system. Usually you don't want to 
        # interface with these directly.
        self.alarm_pub = rospy.Publisher("/alarm/updates", Alarms, latch=True, queue_size=100)
        rospy.Service("/alarm/set", AlarmSet, self.set_alarm)
        rospy.Service("/alarm/get", AlarmGet, self.get_alarm)
       
        self.publish_alarms()

    def set_alarm(self, srv):
        ''' Sets or updates the alarm
        Updating the alarm triggers all of the alarms callbacks
        '''
        alarm = srv.alarm
        if alarm.alarm_name in self.alarms:
            rospy.loginfo("Updating alarm: {}, {}".format(alarm.alarm_name, alarm.raised))
            self.alarms[alarm.alarm_name].update(alarm)
        else:
            rospy.loginfo("Adding alarm: {}, {}".format(alarm.alarm_name, alarm.raised))
            self.alarms[alarm.alarm_name] = Alarm.from_msg(alarm)

        self.publish_alarms()
        return True

    def get_alarm(self, srv):
        ''' Either returns the alarm request if it exists or a blank alarm '''
        rospy.loginfo("Got request for alarm: {}".format(srv.alarm_name))
        return self.alarms.get(srv.alarm_name, Alarm.blank(srv.alarm_name)).as_srv_resp()

    def publish_alarms(self):
        alarms = Alarms()
        alarms.header = Header(stamp=rospy.Time.now())
        alarms.alarms = [a.as_msg() for a in self.alarms.values()]
        self.alarm_pub.publish(alarms)

if __name__ == "__main__":
    rospy.init_node("alarm_server")
    handler_path = ""
    a = AlarmServer(handler_path)
    rospy.spin()
