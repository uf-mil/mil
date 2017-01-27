#!/usr/bin/env python
import rospy
from ros_alarms import HandlerBase 

from std_msgs.msg import Header 
from ros_alarms.msg import Alarms
from ros_alarms.msg import Alarm as AlarmMsg
from ros_alarms.srv import AlarmGet, AlarmGetResponse, AlarmSet, AlarmSetResponse

import json
import inspect


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


class Alarm(object):
    @classmethod
    def blank(cls, name):
        ''' Generate a general blank alarm that is cleared with a low severity '''
        return cls(name, False, severity=0)

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

        self.stamp = rospy.Time.now()

        # Callbacks to run if the alarm is cleared or raised formatted as follows:
        #   [(severity_required, cb1), (severity_required, cb2), ...]
        self.raised_cbs = []
        self.cleared_cbs = []

    def __repr__(self):
        msg = self.as_msg()
        msg.parameters = self.parse_json_str(msg.parameters)
        return msg
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
        if call_when_raised:
            self.raised_cbs.append((severity_required, funct))
            if self.raised and self._severity_cb_check(severity_required):
                # Try to run the callback, absorbing any errors
                try:
                    funct(self)
                except Exception as e:
                    rospy.logwarn("A callback function for the alarm: {} threw an error!".format(self.alarm_name))
                    rospy.logwarn(e)

        if call_when_cleared:
            self.cleared_cbs.append(((0, 5), funct))
            if not self.raised and self._severity_cb_check(severity_required):
                # Try to run the callback, absorbing any errors
                try:
                    funct(self)
                except Exception as e:
                    rospy.logwarn("A callback function for the alarm: {} threw an error!".format(self.alarm_name))
                    rospy.logwarn(e)

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


class AlarmServer(object):
    def __init__(self):
        # Maps alarm name to Alarm objects
        self.alarms = {}
        # Maps meta alarm names to predicate Handler functions
        self.meta_alarms = {}

        # Outside interface to the alarm system. Usually you don't want to 
        # interface with these directly.
        self._alarm_pub = rospy.Publisher("/alarm/updates", AlarmMsg, latch=True, queue_size=100)
        rospy.Service("/alarm/set", AlarmSet, self.set_alarm)
        rospy.Service("/alarm/get", AlarmGet, self.get_alarm)
        
        msg = "Expecting at most the following alarms: {}"
        rospy.loginfo(msg.format(rospy.get_param("/known_alarms", [])))

        self._create_meta_alarms()
        self._create_alarm_handlers()

    def set_alarm(self, srv):
        ''' Sets or updates the alarm
        Updating the alarm triggers all of the alarms callbacks
        '''
        alarm = srv.alarm

        # If the alarm name is `all`, clear all alarms
        if alarm.alarm_name == "all":
            rospy.loginfo("Clearing all alarms.")
            for alarm in self.alarms.values():
                # Don't want to clear meta alarms until the end
                if alarm in self.meta_alarms:
                    continue
                cleared_alarm = alarm.as_msg()
                cleared_alarm.raised = False
                alarm.update(cleared_alarm)
                self._alarm_pub.publish(alarm.as_msg())

            for _alarm in self.meta_alarms.keys():
                alarm = self.alarms[_alarm]
                cleared_alarm = alarm.as_msg()
                cleared_alarm.raised = False
                alarm.update(cleared_alarm)
                self._alarm_pub.publish(alarm.as_msg())
                
            return True

        if alarm.alarm_name in self.alarms:
            self.alarms[alarm.alarm_name].update(alarm)
        else:
            self.alarms[alarm.alarm_name] = Alarm.from_msg(alarm)
        
        self._alarm_pub.publish(alarm)
        return True

    def get_alarm(self, srv):
        ''' Either returns the alarm request if it exists or a blank alarm '''
        rospy.logdebug("Got request for alarm: {}".format(srv.alarm_name))
        return self.alarms.get(srv.alarm_name, Alarm.blank(srv.alarm_name)).as_srv_resp()

    def _handle_meta_alarm(self, meta_alarm, sub_alarms):
        alarms = {name: alarm for name, alarm in self.alarms.items() if name in sub_alarms}
        meta = self.alarms[meta_alarm]

        # Check the predicate, this should return the new `raised` status of the meta alarm
        raised_status = self.meta_alarms[meta_alarm](meta, alarms)
        if raised_status != meta.raised:
            temp = meta.as_msg()
            temp.raised = raised_status
            meta.update(temp)
            self._alarm_pub.publish(meta.as_msg())

    def _create_alarm_handlers(self):
        # If the param exists, load it here
        handler_module = rospy.get_param("~handler_module", None)
        if handler_module is None:
            return

        # Import the module where the handlers are stored
        alarm_handlers = __import__(handler_module, fromlist=[""])
        for handler in [cls for name, cls in inspect.getmembers(alarm_handlers) \
                        if inspect.isclass(cls) and issubclass(cls, HandlerBase) and \
                        hasattr(cls, "alarm_name") and name is not "HandlerBase"]:

            # Have to instantiate so the class exists exists
            h = handler() 

            alarm_name = handler.alarm_name
            severity = handler.severity_required

            if alarm_name not in self.alarms:
                self.alarms[alarm_name] = Alarm.blank(alarm_name)
            self.alarms[alarm_name].raised = h.initally_raised

            # If a handler exists for a meta alarm, we need to save the predicate
            if alarm_name in self.meta_alarms:
                self.meta_alarms[alarm_name] = h.meta_predicate

            # Register the raised or cleared functions
            self.alarms[alarm_name].add_callback(h.raised, call_when_cleared=False, 
                                                 severity_required=severity)

            self.alarms[alarm_name].add_callback(h.cleared, call_when_raised=False, 
                                                 severity_required=severity)

            rospy.loginfo("Loaded handler: {}".format(h.alarm_name))

    def _create_meta_alarms(self, namespace="meta_alarms/"):
        meta_alarms_dict = rospy.get_param(namespace, {})
        for meta, alarms in meta_alarms_dict.iteritems():
            # Add the meta alarm
            if meta not in self.alarms:
                self.alarms[meta] = Alarm.blank(meta)

            default = lambda meta, alarms: any(alarms.items())
            self.meta_alarms[meta] = default

            cb = lambda alarm, meta_name=meta, sub_alarms=alarms: self._handle_meta_alarm(meta_name, sub_alarms)
            for alarm in alarms:
                if alarm not in self.alarms:
                    self.alarms[alarm] = Alarm.blank(alarm)

                self.alarms[alarm].add_callback(cb)


if __name__ == "__main__":
    rospy.init_node("alarm_server")
    a = AlarmServer()
    rospy.spin()
