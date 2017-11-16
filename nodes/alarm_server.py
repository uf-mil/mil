#!/usr/bin/env python
import rospy
from ros_alarms import HandlerBase

from ros_alarms.msg import Alarm as AlarmMsg
from ros_alarms.srv import AlarmGet, AlarmSet
from ros_alarms import Alarm

import inspect


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
                self._alarm_pub.publish(alarm.as_msg())
                alarm.update(cleared_alarm)

            for _alarm in self.meta_alarms.keys():
                alarm = self.alarms[_alarm]
                cleared_alarm = alarm.as_msg()
                cleared_alarm.raised = False
                self._alarm_pub.publish(alarm.as_msg())
                alarm.update(cleared_alarm)

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

    def make_tagged_alarm(self, name):
        '''
        Makes a blank alarm with the node_name of the alarm_server so that users know it is the
        initial state
        '''
        alarm = Alarm.blank(name)
        alarm.node_name = 'alarm_server'
        return alarm

    def _handle_meta_alarm(self, meta_alarm, sub_alarms):
        '''
        Calls the meta_predicate callback for an alarm handler when one of its metal alarms has changed.
        Then, updates the status of the parent alarm, if nessesary.
        '''
        alarms = {name: alarm for name, alarm in self.alarms.items() if name in sub_alarms}
        meta = self.alarms[meta_alarm]

        # Check the predicate, this should return either an alarm object or a boolean for if should be raised
        alarm = self.meta_alarms[meta_alarm](meta, alarms)

        # If it an alarm instance send it out as is
        if isinstance(alarm, Alarm):
            alarm.alarm_name = meta_alarm  # Ensure alarm name is correct
            meta.update(alarm)
            self._alarm_pub.publish(alarm.as_msg())
            return

        # Otherwise it should be a boolean
        if type(alarm) != bool:
            rospy.logwarn('Meta alarm callback for {} failed to return an Alarm or boolean'.format(meta_alarm))
            return

        # If it is a boolean, only update if it changes the raised status
        raised_status = alarm
        if raised_status != meta.raised:
            temp = meta.as_msg()
            temp.raised = bool(raised_status)
            if temp.raised: # If it is raised, set problem description
                temp.problem_description = 'Raised by meta alarm'
            meta.update(temp)
            self._alarm_pub.publish(meta.as_msg())

    def _create_alarm_handlers(self):
        '''
        Alarm handlers are classes imported by the alarm server and run code upon a change of state
        of their respective alarms.

        Handlers should be in a python module (directory with an __init__.py) and in the python path.
        They will be loaded from the module specified with the ~handler_module param to the alarm server.
        '''

        # If the param exists, load it here
        handler_module = rospy.get_param("~handler_module", None)
        if handler_module is None:
            return

        # Import the module where the handlers are stored
        alarm_handlers = __import__(handler_module, fromlist=[""])
        for handler in [cls for name, cls in inspect.getmembers(alarm_handlers)
                        if inspect.isclass(cls) and issubclass(cls, HandlerBase) and
                        hasattr(cls, "alarm_name") and name is not "HandlerBase"]:

            # Have to instantiate so the class exists exists
            h = handler()

            alarm_name = handler.alarm_name
            severity = handler.severity_required

            # Set initial state if necessary (could have already been added while creating metas)
            if hasattr(h, 'initial_alarm'):
                self.alarms[alarm_name] = h.initial_alarm  # Update even if already added to server
            elif alarm_name not in self.alarms:  # Add default initial if not there already
                self.alarms[alarm_name] = self.make_tagged_alarm(alarm_name)
            else:
                pass

            # If a handler exists for a meta alarm, we need to save the predicate
            if alarm_name in self.meta_alarms:
                self.meta_alarms[alarm_name] = h.meta_predicate

            # Register the raised or cleared functions
            self.alarms[alarm_name].add_callback(h.raised, call_when_cleared=False, severity_required=severity)
            self.alarms[alarm_name].add_callback(h.cleared, call_when_raised=False, severity_required=severity)

            rospy.loginfo("Loaded handler: {}".format(h.alarm_name))

    def _create_meta_alarms(self, namespace="meta_alarms/"):
        ''' Adds meta alarms to the alarm server
        Meta alarms are special in that they are not directly raised or cleared but are instead triggered
        by a change of state of their child alarms.

        The /meta_alarms parameter defines a the structure of a meta alarm. It has the following structure:
        {meta_alarm_name : [list of child alarm names], ...}

        Users can also provide more complex triggering mechanisms by providing an alarm handler class with
        a 'meta_predicate' method.
        '''

        meta_alarms_dict = rospy.get_param(namespace, {})
        for meta, alarms in meta_alarms_dict.iteritems():
            # Add the meta alarm
            if meta not in self.alarms:
                self.alarms[meta] = self.make_tagged_alarm(meta)

            def default(meta, alarms):
                '''
                If no predicate for a meta-alarm is provided, then the meta-alarm will be raised
                if any of the child alarms are raised
                '''
                return any(alarms.items())

            self.meta_alarms[meta] = default

            def cb(alarm, meta_name=meta, sub_alarms=alarms):
                return self._handle_meta_alarm(meta_name, sub_alarms)

            for alarm in alarms:
                if alarm not in self.alarms:
                    self.alarms[alarm] = self.make_tagged_alarm(alarm)

                self.alarms[alarm].add_callback(cb)


if __name__ == "__main__":
    rospy.init_node("alarm_server")
    a = AlarmServer()
    rospy.spin()
