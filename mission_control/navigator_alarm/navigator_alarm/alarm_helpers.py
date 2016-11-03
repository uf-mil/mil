import txros
from twisted.internet import defer

import rospy
from navigator_msgs.msg import Alarm
import navigator_tools
import json


def single_alarm(name, action_required=False, severity=3, problem_description=None, parameters=None):
    '''
        Use this only if your node has a single alarm
    '''
    ab = AlarmBroadcaster()
    alarm = ab.add_alarm(name, action_required, severity, problem_description, parameters)
    return ab, alarm


class AlarmBroadcaster(object):
    def __init__(self):
        '''
            Alarm Broadcaster
        '''
        # Notify the world when our alarm happens
        self.alarm_pub = rospy.Publisher('/alarm_raise', Alarm, latch=True, queue_size=10)

        # Notify the world that our alarm has spawned
        # For now, unused - future: Use for system reports
        # self.alarm_spawn_pub = rospy.Publisher('/alarm_spawn', Alarm, latch=True, queue_size=1)

        # Alarms associated with this broadcaster
        # In the future, there will be some background bookkeeping
        self.alarms = []

    def add_alarm(self, name, action_required=False, severity=3, problem_description=None, parameters=None, node_name=None):
        '''
            Factory method for creating a new alarm
        '''
        if node_name is None:
            node_name = rospy.get_name()

        new_alarm = AlarmRaiser(
            alarm_name=name,
            node_name=node_name,
            alarm_publisher=self.alarm_pub,
            action_required=action_required,
            problem_description=problem_description,
            parameters=parameters,
            severity=severity
        )

        self.alarms.append(new_alarm)
        return new_alarm


class AlarmRaiser(object):
    def __init__(self, alarm_name, node_name, alarm_publisher, action_required=False, severity=3, problem_description=None,
                 parameters=None):
        '''
            Alarm object, does alarms
            Generates alarm messages
        '''
        assert severity in range(0, 3 + 1), "Severity must be an integer between 0 and 3"
        if parameters is not None:
            assert isinstance(parameters, dict), "Parameters must be in the form of  dictionary"
        self._alarm_name = alarm_name
        self._node_name = node_name
        self._action_required = action_required
        self._severity = severity
        self._problem_description = problem_description
        self._parameters = parameters

        self._previous_stuff = {
            'alarm_name': self._alarm_name,
            'node_name': self._node_name,
            'action_required': self._action_required,
            'severity': self._severity,
            'problem_description': self._problem_description,
            'parameters': self._parameters,
            'clear': False,
        }

        self._alarm_pub = alarm_publisher

    def different(self, **kwargs):
        new_stuff = kwargs
        if new_stuff == self._previous_stuff:
            return False
        else:
            return True


    def raise_alarm(self, problem_description=None, parameters=None, severity=None):
        '''
            Arguments are override parameters
        '''
        got_problem_description = (problem_description != "") or (self._problem_description is not None)
        assert got_problem_description, "No problem description has been set for this alarm"

        # Allow overrideable severity
        if severity is None:
            severity = self._severity

        if parameters is not None:
            assert isinstance(parameters, dict), "Parameters must be in the form of  dictionary"

        if problem_description is not None:
            _problem_description = problem_description
        else:
            _problem_description = self._problem_description

        if parameters is not None:
            _parameters = parameters
        else:
            _parameters = self._parameters

        alarm_contents = {
            'action_required': self._action_required,
            'problem_description': _problem_description,
            'parameters': json.dumps(_parameters),
            'severity': self._severity,
            'alarm_name': self._alarm_name,
            'node_name': self._node_name,
            'clear': False,
        }
        if not self.different(**alarm_contents):
            return
        else:
            self._previous_stuff = alarm_contents

        alarm_msg = Alarm(
            header=navigator_tools.make_header(),
            **alarm_contents
        )

        self._alarm_pub.publish(alarm_msg)
        return alarm_msg

    def clear_alarm(self):
        alarm_contents = {
            'alarm_name': self._alarm_name,
            'node_name': self._node_name,
            'severity': self._severity,
            'action_required': False,
            'clear': True,
        }
        if not self.different(**alarm_contents):
            return
        else:
            self._previous_stuff = alarm_contents

        alarm_msg = Alarm(
            header=navigator_tools.make_header(),
            **alarm_contents
        )
        self._alarm_pub.publish(alarm_msg)


class AlarmListener(object):
    '''
        Listens for alarms (similar to a TF listener) but can have a callback function (like a ros subscriber).

        If an alarm with the 'alarm_name' is triggered, it (as well as an optional 'args') will be passed
        to the 'callback_funct'.
    '''
    def __init__(self, alarm_name=None, callback_funct=None):
        rospy.Subscriber('/alarm', Alarm, self._check_alarm, queue_size=100)

        # This dictionary will allow the user to listen to an arbitrary number of alarms
        # and have sepreate callbacks and args for each alarm.
        self.callback_linker = {}
        self.known_alarms = []

        if alarm_name is not None and callback_funct is not None:
            self.callback_linker[alarm_name] = {
                'last_time': None,
                'callback': callback_funct,
                'active': False,
            }

    def __getitem__(self, alarm_name):
        '''
            Return true if the alarm is active, false if not
        '''
        if alarm_name in self.callback_linker.keys():
            return self.callback_linker[alarm_name]['active']
        else:
            raise KeyError("{} is not a known alarm to this listener".format(alarm_name))

    def add_listener(self, alarm_name, callback_funct):
        '''
            Creates a new alarm listener and links it to a callback function.
        '''
        self.callback_linker[alarm_name] = {
            'callback': callback_funct,
            'last_time': None,
            'active': False,
        }

    def _check_alarm(self, alarm):
        '''
            We've gotten an alarm, but don't panic! If it's one of the alarms we are listening for,
            pass the alarm to the function callback with any specified args.
        '''

        if alarm.alarm_name not in self.known_alarms:
            self.known_alarms.append(alarm.alarm_name)

        found_alarm = self.callback_linker.get(alarm.alarm_name, None)

        if found_alarm is None:
            return

        if found_alarm['last_time'] is None:
            found_alarm['last_time'] = alarm.header.stamp

        elif alarm.header.stamp > found_alarm['last_time']:
            # Check if the alarm is new
            found_alarm['last_time'] = alarm.header.stamp
            found_alarm['active'] = not alarm.clear

        else:
            # Otherwise do nothing
            return

        callback = found_alarm['callback']
        callback(alarm)


class AlarmListenerTx():
    @txros.util.cancellableInlineCallbacks
    def init(self, nh):
        self.callback_linker = {}
        self.known_alarms = []

        self.nh = nh
        sub = yield self.nh.subscribe("/alarm", Alarm, self._check_alarm)
        yield self.nh.sleep(.1)  # Sleep to build alarm buffer

    def __getitem__(self, alarm_name):
        '''
            Return true if the alarm is active, false if not
        '''
        if alarm_name in self.callback_linker.keys():
            return self.callback_linker[alarm_name]['active']
        else:
            raise KeyError("{} is not a known alarm to this listener".format(alarm_name))

    def add_listener(self, alarm_name, callback_funct):
        '''
            Creates a new alarm listener and links it to a callback function.
            The callback will be immediately called with the current alarm's state.
        '''
        self.callback_linker[alarm_name] = {
            'callback': callback_funct,
            'last_time': None,
            'active': False,
            'last_alarm': None,
        }

    @txros.util.cancellableInlineCallbacks
    def wait_for_alarm(self, alarm_name, timeout=10):
        '''
            Assumes we have the alarm being listened to already (ie add_listener has been called already)
        '''
        found = False

        if alarm_name in self.known_alarms:
            found = self.callback_linker[alarm_name]['last_alarm']

        start_time = self.nh.get_time()  # Poor man's timeout
        while not found and self.nh.get_time() - start_time < txros.nodehandle.genpy.Duration(timeout):
            yield self.nh.sleep(.07)  # This is a bit more than the alarm pub frequency
            if alarm_name in self.known_alarms:
                found = self.callback_linker[alarm_name]['last_alarm']

        defer.returnValue(found)

    def _check_alarm(self, alarm):
        '''
            We've gotten an alarm, but don't panic! If it's one of the alarms we are listening for,
            pass the alarm to the function callback with any specified args.
        '''
        if alarm.alarm_name not in self.known_alarms:
            self.known_alarms.append(alarm.alarm_name)

        found_alarm = self.callback_linker.get(alarm.alarm_name, None)

        if found_alarm is None:
            return

        self.callback_linker['last_alarm'] = alarm

        if found_alarm['last_time'] is None:
            found_alarm['last_time'] = alarm.header.stamp

        elif alarm.header.stamp > found_alarm['last_time']:
            # Check if the alarm is new
            found_alarm['last_time'] = alarm.header.stamp
            found_alarm['active'] = not alarm.clear

        else:
            # Otherwise do nothing
            return

        callback = found_alarm['callback']
        callback(alarm)