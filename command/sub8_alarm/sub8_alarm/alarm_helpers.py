import rospy
from sub8_msgs.msg import Alarm
from std_msgs.msg import Header
import json


class AlarmBroadcaster(object):
    def __init__(self):
        '''Alarm Broadcaster'''
        # Notify the world when our alarm happens
        self.alarm_pub = rospy.Publisher('/alarm', Alarm, latch=True, queue_size=10)

        # Notify the world that our alarm has spawned
        # For now, unused - future: Use for system reports
        # self.alarm_spawn_pub = rospy.Publisher('/alarm_spawn', Alarm, latch=True, queue_size=1)

        # Get ROS name
        self.node_name = rospy.get_name()

        # Alarms associated with this broadcaster
        # In the future, there will be some background bookkeeping
        self.alarms = []

    def add_alarm(self, name, action_required=False, severity=3, problem_description=None, parameters=None):
        '''Factory method for creating a new alarm'''
        new_alarm = AlarmRaiser(
            alarm_name=name,
            node_name=self.node_name,
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
        '''Alarm object, does alarms
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

        self._alarm_pub = alarm_publisher

    def raise_alarm(self, problem_description=None, parameters=None):
        '''Arguments are override parameters'''
        got_problem_description = (problem_description != "") or (self._problem_description is not None)
        assert got_problem_description, "No problem description has been set for this alarm"

        if parameters is not None:
            assert isinstance(parameters, dict), "Parameters must be in the form of  dictionary"

        if problem_description is not None:
            _problem_description = problem_description
        else:
            _problem_description = self.problem_description

        if parameters is not None:
            _parameters = parameters
        else:
            _parameters = self._parameters

        alarm_msg = Alarm(
            header=Header(
                stamp=rospy.Time.now()
            ),
            action_required=self._action_required,
            problem_description=_problem_description,
            parameters=json.dumps(_parameters),
            severity=self._severity,
            alarm_name=self._alarm_name,
            node_name=self._node_name,
        )

        self._alarm_pub.publish(alarm_msg)
        return alarm_msg
