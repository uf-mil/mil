import rospy
from ros_alarms.srv import AlarmSet, AlarmGet, AlarmSetRequest, AlarmGetRequest
import json


class AlarmBroadcaster(object):
    def __init__(self, name, node_name=None):
        self._alarm_name = name
        self._node_name = rospy.get_name() if node_name is None else node_name

        self._alarm_set = rospy.ServiceProxy("/alarm/set", AlarmSet)
        rospy.wait_for_service("/alarm/set")
        rospy.loginfo("Created alarm broadcaster for alarm {}".format(name))

    def _generate_request(self, raised, action_required=False, problem_description="", 
                          parameters={}, severity=5):
        request = AlarmSetRequest()
        request.alarm.alarm_name = self._alarm_name
        request.alarm.node_name = self._node_name

        request.alarm.raised = raised
        request.alarm.action_required = action_required
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
    def __init__(self, name, node_name=None):
        self._alarm_name = name

        self._alarm_get = rospy.ServiceProxy("/alarm/get", AlarmGet)
        rospy.wait_for_service("/alarm/get")

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
