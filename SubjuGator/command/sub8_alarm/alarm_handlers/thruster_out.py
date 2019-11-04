import rospy
from ros_alarms import HandlerBase, Alarm
from sub8_msgs.srv import UpdateThrusterLayout


class ThrusterOut(HandlerBase):
    alarm_name = 'thruster-out'

    def __init__(self):
        # Alarm server wil set this as the intial state of kill alarm
        self.initial_alarm = Alarm(self.alarm_name, False,
                                   node_name='alarm_server',
                                   parameters={'offline_thruster_names': []})

        self._update_layout_proxy = rospy.ServiceProxy('update_thruster_layout', UpdateThrusterLayout)

    def update_layout(self, *args, **kwargs):
        try:
            self._update_layout_proxy(*args, **kwargs)
        except rospy.ServiceException as e:
            rospy.logwarn('Error updating thruster layout: {}'.format(e))

    def raised(self, alarm):
        self.update_layout(alarm.parameters['offline_thruster_names'])

    def cleared(self, alarm):
        self.update_layout(alarm.parameters['offline_thruster_names'])
