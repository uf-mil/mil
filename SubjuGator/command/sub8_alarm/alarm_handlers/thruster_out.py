import rospy
from ros_alarms import HandlerBase, Alarm
from sub8_msgs.srv import UpdateThrusterLayout


class ThrusterOut(HandlerBase):
    """
    Alarm class to indicate that some of the thrusters in the thruster layout are
    offline.

    Attributes:
        alarm_name (str): The name of the alarm. Set to ``thruster-out``.
    """
    alarm_name = 'thruster-out'

    def __init__(self):
        # Alarm server wil set this as the intial state of kill alarm
        self.initial_alarm = Alarm(self.alarm_name, False,
                                   node_name='alarm_server',
                                   parameters={'offline_thruster_names': []})

        self._update_layout_proxy = rospy.ServiceProxy('update_thruster_layout', UpdateThrusterLayout)

    def update_layout(self, *args, **kwargs):
        """
        Attempts to update the thruster layout with the name of any offline thrusters.
        """
        try:
            self._update_layout_proxy(*args, **kwargs)
        except rospy.ServiceException as e:
            rospy.logwarn('Error updating thruster layout: {}'.format(e))

    def raised(self, alarm: Alarm):
        """
        Called when the alarm is raised. Attempts to update the thruster layout with
        :meth:`~.update_layout`.
        """
        self.update_layout(alarm.parameters['offline_thruster_names'])

    def cleared(self, alarm: Alarm):
        """
        Called when the alarm is cleared. Attempts to update the thruster layout with
        :meth:`~.update_layout`.
        """
        self.update_layout(alarm.parameters['offline_thruster_names'])
