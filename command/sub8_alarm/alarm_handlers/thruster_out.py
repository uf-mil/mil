import rospy
from ros_alarms import HandlerBase
from sub8_msgs.srv import UpdateThrusterLayout


class ThrusterOut(HandlerBase):
    alarm_name = 'thruster-out'

    def __init__(self):
        self.update_layout = rospy.ServiceProxy('update_thruster_layout', UpdateThrusterLayout)

    def raised(self, alarm):
        if alarm.parameters is None:
            return

        self.update_layout(map(str, alarm.parameters['thruster_names']))

    def cleared(self, alarm):
        if alarm.parameters.get("clear_all", False):
            self.update_layout()
