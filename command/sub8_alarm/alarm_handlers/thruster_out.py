import rospy
from ros_alarms import HandlerBase, AlarmBroadcaster
from sub8_msgs.srv import UpdateThrusterLayout


class ThrusterOut(HandlerBase):
    alarm_name = 'thruster-out'

    def __init__(self):
        # Keep some knowledge of which thrusters we have working
        self.dropped_thrusters = []
        self.update_layout = rospy.ServiceProxy('update_thruster_layout', UpdateThrusterLayout)

    def raised(self, alarm):
        if alarm.parameters is None:
            return

        self.update_layout(alarm.parameters['thrusters_out'])

    def cleared(self, alarm):
        self.update_layout([])

