import rospy
from ros_alarms import HandlerBase, AlarmBroadcaster
from sub8_msgs.srv import UpdateThrusterLayout


class ThrusterOut(HandlerBase):
    alarm_name = 'thruster-out'

    def __init__(self):
        # Keep some knowledge of which thrusters we have working
        self.dropped_thrusters = []
        self.update_layout = rospy.ServiceProxy('update_thruster_layout', UpdateThrusterLayout)
        self.ab = AlarmBroadcaster("thruster-kill", node_name="thruster_out_kill")

    def drop_thruster(self, thruster_name):
        if thruster_name not in self.dropped_thrusters:
            self.dropped_thrusters.append(thruster_name)

        # a priori: Each thruster name is a string, do not need to map(str, remaining_thursters)
        rospy.logwarn("Dropped thrusters: " + ', '.join(self.dropped_thrusters))
         
        # Kill if we lose too many thrusters
        if len(self.dropped_thrusters) > rospy.get_param("thruster_loss_limit", 2):
            self.ab.raise_alarm()
        else:
            self.ab.clear_alarm()

    def raised(self, alarm):
        if alarm.parameters is None:
            return

        self.drop_thruster(alarm.parameters['thruster_name'])
        self.update_layout(self.dropped_thrusters)

    def cleared(self, alarm):
        if alarm.parameters is None or alarm.parameters == {}:
            return

        if alarm.parameters.get('clear_all', False):
            # Used on startup of thruster mapper to clear all missing thrusters
            self.dropped_thrusters = []
            self.update_layout()

        elif alarm.parameters['thruster_name'] in self.dropped_thrusters:
            self.dropped_thrusters.pop(self.dropped_thrusters.index(alarm.parameters['thruster_name']))
            self.update_layout(self.dropped_thrusters)

            rospy.logwarn("Reviving thuster {}".format(alarm.parameters['thruster_name']))
            rospy.logwarn("Dropped thrusters: " + ', '.join(self.dropped_thrusters))
