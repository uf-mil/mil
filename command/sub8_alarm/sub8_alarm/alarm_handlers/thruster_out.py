import rospy
from _template import HandlerBase
from sub8_msgs.srv import UpdateThrusterLayout


class ThrusterOut(HandlerBase):
    alarm_name = 'thruster-out'

    def __init__(self):
        # Keep some knowledge of which thrusters we have working
        self.dropped_thrusters = []
        self.update_layout = rospy.ServiceProxy('update_thruster_layout', UpdateThrusterLayout)

    def drop_thruster(self, thruster_name):
        if thruster_name not in self.dropped_thrusters:
            self.dropped_thrusters.append(thruster_name)
        # Drop a single thruster
        rospy.logwarn("Dropping thuster {}".format(thruster_name))
        # a priori: Each thruster name is a string, do not need to map(str, remaining_thursters)
        rospy.logwarn("Dropped thrusters: " + ', '.join(self.dropped_thrusters))

    def handle(self, alarm, time_sent, parameters):
        if parameters is None:
            return

        self.drop_thruster(parameters['thruster_name'])
        self.update_layout(self.dropped_thrusters)

    def cancel(self, alarm, time_sent, parameters):
        if parameters is None:
            return

        if parameters.get('clear_all', False):
            # Used on startup of thruster mapper to clear all missing thrusters
            self.dropped_thrusters = []
            self.update_layout()

        elif parameters['thruster_name'] in self.dropped_thrusters:
            self.dropped_thrusters.pop(self.dropped_thrusters.index(parameters['thruster_name']))
            self.update_layout(self.dropped_thrusters)

            rospy.logwarn("Reviving thuster {}".format(parameters['thruster_name']))
            rospy.logwarn("Dropped thrusters: " + ', '.join(self.dropped_thrusters))
