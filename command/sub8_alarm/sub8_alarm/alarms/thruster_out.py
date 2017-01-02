import rospy
from sub8_msgs.srv import UpdateThrusterLayout


class Handler(object):
    alarm_name = 'thruster_out'

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

    def handle(self, time_sent, parameters, alarm_name):
        self.drop_thruster(parameters['thruster_name'])
        self.update_layout(self.dropped_thrusters)

    def cancel(self, time_sent, parameters, alarm_name):
        if parameters['clear_all']:
            # Used on startup of thruster mapper to clear all missing thrusters
            self.dropped_thrusters = []
            self.update_layout()

        elif parameters['thruster_name'] in self.dropped_thrusters:
            self.dropped_thrusters.pop(self.dropped_thrusters.index(parameters['thruster_name']))
            self.update_layout(self.dropped_thrusters)
