import rospy
from sub8_msgs.srv import UpdateThrusterLayout


class ThrusterOut(object):
    alarm_name = 'thruster_out'

    def __init__(self):
        # Keep some knowledge of which thrusters we have working
        self.dropped_thrusters = []
        self.update_layout = rospy.ServiceProxy('update_thruster_layout', UpdateThrusterLayout)

    def drop_thruster(self, thruster_name):
        self.dropped_thrusters.append(thruster_name)
        # Drop a single thruster
        rospy.logwarn("Dropping thuster {}".format(thruster_name))
        # a priori: Each thruster name is a string, do not need to map(str, remaining_thursters)
        rospy.logwarn("Dropped thrusters: " + ', '.join(self.dropped_thrusters))

    def handle(self, time_sent, parameters):
        self.drop_thruster(parameters['thruster_name'])
        self.update_layout(self.dropped_thrusters)
