import rospy
from sub8_ros_tools import wait_for_param
from sub8_msgs.srv import UpdateThrusterLayout


class ThrusterOut(object):
    alarm_name = 'thruster_out'
    def __init__(self):
        # Keep some knowledge of which thrusters we have working
        self.thruster_layout = wait_for_param('busses')
        self.update_layout = rospy.ServiceProxy('update_thruster_layout', UpdateThrusterLayout)

    def drop_thruster(self, thruster_name):
        # Drop a single thruster
        remaining_thrusters = []
        for port in self.thruster_layout:
            if thruster_name in port['thrusters']:
                port['thrusters'].pop(thruster_name)

            remaining_thrusters.extend(port['thrusters'].keys())

        rospy.logwarn("Dropping thuster {}".format(thruster_name))
        # a priori: Each thruster name is a string, do not need to map(str, remaining_thursters)
        rospy.logwarn("Thrusters remaining " + ', '.join(remaining_thrusters))

    def handle(self, time_sent, parameters):
        self.drop_thruster(parameters['thruster_name'])
        rospy.set_param('busses', self.thruster_layout)
        self.update_layout()