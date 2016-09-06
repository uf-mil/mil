#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped
from navigator_msgs.srv import WrenchSelect
from std_msgs.msg import Bool

rospy.init_node('wrench_arbiter')

class WrenchArbiter(object):
    def __init__(self):
        # Set the three receiving variables
        self.rc_wrench = None
        self.autonomous_wrench = None

        # This is what is used to determine which input should be output
        self.control = None
        self.control_inputs = ['rc', 'autonomous']

        # ROS stuff - Wrench changing service, final output wrench, and the three input sources
        rospy.Service('change_wrench', WrenchSelect, self.change_wrench)
        self.wrench_pub = rospy.Publisher("/wrench/cmd", WrenchStamped, queue_size=1)
        self.learn = rospy.Publisher("learn", Bool, queue_size=1)

        # Subscribers to listen for wrenches
        rospy.Subscriber("/wrench/rc", WrenchStamped,
                         lambda msg: self.republish(msg, control_t="rc", learn=False))
        rospy.Subscriber("/wrench/autonomous", WrenchStamped,
                         lambda msg: self.republish(msg, control_t="autonomous", learn=True))

    def republish(self, msg, control_t, learn):
        '''
        Republishes message if it's the correct control type. `learn` will publish to the learn topic
        to start or stop concurrent learning.
        '''
        # If there's no control selected, send zeros
        if self.control is None:
            msg.wrench.force.x = 0
            msg.wrench.force.y = 0
            msg.wrench.torque.z = 0
            learn = False

        elif control_t != self.control:
            return

        self.learn.publish(learn)
        msg.header.frame_id = "/base_link"
        self.wrench_pub.publish(msg)

    def change_wrench(self, req):
        '''
        This sets the wrench output to the correct source by setting the 'control' variable
        to the right source.
        '''
        rospy.loginfo("Server received request for wrench control change - " + req.str)
        self.control = req.str if req.str in self.control_inputs else None
        return self.control is not None


if __name__ == "__main__":
    arb = WrenchArbiter()
    rospy.spin()
