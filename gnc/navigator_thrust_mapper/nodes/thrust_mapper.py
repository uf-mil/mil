#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from roboteq_msgs.msg import Command
from ros_alarms import AlarmListener
from navigator_thrust_mapper import ThrusterMap


class ThrusterMapperNode(object):
    '''
    Node to publish individual thruster commands from the body frame wrench.
    See thruster_map.py for more details on this process as this is simply the ROS wrapper.
    '''
    def __init__(self):
        # Used for mapping wrench to individual thrusts
        self.thruster_map = ThrusterMap.from_ros_params()

        # To track kill state so no thrust is sent when killed (kill board hardware also ensures this)
        self.kill = False
        self.kill_listener = AlarmListener('kill', self.kill_cb)
        self.kill_listener.wait_for_server()

        # Start off with no wrench
        self.wrench = np.zeros(3)

        # ROS setup
        self.BL_pub = rospy.Publisher("/BL_motor/cmd", Command, queue_size=1)
        self.BR_pub = rospy.Publisher("/BR_motor/cmd", Command, queue_size=1)
        self.FL_pub = rospy.Publisher("/FL_motor/cmd", Command, queue_size=1)
        self.FR_pub = rospy.Publisher("/FR_motor/cmd", Command, queue_size=1)
        rospy.Subscriber("/wrench/cmd", WrenchStamped, self.wrench_cb)

    def kill_cb(self, alarm):
        self.kill = alarm.raised

    def wrench_cb(self, msg):
        '''
        Store last commanded wrench for later mapping to thrusts
        '''
        force = msg.wrench.force
        torque = msg.wrench.torque
        self.wrench = np.array((force.x, force.y, torque.z))

    def publish_thrusts(self):
        '''
        Use the mapper to find the individual thrusts needed to acheive the current body wrench.
        If killed, publish 0 to all thrusters just to be safe.
        '''
        BL, BR, FL, FR = Command(), Command(), Command(), Command()
        if not self.kill:  # Only get thrust of not killed, otherwise publish default 0 thrust
            BL.setpoint, BR.setpoint, FL.setpoint, FR.setpoint = self.thruster_map.wrench_to_thrusts(self.wrench)
        self.BL_pub.publish(BL)
        self.BR_pub.publish(BR)
        self.FL_pub.publish(FL)
        self.FR_pub.publish(FR)


if __name__ == "__main__":
    rospy.init_node('thrust_mapper')
    node = ThrusterMapperNode()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        node.publish_thrusts()
        rate.sleep()
