#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
from navigator_thrust_mapper import ThrusterMap
from roboteq_msgs.msg import Command
from ros_alarms import AlarmListener
from ros_alarms.msg import Alarm
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


class ThrusterMapperNode:
    """
    Node to publish individual thruster commands from the body frame wrench.
    See thruster_map.py for more details on this process as this is simply the ROS wrapper.
    """

    def __init__(self):
        self.is_vrx = rospy.get_param("/is_vrx", default=False)

        # Used for mapping wrench to individual thrusts
        urdf = rospy.get_param("/robot_description", default=None)
        if urdf is None or len(urdf) == 0:
            raise Exception("robot description not set or empty")
        if self.is_vrx:
            self.thruster_map = ThrusterMap.from_vrx_urdf(urdf)
        else:
            self.thruster_map = ThrusterMap.from_urdf(urdf)

        # To track kill state so no thrust is sent when killed (kill board hardware also ensures this)
        self.kill = False
        if not self.is_vrx:
            self.kill_listener = AlarmListener("kill", self.kill_callback)
            self.kill_listener.wait_for_server()

        # Start off with no wrench
        self.wrench = np.zeros(3)

        # Publisher for each thruster
        if self.is_vrx:
            self.publishers = [
                rospy.Publisher(
                    f"/wamv/thrusters/{name[5:]}_thrust_cmd",
                    Float32,
                    queue_size=1,
                )
                for name in self.thruster_map.names
            ]
        else:
            self.publishers = [
                rospy.Publisher(f"/{name}_motor/cmd", Command, queue_size=1)
                for name in self.thruster_map.names
            ]

        # Joint state publisher
        # TODO(ironmig):
        if not self.is_vrx:
            self.joint_state_pub = rospy.Publisher(
                "/thruster_states", JointState, queue_size=1
            )
            self.joint_state_msg = JointState()
            for name in self.thruster_map.joints:
                self.joint_state_msg.name.append(name)
                self.joint_state_msg.position.append(0)
                self.joint_state_msg.effort.append(0)

        rospy.Subscriber("/wrench/cmd", WrenchStamped, self.wrench_cb)

    def kill_callback(self, alarm: Alarm) -> None:
        """
        Serves as the callback for the ROS 'kill' alarm.

        Args:
            alarm: Alarm - The alarm message that triggered
            the callback
        """
        self.kill = alarm.raised

    def wrench_cb(self, msg: WrenchStamped) -> None:
        """
        Store last commanded wrench for later mapping to thrusts.

        Operates as the callback function for the subscriber of the /wrench/cmd topic.

        Args:
            msg: WrenchStamped - The messaged passed to the
            callback from the topic.
        """
        force = msg.wrench.force
        torque = msg.wrench.torque
        self.wrench = np.array((force.x, force.y, torque.z))

    def publish_thrusts(self) -> None:
        """
        Use the mapper to find the individual thrusts needed to acheive the current body wrench.
        If killed, publish 0 to all thrusters just to be safe.
        """
        commands = [Command() for i in range(len(self.publishers))]
        if not self.kill:
            thrusts = self.thruster_map.wrench_to_thrusts(self.wrench)
            if len(thrusts) != len(self.publishers):
                rospy.logfatal("Number of thrusts does not equal number of publishers")
                return
            for i in range(len(self.publishers)):
                commands[i].setpoint = thrusts[i]
        if not self.is_vrx:
            for i in range(len(self.publishers)):
                self.joint_state_msg.effort[i] = commands[i].setpoint
                self.publishers[i].publish(commands[i])
            self.joint_state_pub.publish(self.joint_state_msg)
        else:
            for i in range(len(self.publishers)):
                self.publishers[i].publish(commands[i].setpoint)


if __name__ == "__main__":
    rospy.init_node("thrust_mapper")
    node = ThrusterMapperNode()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        node.publish_thrusts()
        rate.sleep()
