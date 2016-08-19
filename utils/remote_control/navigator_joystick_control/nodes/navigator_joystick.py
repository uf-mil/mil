#!/usr/bin/env python

import rospy
import roslib
import numpy as np
import math
import tf
import itertools
from navigator_tools import make_header
from geometry_msgs.msg import WrenchStamped, Twist, PoseStamped
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from navigator_msg_multiplexer.srv import wrench_arbiter
from kill_handling.broadcaster import KillBroadcaster
from sub8_alarm import AlarmBroadcaster

rospy.init_node("joystick")

class Joystick(object):
    # Base class for whatever you are writing
    def __init__(self):

        self.force_scale = rospy.get_param("~force_scale")
        self.torque_scale = rospy.get_param("~torque_scale")

        self.last_change_mode = False
        self.last_station_hold_state = False
        self.last_kill = False
        self.last_rc = False
        self.last_auto = False

        self.killed = False
        self.docking = False

        self.wrench_choices = itertools.cycle(['rc', 'autonomous'])

        self.current_pose = Odometry()

        self.alarm_broadcaster = AlarmBroadcaster()

        self.kill_alarm = self.alarm_broadcaster.add_alarm(
            name='kill',
            action_required=True,
            severity=0
        )

        self.docking_alarm = self.alarm_broadcaster.add_alarm(
            name='docking',
            action_required=True,
            severity=0
        )

        self.wrench_pub = rospy.Publisher("/wrench/rc", WrenchStamped, queue_size=1)
        self.kb = KillBroadcaster(id='station_hold', description='Reset Pose')
        #rospy.wait_for_service('/change_wrench')
        self.wrench_changer = rospy.ServiceProxy('/change_wrench', wrench_arbiter)

        rospy.Subscriber("joy", Joy, self.joy_cb)
        rospy.Subscriber("odom", Odometry, self.odom_cb)

    def odom_cb(self, msg):
        self.current_pose = msg

    def joy_cb(self, joy):

        # Handle Button presses
        change_mode = bool(joy.buttons[3])  # Y
        kill = bool(joy.buttons[2])  # X
        station_hold = bool(joy.buttons[0])  # A
        docking = bool(joy.buttons[1])  # B
        rc_control = bool(joy.buttons[11])  # d-pad left
        auto_control = bool(joy.buttons[12])  # d-pad right

        # Change vehicle mode
        if change_mode == 1 and change_mode != self.last_change_mode:
            mode = next(self.wrench_choices)
            rospy.loginfo("Changing Control Mode: {}".format(mode))
            self.wrench_changer(mode)
        
        if rc_control == 1 and rc_control != self.last_rc:
            rospy.loginfo("Changing Control to RC")
            self.wrench_changer("rc")

        if auto_control == 1 and auto_control != self.last_auto:
            rospy.loginfo("Changing Control to Autonomous")
            self.wrench_changer("autonomous")

        # Station hold
        if station_hold == 1 and station_hold != self.last_station_hold_state:
            rospy.loginfo("Station Holding")
            self.kb.send(active=True)
            self.kb.send(active=False)  # Resets c3, this'll change when c3 is replaced
            self.wrench_changer("autonomous")

        # Turn on full system kill
        if kill == 1 and kill != self.last_kill:
            rospy.loginfo("Toggling Kill")
            if self.killed:
                self.kill_alarm.clear_alarm()
            else:
                self.wrench_changer("rc")
                self.kill_alarm.raise_alarm(
                    problem_description='System kill from location: joystick'
                )

            self.killed = not self.killed

        # Turn on docking mode
        if docking == 1 and docking != self.last_docking_state:
            rospy.loginfo("Toggling Docking")

            if self.docking:
                self.docking_alarm.clear_alarm()
            else:
                self.docking_alarm.raise_alarm(
                    problem_description='Docking kill from location: joystick'
                )

            self.docking = not self.docking
            

        self.last_change_mode = change_mode
        self.last_kill = kill
        self.last_station_hold_state = station_hold
        self.last_docking_state = docking
        self.last_auto_control = auto_control
        self.last_rc = rc_control
        self.last_auto = auto_control

        # Handle joystick commands
        left_stick_x = joy.axes[1]
        left_stick_y = joy.axes[0]
        right_stick_y = joy.axes[3]

        wrench = WrenchStamped()
        wrench.header.frame_id = "/base_link"
        wrench.wrench.force.x = self.force_scale * left_stick_x
        wrench.wrench.force.y = self.force_scale * left_stick_y
        wrench.wrench.torque.z = self.torque_scale * right_stick_y
        self.wrench_pub.publish(wrench)


if __name__ == "__main__":

    joystick = Joystick()
    rospy.spin()
