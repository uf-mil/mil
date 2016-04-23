#!/usr/bin/env python

import rospy
import roslib
import numpy as np
import math
import tf
from geometry_msgs.msg import WrenchStamped, Twist, Point
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from navigator_msg_multiplexer.srv import wrench_arbiter
from sub8_alarm import AlarmBroadcaster

rospy.init_node("joystick", log_level = rospy.DEBUG)

class JOYSTICK(object):
    # Base class for whatever you are writing
    def __init__(self):

        self.force_scale = rospy.get_param("~force_scale")
        self.torque_scale = rospy.get_param("~torque_scale")

        self.last_controller_state = 0;
        self.last_station_hold_state = 0;
        self.last_kill_state = 0
        self.last_revive_state = 0
        self.current_pose = Odometry()

        self.wrench_toggle_turn = 0

        self.alarm_broadcaster = AlarmBroadcaster()

        self.full_kill_alarm = self.alarm_broadcaster.add_alarm(
            name='full_kill',
            action_required=True,
            severity=0
        )

        self.revive_alarm = self.alarm_broadcaster.add_alarm(
            name='revive',
            action_required=True,
            severity=0
        )

        self.docking_alarm = self.alarm_broadcaster.add_alarm(
            name='half_kill',
            action_required=True,
            severity=0
        )

        self.wrench_pub = rospy.Publisher("/wrench/rc", WrenchStamped, queue_size = 1)
        self.des_pose_pub = rospy.Publisher("/set_desired_pose", Point, queue_size = 1)
        #rospy.wait_for_service('/change_wrench')
        self.wrench_changer = rospy.ServiceProxy('/change_wrench', wrench_arbiter)
        rospy.Subscriber("joy", Joy, self.joy_cb)
        rospy.Subscriber("odom", Odometry, self.odom_cb)

    def odom_cb(self, msg):
        self.current_pose = msg

    def joy_cb(self, joy):


        change_mode = joy.buttons[7]
        revive = joy.buttons[1]
        kill = joy.buttons[8]
        station_hold = joy.buttons[0]
        docking = joy.buttons[6]
        left_stick_x = joy.axes[1]
        left_stick_y = joy.axes[0]
        right_stick_y = joy.axes[3]

        # Change vehicle mode
        if change_mode == 1 and change_mode != self.last_controller_state:
            rospy.logdebug("Changing Control Mode")
            if self.wrench_toggle_turn == 0:
                self.wrench_changer("rc")
                self.wrench_toggle_turn +=1
            elif self.wrench_toggle_turn == 1:
                self.wrench_changer("autonomous")
                self.wrench_toggle_turn +=1
            elif self.wrench_toggle_turn == 2:
                self.wrench_changer("gui")
                self.wrench_toggle_turn +=1

            if self.wrench_toggle_turn == 3:
                self.wrench_toggle_turn = 0

        # Station hold
        if station_hold == 1 and station_hold != self.last_station_hold_state:
            rospy.logdebug("Station Holding")
            des_pose = Point()
            orientation = self.current_pose.pose.pose.orientation
            q = np.array((orientation.x, orientation.y, orientation.z, orientation.w))
            rotation = tf.transformations.euler_from_quaternion(q)
            des_pose.x = self.current_pose.pose.pose.position.x;
            des_pose.y = self.current_pose.pose.pose.position.y;
            des_pose.z = rotation[2];
            self.des_pose_pub.publish(des_pose);
            self.wrench_changer("autonomous")

        # Turn on full system kill
        if kill == 1 and kill != self.last_kill_state:
            rospy.logdebug("Toggling Kill")
            self.full_kill_alarm.raise_alarm(
                problem_description='System kill from location: {}'.format("joystick"),
                parameters={None: None}
            )

        if revive == 1 and revive != self.last_revive_state:
            rospy.logdebug("Toggling Kill")
            self.revive_alarm.raise_alarm(
                problem_description='Revive from location: {}'.format("joystick"),
                parameters={None: None}
            )

        # Turn on docking mode
        if docking == 1 and docking != self.last_docking_state:
            rospy.logdebug("Toggling Docking")
            self.docking_alarm.raise_alarm(
                problem_description='Docking kill from location: {}'.format("joystick"),
                parameters={None: None}
            )

        self.last_controller_state = change_mode
        self.last_kill_state = kill
        self.last_station_hold_state = station_hold
        self.last_docking_state = docking
        self.last_revive_state = revive

        wrench = WrenchStamped()
        wrench.header.frame_id = "/base_link";
        wrench.wrench.force.x = self.force_scale * left_stick_x;
        wrench.wrench.force.y = self.force_scale * left_stick_y;
        wrench.wrench.torque.z = self.torque_scale * right_stick_y;
        self.wrench_pub.publish(wrench);


if __name__ == "__main__":

    joystick = JOYSTICK()
    rospy.spin()
