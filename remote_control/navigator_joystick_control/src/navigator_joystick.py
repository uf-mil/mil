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

rospy.init_node("joystick")

class JOYSTICK(object):
    # Base class for whatever you are writing
    def __init__(self):

        self.force_scale = rospy.get_param("force_scale")
        self.torque_scale = rospy.get_param("torque_scale")

        self.wrench_controller = True;
        self.last_controller_state = 0;
        self.last_station_hold_state = 0;
        self.last_kill_state = 0
        self.current_pose = Odometry()

        alarm_broadcaster = AlarmBroadcaster()
        thruster_out_alarm = alarm_broadcaster.add_alarm(
            name='full_kill',
            action_required=True,
            severity=0
        )

        self.wrench_pub = rospy.Publisher("/wrench/rc", WrenchStamped, queue_size = 1)
        self.des_pose_pub = rospy.Publisher("/set_desired_pose", Point, queue_size = 1)
        rospy.wait_for_service('/change_wrench')
        self.wrench_changer = rospy.ServiceProxy('/change_wrench', wrench_arbiter)
        rospy.Subscriber("joy", Joy, self.joy_cb)
        rospy.Subscriber("odom", Odometry, self.odom_cb)

    def odom_cb(self, msg):
        self.current_pose = msg

    def joy_cb(self, joy):

        change_mode = joy.buttons[7]
        kill = joy.buttons[8]
        station_hold = joy.buttons[0]
        left_stick_x = joy.axes[1]
        left_stick_y = joy.axes[0]
        right_stick_y = joy.axes[3]


        # Change vehicle mode
        if change_mode == 1 and change_mode != self.last_controller_state:
            self.wrench_controller = not self.wrench_controller
            if self.wrench_controller == False:
                self.wrench_changer("rc")
            if self.wrench_controller == True:
                self.wrench_changer("autonomous")

        if station_hold == 1 and station_hold != self.last_station_hold_state:
            des_pose = Point()
            q = np.array((self.current_pose.pose.orientation.x, self.current_pose.orientation.y, self.current_pose.pose.pose.orientation.z, self.current_pose.pose.pose.orientation.w))
            rotation = tf.transformations.euler_from_quaternion(q)

            point.x = self.current_pose.pose.pose.position.x;
            point.y = self.current_pose.pose.pose.position.y;
            point.z = rotation[2];
            self.des_pose_pub.publish(des_pose);
            self.wrench_changer("autonomous")

        if kill == 1 and kill != self.last_kill_state:
            thruster_out_alarm.raise_alarm(
                problem_description='System kill from location: {}'.format("joystick"),
                parameters={None}
            )

        self.last_controller_state = change_mode
        self.last_kill_state = kill
        self.last_station_hold_state = station_hold

        wrench = WrenchStamped()
        wrench.header.frame_id = "/base_link";
        wrench.wrench.force.x = self.force_scale * left_stick_x;
        wrench.wrench.force.y = -1 * self.force_scale_* left_stick_y;
        wrench.wrench.torque.z = -1 * self.torque * right_stick_y;
        self.wrench_pub.publish(wrench);


if __name__ == "__main__":


    joystick = JOYSTICK()
    rospy.spin()
