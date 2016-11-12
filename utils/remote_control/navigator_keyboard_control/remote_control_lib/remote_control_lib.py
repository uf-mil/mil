#!/usr/bin/env python

'''
Remote Control Library: This object contains all of the common variables and
functions that are shared by the various remote control devices on NaviGator.
'''


import itertools

from geometry_msgs.msg import WrenchStamped
from navigator_alarm import AlarmBroadcaster
from navigator_alarm import AlarmListener
from navigator_msgs.srv import WrenchSelect
import rospy


__author__ = "Anthony Olive"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2016, MIL"
__license__ = "MIT"


class RemoteControl(object):

    def __init__(self, controller_name, wrench_pub=None):
        self.name = controller_name
        self.wrench_choices = itertools.cycle(['rc', 'keyboard', 'autonomous'])

        self.alarm_broadcaster = AlarmBroadcaster()
        self.kill_alarm = self.alarm_broadcaster.add_alarm(
            name='kill',
            action_required=True,
            severity=0
        )
        self.station_hold_alarm = self.alarm_broadcaster.add_alarm(
            name='station_hold',
            action_required=False,
            severity=3
        )

        # rospy.wait_for_service('/change_wrench')
        self.wrench_changer = rospy.ServiceProxy('/change_wrench', WrenchSelect)
        self.kill_listener = AlarmListener('kill', self.update_kill_status)

        if (wrench_pub is None):
            self.wrench_pub = None
        else:
            self.wrench_pub = rospy.Publisher(wrench_pub, WrenchStamped, queue_size=1)

        self.is_killed = False
        self.is_timed_out = False
        self.clear_wrench()

    def timeout_check(function):
        def decorated_function(self):
            if (not self.is_timed_out):
                function(self)
        return decorated_function

    def update_kill_status(self, alarm):
        '''
        Updates the kill status display when there is an update on the kill
        alarm. Caches the last displayed kill status to avoid updating the
        display with the same information twice.
        '''
        if (alarm.clear):
            self.is_killed = False
        else:
            self.is_killed = True

    @timeout_check
    def kill(self):
        '''
        Kills the system regardless of what state it is in.
        '''
        rospy.loginfo("Toggling Kill")
        self.wrench_changer("rc")
        self.kill_alarm.raise_alarm(
            problem_description='System kill from location: {}'.format(self.name)
        )

    @timeout_check
    def toggle_kill(self):
        '''
        Toggles the kill status when the toggle_kill_button is pressed.
        '''
        rospy.loginfo("Toggling Kill")

        # Responds to the kill broadcaster and checks the status of the kill alarm
        if self.is_killed:
            self.kill_alarm.clear_alarm()
        else:
            self.wrench_changer("rc")
            self.kill_alarm.raise_alarm(
                problem_description='System kill from location: {}'.format(self.name)
            )

    @timeout_check
    def station_hold(self):
        '''
        Sets the goal point to the current location and switches to autonomous
        mode in order to stay at that point.
        '''
        rospy.loginfo("Station Holding")

        # Trigger station holding at the current pose
        self.station_hold_alarm.raise_alarm(
            problem_description='Request to station hold from: {}'.format(self.name)
        )

        self.wrench_changer("autonomous")

    @timeout_check
    def select_autonomous_control(self):
        '''
        Selects the autonomously generated trajectory as the active controller.
        '''
        rospy.loginfo("Changing Control to Autonomous")
        self.wrench_changer("autonomous")

    @timeout_check
    def select_rc_control(self):
        '''
        Selects the XBox remote joystick as the active controller.
        '''
        rospy.loginfo("Changing Control to RC")
        self.wrench_changer("rc")

    @timeout_check
    def select_keyboard_control(self):
        '''
        Selects the keyboard teleoperation service as the active controller.
        '''
        rospy.loginfo("Changing Control to Keyboard")
        self.wrench_changer("keyboard")

    @timeout_check
    def select_next_control(self):
        '''
        Selects the autonomously generated trajectory as the active controller.
        '''
        mode = next(self.wrench_choices)
        rospy.loginfo("Changing Control Mode: {}".format(mode))
        self.wrench_changer(mode)

    @timeout_check
    def publish_wrench(self, x, y, rotation, stamp=None):
        '''
        Publishes a wrench to the specified node based on force inputs from the
        controller.
        '''
        if (self.wrench_pub is not None):
            wrench = WrenchStamped()
            wrench.header.frame_id = "/base_link"
            wrench.header.stamp = stamp
            wrench.wrench.force.x = x
            wrench.wrench.force.y = y
            wrench.wrench.torque.z = rotation
            self.wrench_pub.publish(wrench)

    @timeout_check
    def clear_wrench(self):
        '''
        Publishes a wrench to the specified node based on force inputs from the
        controller.
        '''
        if (self.wrench_pub is not None):
            wrench = WrenchStamped()
            wrench.header.frame_id = "/base_link"
            wrench.wrench.force.x = 0
            wrench.wrench.force.y = 0
            wrench.wrench.torque.z = 0
            self.wrench_pub.publish(wrench)
