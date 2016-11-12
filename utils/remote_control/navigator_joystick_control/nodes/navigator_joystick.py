#!/usr/bin/env python


import actionlib
from navigator_msgs.msg import ShooterDoAction, ShooterDoActionGoal
from remote_control_lib import RemoteControl
import rospy
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger, TriggerRequest


__author__ = "Anthony Olive"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2016, MIL"
__license__ = "MIT"


rospy.init_node('joystick')


class Joystick(object):

    def __init__(self):
        self.force_scale = rospy.get_param("~force_scale", 600)
        self.torque_scale = rospy.get_param("~torque_scale", 500)

        self.remote = RemoteControl('joystick', "/wrench/rc")

        self.shooter_fire_client = actionlib.SimpleActionClient('/shooter/fire', ShooterDoAction)
        self.shooter_load_client = actionlib.SimpleActionClient('/shooter/load', ShooterDoAction)
        self.shooter_cancel_client = rospy.ServiceProxy('/shooter/cancel', Trigger)

        rospy.Subscriber("joy", Joy, self.joy_recieved)

        self.active = False
        self.reset()

    def reset(self):
        '''
        Used to reset the state of the controller.
        Sometimes when it disconnects then comes back online, the settings are all
            out of wack.
        '''
        self.last_kill = False
        self.last_station_hold_state = False
        self.last_change_mode = False
        self.last_auto_control = False
        self.last_rc_control = False
        self.last_keyboard_control = False
        self.last_shooter_shoot = False
        self.last_shooter_cancel = False
        self.last_shooter_load = False

        self.start_count = 0
        self.last_joy = None
        self.active = False

        self.remote.clear_wrench()

    def check_for_timeout(self, joy):
        if self.last_joy is None:
            self.last_joy = joy
            return

        if joy.axes == self.last_joy.axes and joy.buttons == self.last_joy.buttons:

            # No change in state
            if rospy.Time.now() - self.last_joy.header.stamp > rospy.Duration(15 * 60):

                # The controller times out after 15 minutes
                if self.active:
                    rospy.logwarn("Controller Timed out. Hold start to resume.")
                    self.reset()

        else:
            joy.header.stamp = rospy.Time.now()  # In the sim, stamps weren't working right
            self.last_joy = joy

    def shooter_load_cb(self, status, result):
        rospy.loginfo("Shooter Load Status={} Success={} Error={}".format(status, result.success, result.error))

    def shooter_fire_cb(self, status, result):
        rospy.loginfo("Shooter Fire Status={} Success={} Error={}".format(status, result.success, result.error))

    def joy_recieved(self, joy):
        self.check_for_timeout(joy)

        # Assigns readable names to the buttons that are used
        start = joy.buttons[7]
        change_mode = bool(joy.buttons[3])  # Y
        kill = bool(joy.buttons[2])  # X
        station_hold = bool(joy.buttons[0])  # A
        rc_control = bool(joy.buttons[11])  # d-pad left
        auto_control = bool(joy.buttons[12])  # d-pad right
        keyboard_control = bool(joy.buttons[14])  # d-pad down
        shooter_shoot = joy.axes[5] < -0.9
        shooter_load = bool(joy.buttons[4])
        shooter_cancel = bool(joy.buttons[5])

        print keyboard_control

        # Reset controller state if only start is pressed down about 3 seconds
        self.start_count += start
        if self.start_count > 10:
            rospy.loginfo("Resetting controller state.")
            self.reset()
            self.active = True

            self.kill_alarm.clear_alarm()
            self.wrench_changer("rc")

        if not self.active:
            return

        if kill and kill != self.last_kill:
            self.remote.toggle_kill()

        if station_hold and station_hold != self.last_station_hold_state:
            self.remote.station_hold()

        if change_mode and change_mode != self.last_change_mode:
            self.remote.select_next_control()

        if auto_control and auto_control != self.last_auto_control:
            self.remote.select_autonomous_control()

        if rc_control and rc_control != self.last_rc_control:
            self.remote.select_rc_control()

        if keyboard_control and keyboard_control != self.last_keyboard_control:
            self.remote.select_keyboard_control()

        if shooter_shoot and not self.last_shooter_shoot:
            rospy.loginfo("Joystick input : Shoot")
            self.shooter_fire_client.send_goal(goal=ShooterDoActionGoal(), done_cb=self.shooter_fire_cb)

        if shooter_load and not self.last_shooter_load:
            rospy.loginfo("Joystick input : Load")
            self.shooter_load_client.send_goal(goal=ShooterDoActionGoal(), done_cb=self.shooter_load_cb)

        if shooter_cancel and not self.last_shooter_cancel:
            rospy.loginfo("Joystick input : Cancel")
            self.shooter_cancel_client(TriggerRequest())

        self.last_kill = kill
        self.last_station_hold_state = station_hold
        self.last_change_mode = change_mode
        self.last_auto_control = auto_control
        self.last_rc_control = rc_control
        self.last_keyboard_control = keyboard_control
        self.last_shooter_shoot = shooter_shoot
        self.last_shooter_cancel = shooter_cancel
        self.last_shooter_load = shooter_load

        # Scale joystick input to force and publish a wrench
        x = joy.axes[1] * self.force_scale
        y = joy.axes[0] * self.force_scale
        rotation = joy.axes[3] * self.torque_scale
        self.remote.publish_wrench(x, y, rotation, joy.header.stamp)


if __name__ == "__main__":
    joystick = Joystick()
    rospy.spin()
