#!/usr/bin/env python
from remote_control_lib import RemoteControl
import rospy
from sensor_msgs.msg import Joy


__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2016, MIL"
__license__ = "MIT"


rospy.init_node("joystick")


class Joystick(object):

    def __init__(self):
        self.force_scale = rospy.get_param("~force_scale", 600)
        self.torque_scale = rospy.get_param("~torque_scale", 500)

        self.remote = RemoteControl("joystick", "/wrench/rc")
        self.reset()
        rospy.Subscriber("joy", Joy, self.joy_recieved)

    def reset(self):
        '''
        Used to reset the state of the controller. Sometimes when it
        disconnects then comes back online, the settings are all out of whack.
        '''
        self.last_raise_kill = False
        self.last_clear_kill = False
        self.last_station_hold_state = False
        self.last_rc_control = False
        self.last_emergency_control = False
        self.last_keyboard_control = False
        self.last_back = False
        self.last_auto_control = False
        self.thruster_deploy_count = 0
        self.thruster_retract_count = 0

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

    def joy_recieved(self, joy):
        self.check_for_timeout(joy)

        # Assigns readable names to the buttons that are used
        start = joy.buttons[7]
        back = joy.buttons[6]
        raise_kill = bool(joy.buttons[1])  # B
        clear_kill = bool(joy.buttons[2])  # X
        station_hold = bool(joy.buttons[0])  # A
        rc_control = bool(joy.axes[6] > 0.9)  # d-pad left
        emergency_control = False
        keyboard_control = False
        auto_control = bool(joy.axes[6] < -0.9)  # d-pad right
        thruster_retract = bool(joy.buttons[4])  # LB
        thruster_deploy = bool(joy.buttons[5])  # RB

        if back and not self.last_back:
            rospy.loginfo('Back pressed. Going inactive')
            self.reset()
            return

        # Reset controller state if only start is pressed down about 1 second
        self.start_count += start
        if self.start_count > 5:
            rospy.loginfo("Resetting controller state")
            self.reset()
            self.active = True

        if not self.active:
            self.remote.clear_wrench()
            return

        if thruster_retract:
            self.thruster_retract_count += 1
        else:
            self.thruster_retract_count = 0
        if thruster_deploy:
            self.thruster_deploy_count += 1
        else:
            self.thruster_deploy_count = 0

        if self.thruster_retract_count > 10:
            self.remote.retract_thrusters()
            self.thruster_retract_count = 0
        elif self.thruster_deploy_count > 10:
            self.remote.deploy_thrusters()
            self.thruster_deploy_count = 0

        if raise_kill and not self.last_raise_kill:
            self.remote.kill()

        if clear_kill and not self.last_clear_kill:
            self.remote.clear_kill()

        if station_hold and not self.last_station_hold_state:
            self.remote.station_hold()

        if rc_control and not self.last_rc_control:
            self.remote.select_rc_control()

        if emergency_control and not self.last_emergency_control:
            self.remote.select_emergency_control()

        if keyboard_control and not self.last_keyboard_control:
            self.remote.select_keyboard_control()

        if auto_control and not self.last_auto_control:
            self.remote.select_autonomous_control()

        self.last_back = back
        self.last_raise_kill = raise_kill
        self.last_clear_kill = clear_kill
        self.last_station_hold_state = station_hold
        self.last_rc_control = rc_control
        self.last_emergency_control = emergency_control
        self.last_keyboard_control = keyboard_control
        self.last_auto_control = auto_control

        # Scale joystick input to force and publish a wrench
        x = joy.axes[1] * self.force_scale
        y = joy.axes[0] * self.force_scale
        rotation = joy.axes[3] * self.torque_scale
        self.remote.publish_wrench(x, y, rotation, rospy.Time.now())


if __name__ == "__main__":
    joystick = Joystick()
    rospy.spin()
