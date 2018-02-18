#!/usr/bin/env python


from remote_control_lib import RemoteControl
import rospy
from sensor_msgs.msg import Joy


__maintainer__ = "David Zobel"
__email__ = "zobeldavid@ufl.edu"
__copyright__ = "Copyright 2016, MIL"
__license__ = "MIT"


rospy.init_node("emergency")


class Joystick(object):

    def __init__(self):
        self.force_scale = rospy.get_param("/joystick_wrench/force_scale", 600)
        self.torque_scale = rospy.get_param("/joystick_wrench/torque_scale", 500)

        self.remote = RemoteControl("emergency", "/wrench/emergency")
        rospy.Subscriber("joy_emergency", Joy, self.joy_recieved)

        self.active = False
        self.reset()

    def reset(self):
        '''
        Used to reset the state of the controller. Sometimes when it
        disconnects then comes back online, the settings are all out of whack.
        '''
        self.last_kill = False
        self.last_station_hold_state = False
        self.last_auto_control = False
        self.last_emergency_control = False
        self.last_shooter_cancel = False
        self.last_change_mode = False

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
        self.last_time = rospy.Time.now()
        self.check_for_timeout(joy)

        # Assigns readable names to the buttons that are used
        start = joy.buttons[7]
        kill = bool(joy.buttons[2])
        station_hold = bool(joy.buttons[0])
        emergency_control = bool(joy.buttons[11])
        auto_control = bool(joy.buttons[12])
        shooter_cancel = bool(joy.buttons[1])
        change_mode = bool(joy.buttons[3])

        # Reset controller state if only start is pressed down about 3 seconds
        self.start_count += start
        if self.start_count > 10:
            rospy.loginfo("Resetting controller state")
            self.reset()
            self.active = True
            self.remote.clear_kill()

        if not self.active:
            return

        if kill and not self.last_kill:
            self.remote.toggle_kill()

        if station_hold and not self.last_station_hold_state:
            self.remote.station_hold()

        if auto_control and not self.last_auto_control:
            self.remote.select_autonomous_control()

        if emergency_control and not self.last_emergency_control:
            self.remote.select_emergency_control()

        if shooter_cancel and not self.last_shooter_cancel:
            self.remote.shooter_cancel()

        if change_mode and not self.last_change_mode:
            self.remote.select_next_control()

        self.last_kill = kill
        self.last_station_hold_state = station_hold
        self.last_auto_control = auto_control
        self.last_emergency_control = emergency_control
        self.last_shooter_cancel = shooter_cancel
        self.last_change_mode = change_mode

        # Scale joystick input to force and publish a wrench
        x = joy.axes[1] * self.force_scale
        y = joy.axes[0] * self.force_scale
        rotation = joy.axes[3] * self.torque_scale
        self.remote.publish_wrench(x, y, rotation, joy.header.stamp)

    def die_check(self, event):
        '''
        Publishes zeros after 2 seconds of no update
        in case node navigator_emergency_xbee dies
        '''
        if self.active:

            # No new instructions after 2 seconds
            if rospy.Time.now() - self.last_time > rospy.Duration(2):

                # Zero the wrench, reset
                self.reset()


if __name__ == "__main__":
    emergency = Joystick()
    rospy.Timer(rospy.Duration(1), emergency.die_check, oneshot=False)
    rospy.spin()
