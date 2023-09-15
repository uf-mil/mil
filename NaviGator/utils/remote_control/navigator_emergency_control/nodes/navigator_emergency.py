#!/usr/bin/env python3
import rospy
from remote_control_lib import RemoteControl
from sensor_msgs.msg import Joy

__maintainer__ = "David Zobel"
__email__ = "zobeldavid@ufl.edu"
__copyright__ = "Copyright 2016, MIL"
__license__ = "MIT"


class Joystick:
    """
    Stores the history of operations pertaining to how the joystick operates.

    Possible operations could include resetting several parameters (imported from one of the inherited classes)
    to their default values, keeping track of duration to determine a timeout period, and assigning values to attributes that
    are used as reference for conditional statements and other functionalities such as incrementation, reassigning values, etc.

    Attributes:
        force_scale (int): Sets the force scale that needs to be amplified by.
        torque_scale (int): Sets the torque scale that needs to be amplified by.
        last_raise_kill (bool): Determines whether the kill alarm is activated.
        last_clear_kill (bool): Clears the system kill no matter the state that it is in.
        last_station_hold_state (bool): Determines whether the goal point is set as the current location.
        last_emergency_control (bool): Determines where the emergency controller is the active controller.
        last_go_inactive (bool): Checks whether something is going inactive or not.
        thruster_deploy_count (int): The number of thrusters that are being deployed.
        thruster_retract_count (int): The number of thrusters that are being retracted.
        start_count (int): Number of seconds that start is being pressed down.
        active (bool): Indicates the current state of the controller.
    """

    def __init__(self):
        self.force_scale = rospy.get_param("/joystick_wrench/force_scale", 600)
        self.torque_scale = rospy.get_param("/joystick_wrench/torque_scale", 500)

        self.remote = RemoteControl("emergency", "/wrench/emergency")
        rospy.Subscriber("joy_emergency", Joy, self.joy_recieved)

        self.active = False
        self.reset()

    def reset(self) -> None:
        """
        Used to reset the state of the controller. Sometimes when it
        disconnects then comes back online, the settings are all out of whack.

        Args:
            No arguments are passed in.

        Returns:
            No return statement but values are being reset to their default state.
        """
        self.last_raise_kill = False
        self.last_clear_kill = False
        self.last_station_hold_state = False
        self.last_emergency_control = False
        self.last_go_inactive = False
        self.thruster_deploy_count = 0
        self.thruster_retract_count = 0

        self.start_count = 0
        self.last_joy = None
        self.active = False
        self.remote.clear_wrench()

    def check_for_timeout(self, joy: Joy):
        """
        This checks for a particular duration when the controller times out.

        Args:
            joy (Joy): The Joy message.
        """
        if self.last_joy is None:
            self.last_joy = joy
            return

        if joy.axes == self.last_joy.axes and joy.buttons == self.last_joy.buttons:
            # No change in state
            # The controller times out after 15 minutes
            if (
                rospy.Time.now() - self.last_joy.header.stamp > rospy.Duration(15 * 60)
                and self.active
            ):
                rospy.logwarn("Controller Timed out. Hold start to resume.")
                self.reset()

        else:
            joy.header.stamp = (
                rospy.Time.now()
            )  # In the sim, stamps weren't working right
            self.last_joy = joy

    def joy_recieved(self, joy: Joy) -> None:
        """
        Button elements are being assigned and simplied to readable names. The
        number of deployments or retractions for thrusters are being updated based
        on several conditions. Moreover, additional settings are changed based on the
        state of the controller and the activation of potential alarms or switches.

        Args:
            joy (Joy): The Joy message.
        """
        self.last_time = rospy.Time.now()
        self.check_for_timeout(joy)

        # Assigns readable names to the buttons that are used
        start = joy.buttons[7]
        go_inactive = joy.buttons[6]
        raise_kill = bool(joy.buttons[1])
        clear_kill = bool(joy.buttons[2])
        station_hold = bool(joy.buttons[0])
        emergency_control = bool(joy.buttons[13])
        thruster_retract = bool(joy.buttons[4])
        thruster_deploy = bool(joy.buttons[5])

        if go_inactive and not self.last_go_inactive:
            rospy.loginfo("Go inactive pressed. Going inactive")
            self.reset()
            return

        # Reset controller state if only start is pressed down about 1 seconds
        self.start_count += start
        if self.start_count > 5:
            rospy.loginfo("Resetting controller state")
            self.reset()
            self.active = True

        if not self.active:
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

        if emergency_control and not self.last_emergency_control:
            self.remote.select_emergency_control()

        self.last_raise_go_inactive = go_inactive
        self.last_raise_kill = raise_kill
        self.last_clear_kill = clear_kill
        self.last_station_hold_state = station_hold
        self.last_emergency_control = emergency_control

        # Scale joystick input to force and publish a wrench
        x = joy.axes[1] * self.force_scale
        y = joy.axes[0] * self.force_scale
        rotation = joy.axes[3] * self.torque_scale
        self.remote.publish_wrench(x, y, rotation, joy.header.stamp)

    def die_check(self, _: rospy.timer.TimerEvent) -> None:
        """
        Publishes zeros after 2 seconds of no update in case node dies.
        """
        # No new instructions after 2 seconds
        if self.active and rospy.Time.now() - self.last_time > rospy.Duration(2):
            # Zero the wrench, reset
            self.reset()


if __name__ == "__main__":
    rospy.init_node("emergency")

    emergency = Joystick()
    rospy.Timer(rospy.Duration(1), emergency.die_check, oneshot=False)
    rospy.spin()
