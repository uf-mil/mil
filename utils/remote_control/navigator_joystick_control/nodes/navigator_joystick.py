#!/usr/bin/env python


import itertools

from geometry_msgs.msg import WrenchStamped
from kill_handling.broadcaster import KillBroadcaster
from nav_msgs.msg import Odometry
from navigator_alarm import AlarmBroadcaster
from navigator_msgs.srv import WrenchSelect, ShooterManual, ShooterManualRequest
from navigator_msgs.msg import ShooterDoAction, ShooterDoActionGoal
import rospy
import actionlib
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool, SetBoolRequest, Trigger, TriggerRequest

rospy.init_node("joystick")


class Joystick(object):

    def __init__(self):

        self.force_scale = rospy.get_param("~force_scale", 600)
        self.torque_scale = rospy.get_param("~torque_scale", 500)

        self.wrench_choices = itertools.cycle(['rc', 'autonomous'])
        self.current_pose = Odometry()

        self.active = False

        self.alarm_broadcaster = AlarmBroadcaster()
        self.kill_alarm = self.alarm_broadcaster.add_alarm(
            name='kill',
            action_required=True,
            severity=0
        )

        self.station_hold = self.alarm_broadcaster.add_alarm(
            name='station_hold',
            action_required=False,
            severity=3
        )

        # self.docking_alarm = self.alarm_broadcaster.add_alarm(
        #     name='docking',
        #     action_required=True,
        #     severity=0
        # )

        self.wrench_pub = rospy.Publisher("/wrench/rc", WrenchStamped, queue_size=1)
        # rospy.wait_for_service('/change_wrench')
        self.wrench_changer = rospy.ServiceProxy('/change_wrench', WrenchSelect)

        self.shooter_fire_client = actionlib.SimpleActionClient('/shooter/fire', ShooterDoAction)
        self.shooter_load_client = actionlib.SimpleActionClient('/shooter/load', ShooterDoAction)
        self.shooter_cancel_client = rospy.ServiceProxy('/shooter/cancel', Trigger)

        rospy.Subscriber("joy", Joy, self.joy_cb)
        self.reset()

    def reset(self):
        '''
        Used to reset the state of the controller.
        Sometimes when it disconnects then comes back online, the settings are all
            out of wack.
        '''
        self.last_change_mode = False
        self.last_station_hold_state = False
        self.last_kill = False
        self.last_rc = False
        self.last_auto = False
        self.last_shooter_shoot = False
        self.last_shooter_cancel = False
        self.last_shooter_load = False

        self.start_count = 0
        self.last_joy = None
        self.active = False

        self.killed = False
        # self.docking = False

        wrench = WrenchStamped()
        wrench.header.frame_id = "/base_link"
        wrench.wrench.force.x = 0
        wrench.wrench.force.y = 0
        wrench.wrench.torque.z = 0
        self.wrench_pub.publish(wrench)

    def check_for_timeout(self, joy):
        if self.last_joy is None:
            self.last_joy = joy
            return

        if joy.axes == self.last_joy.axes and \
           joy.buttons == self.last_joy.buttons:
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

    def joy_cb(self, joy):
        self.check_for_timeout(joy)

        # Handle Button presses
        start = joy.buttons[7]
        change_mode = bool(joy.buttons[3])  # Y
        kill = bool(joy.buttons[2])  # X
        station_hold = bool(joy.buttons[0])  # A
        # docking = bool(joy.buttons[1])  # B
        rc_control = bool(joy.buttons[11])  # d-pad left
        auto_control = bool(joy.buttons[12])  # d-pad right
        shooter_shoot = joy.axes[5] < -0.9
        shooter_load = bool(joy.buttons[4])
        shooter_cancel = bool(joy.buttons[5])

        # Reset controller state if only start is pressed down for awhile
        self.start_count += start
        if self.start_count > 10:  # About 3 seconds
            rospy.loginfo("Resetting controller state.")
            self.reset()
            self.active = True

            self.kill_alarm.clear_alarm()
            self.wrench_changer("rc")

        if not self.active:
            return

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
            self.station_hold.raise_alarm()
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

        # # Turn on docking mode
        # if docking == 1 and docking != self.last_docking_state:
        #     rospy.loginfo("Toggling Docking")

        #     if self.docking:
        #         self.docking_alarm.clear_alarm()
        #     else:
        #         self.docking_alarm.raise_alarm(
        #             problem_description='Docking kill from location: joystick'
        #         )

        #     self.docking = not self.docking

        if shooter_shoot and not self.last_shooter_shoot:
            rospy.loginfo("Joystick input : Shoot")
            self.shooter_fire_client.send_goal(goal=ShooterDoActionGoal(), done_cb=self.shooter_fire_cb)
        if shooter_load and not self.last_shooter_load:
            rospy.loginfo("Joystick input : Load")
            self.shooter_load_client.send_goal(goal=ShooterDoActionGoal(), done_cb=self.shooter_load_cb)
        if shooter_cancel and not self.last_shooter_cancel:
            rospy.loginfo("Joystick input : Cancel")
            self.shooter_cancel_client(TriggerRequest())

        self.last_start = start
        self.last_change_mode = change_mode
        self.last_kill = kill
        self.last_station_hold_state = station_hold
        # self.last_docking_state = docking
        self.last_auto_control = auto_control
        self.last_rc = rc_control
        self.last_auto = auto_control
        self.last_shooter_shoot = shooter_shoot
        self.last_shooter_cancel = shooter_cancel
        self.last_shooter_load = shooter_load

        # Handle joystick commands
        left_stick_x = joy.axes[1]
        left_stick_y = joy.axes[0]
        right_stick_y = joy.axes[3]

        wrench = WrenchStamped()
        wrench.header.frame_id = "/base_link"
        wrench.header.stamp = joy.header.stamp
        wrench.wrench.force.x = self.force_scale * left_stick_x
        wrench.wrench.force.y = self.force_scale * left_stick_y
        wrench.wrench.torque.z = self.torque_scale * right_stick_y
        self.wrench_pub.publish(wrench)


if __name__ == "__main__":
    joystick = Joystick()
    rospy.spin()
