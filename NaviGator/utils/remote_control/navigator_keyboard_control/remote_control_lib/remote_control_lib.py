#!/usr/bin/env python3

"""
Remote Control Library: This object contains all of the common variables and
functions that are shared by the various remote control devices on NaviGator.
"""

import functools
import itertools
from typing import Callable, Optional

import actionlib
import genpy
import rospy
from actionlib import TerminalState
from geometry_msgs.msg import WrenchStamped
from mil_missions_core import MissionClient
from navigator_msgs.msg import ShooterDoAction, ShooterDoActionGoal
from navigator_msgs.srv import ShooterManual
from ros_alarms import AlarmBroadcaster, AlarmListener
from std_srvs.srv import Trigger, TriggerRequest
from topic_tools.srv import MuxSelect

__author__ = "Anthony Olive"


class RemoteControl:
    """
    Helper class which assists in allowing the remote execution of several key
    robot services.
    """

    def __init__(self, controller_name: str, wrench_pub: Optional[str] = None):
        self.name = controller_name
        self.wrench_choices = itertools.cycle(
            ["rc", "emergency", "keyboard", "autonomous"]
        )

        self.kill_broadcaster = AlarmBroadcaster("kill")
        self.station_hold_broadcaster = AlarmBroadcaster("station-hold")

        self.wrench_changer = rospy.ServiceProxy("/wrench/select", MuxSelect)
        self.task_client = MissionClient()
        self.kill_listener = AlarmListener(
            "kill", callback_funct=self._update_kill_status
        )

        if wrench_pub is None:
            self.wrench_pub = wrench_pub
        else:
            self.wrench_pub = rospy.Publisher(wrench_pub, WrenchStamped, queue_size=1)

        self.shooter_load_client = actionlib.SimpleActionClient(
            "/shooter/load", ShooterDoAction
        )
        self.shooter_fire_client = actionlib.SimpleActionClient(
            "/shooter/fire", ShooterDoAction
        )
        self.shooter_cancel_client = rospy.ServiceProxy("/shooter/cancel", Trigger)
        self.shooter_manual_client = rospy.ServiceProxy(
            "/shooter/manual", ShooterManual
        )
        self.shooter_reset_client = rospy.ServiceProxy("/shooter/reset", Trigger)

        self.is_killed = False
        self.is_timed_out = False
        self.clear_wrench()

    def _timeout_check(function: Callable):
        """
        Simple decorator to check whether or not the remote control device is
        timed out before running the function that was called.
        """

        @functools.wraps(function)
        def wrapper(self, *args, **kwargs):
            if not self.is_timed_out:
                return function(self, *args, **kwargs)

        return wrapper

    def _update_kill_status(self, alarm):
        """
        Updates the kill status display when there is an update on the kill
        alarm.
        """
        self.is_killed = alarm.raised

    @_timeout_check
    def kill(self, *args, **kwargs) -> None:
        """
        Kills the system regardless of what state it is in.
        """
        rospy.loginfo("Killing")
        self.kill_broadcaster.raise_alarm(
            problem_description="System kill from user remote control",
            parameters={"location": self.name},
        )

    @_timeout_check
    def clear_kill(self, *args, **kwargs) -> None:
        """
        Clears the system kill regardless of what state it is in.
        """
        rospy.loginfo("Reviving")
        self.kill_broadcaster.clear_alarm()

    @_timeout_check
    def toggle_kill(self, *args, **kwargs) -> None:
        """
        Toggles the kill status when the toggle_kill_button is pressed.
        """
        rospy.loginfo("Toggling Kill")

        # Responds to the kill broadcaster and checks the status of the kill alarm
        if self.is_killed:
            self.kill_broadcaster.clear_alarm()
        else:
            self.kill_broadcaster.raise_alarm(
                problem_description="System kill from user remote control",
                parameters={"location": self.name},
            )

    @_timeout_check
    def station_hold(self, *args, **kwargs) -> None:
        """
        Sets the goal point to the current location and switches to autonomous
        mode in order to stay at that point.
        """
        rospy.loginfo("Station Holding")

        # Trigger station holding at the current pose
        self.station_hold_broadcaster.raise_alarm(
            problem_description="Request to station hold from remote control",
            parameters={"location": self.name},
        )

    @_timeout_check
    def deploy_thrusters(self, *args, **kwargs) -> None:
        """
        Deploys the thrusters by creating a new task in the task client.
        """

        def cb(terminal_state, result):
            if terminal_state == 3:
                rospy.loginfo("Thrusters Deployed!")
            else:
                rospy.logwarn(
                    "Error deploying thrusters: {}, status: {}".format(
                        TerminalState.to_string(terminal_state), result.status
                    )
                )

        self.task_client.run_task("DeployThrusters", done_cb=cb)

    @_timeout_check
    def retract_thrusters(self, *args, **kwargs) -> None:
        """
        Retracts the thrusters by creating a new task in the task client.
        """

        def cb(terminal_state, result):
            if terminal_state == 3:
                rospy.loginfo("Thrusters Retracted!")
            else:
                rospy.logwarn(
                    "Error rectracting thrusters: {}, status: {}".format(
                        TerminalState.to_string(terminal_state), result.status
                    )
                )

        self.task_client.run_task("RetractThrusters", done_cb=cb)

    @_timeout_check
    def select_autonomous_control(self, *args, **kwargs) -> None:
        """
        Selects the autonomously generated trajectory as the active controller.
        """
        rospy.loginfo("Changing Control to Autonomous")
        self.wrench_changer("autonomous")

    @_timeout_check
    def select_rc_control(self, *args, **kwargs) -> None:
        """
        Selects the Xbox remote joystick as the active controller.
        """
        rospy.loginfo("Changing Control to RC")
        self.wrench_changer("rc")

    def select_emergency_control(self, *args, **kwargs) -> None:
        """
        Selects the emergency controller as the active controller.
        """
        rospy.loginfo("Changing Control to Emergency Controller")
        self.wrench_changer("emergency")

    @_timeout_check
    def select_keyboard_control(self, *args, **kwargs) -> None:
        """
        Selects the keyboard teleoperation service as the active controller.
        """
        rospy.loginfo("Changing Control to Keyboard")
        self.wrench_changer("keyboard")

    @_timeout_check
    def select_next_control(self, *args, **kwargs) -> None:
        """
        Selects the autonomously generated trajectory as the active controller.
        """
        mode = next(self.wrench_choices)
        rospy.loginfo(f"Changing Control Mode: {mode}")
        self.wrench_changer(mode)

    def _shooter_load_feedback(self, status, result):
        """
        Prints the feedback that is returned by the shooter load action client
        """
        rospy.loginfo(
            "Shooter Load Status={} Success={} Error={}".format(
                status, result.success, result.error
            )
        )

    @_timeout_check
    def shooter_load(self, *args, **kwargs) -> None:
        """
        Loads the shooter by using the action client to retract the linear actuator.
        """
        self.shooter_load_client.send_goal(
            goal=ShooterDoActionGoal(), done_cb=self._shooter_load_feedback
        )
        rospy.loginfo("Kip, do not throw away your shot.")

    def _shooter_fire_feedback(self, status, result) -> None:
        """
        Prints the feedback that is returned by the shooter fire action client
        """
        rospy.loginfo(
            "Shooter Fire Status={} Success={} Error={}".format(
                status, result.success, result.error
            )
        )

    @_timeout_check
    def shooter_fire(self, *args, **kwargs) -> None:
        """
        Fires the shooter by using the action client to spin up the
        acceleration discs and extend the linear actuator.
        """
        self.shooter_fire_client.send_goal(
            goal=ShooterDoActionGoal(), done_cb=self._shooter_fire_feedback
        )
        rospy.loginfo(
            "One, two, three, four, five, six, seven, eight, nine. Number... TEN PACES! FIRE!"
        )

    @_timeout_check
    def shooter_cancel(self, *args, **kwargs) -> None:
        """
        Cancels the process that the shooter action client is currently
        running.
        """
        rospy.loginfo("Canceling shooter requests")
        self.shooter_cancel_client(TriggerRequest())
        rospy.loginfo("I imaging death so much it feels more like a memory.")
        rospy.loginfo(
            "When's it gonna get me? While I'm blocked? Seven clocks ahead of me?"
        )

    def _shooter_reset_helper(self, event):
        """
        Used to actually call the shooter's reset service.
        """
        rospy.loginfo("Resetting the shooter service")
        self.shooter_reset_client(TriggerRequest())
        rospy.loginfo(
            "In New York you can be a new man! In New York you can be a new man!"
        )

    @_timeout_check
    def shooter_reset(self, *args, **kwargs) -> None:
        """
        Ensures that the shooter is fully retracted by initiating a retract and
        using a ~6s delay before calling the actual reset service.
        """
        self.shooter_linear_retract()
        rospy.Timer(rospy.Duration(6), self._shooter_reset_helper, oneshot=True)

    @_timeout_check
    def shooter_linear_extend(self, *args, **kwargs):
        """
        Extends the shooter's linear actuator by setting it's speed to full
        forward.
        """
        rospy.loginfo("Extending the shooter's linear actuator")
        self.shooter_manual_client(1, 0)

    @_timeout_check
    def shooter_linear_retract(self, *args, **kwargs):
        """
        Retracts the shooter's linear actuator by setting it's speed to full
        reverse.
        """
        rospy.loginfo("Retracting the shooter's linear actuator")
        self.shooter_manual_client(-1, 0)

    @_timeout_check
    def set_disc_speed(self, speed: int, *args, **kwargs) -> None:
        """
        Sets the shooters disc speed to the speed value passed in. The value is
        a percentage from -100 to 100, which is scaled down to a number from -1
        to 1.

        Args:
            speed (int): The speed to set the shooter disc to.
        """
        rospy.loginfo(f"Setting the shooter's accelerator disc speed to {speed}")
        self.shooter_manual_client(0, float(speed) / -100)

    @_timeout_check
    def publish_wrench(
        self,
        x: float,
        y: float,
        rotation: float,
        stamp: Optional[genpy.rostime.Time] = None,
        *args,
        **kwargs,
    ) -> None:
        """
        Publishes a wrench to the specified node based on force inputs from the
        controller.

        Args:
            x (float): The x-dimension of the wrench's force vector.
            y (float): The y-dimension of the wrench's force vector.
            rotation (float): The z-dimension of torque.
            stamp (Optional[genpy.rostime.Time]): The time to attach to the header.
                If ``None``, then use the current time.
        """
        if stamp is None:
            stamp = rospy.Time.now()

        if self.wrench_pub is not None:
            wrench = WrenchStamped()
            wrench.header.frame_id = "/base_link"
            wrench.header.stamp = stamp
            wrench.wrench.force.x = x
            wrench.wrench.force.y = y
            wrench.wrench.torque.z = rotation
            self.wrench_pub.publish(wrench)

    @_timeout_check
    def clear_wrench(self, *args, **kwargs) -> None:
        """
        Publishes a zeroed wrench to the specified node based on force inputs from the
        controller.
        """
        if self.wrench_pub is not None:
            wrench = WrenchStamped()
            wrench.header.frame_id = "/base_link"
            wrench.header.stamp = rospy.Time.now()
            wrench.wrench.force.x = 0
            wrench.wrench.force.y = 0
            wrench.wrench.torque.z = 0
            self.wrench_pub.publish(wrench)
