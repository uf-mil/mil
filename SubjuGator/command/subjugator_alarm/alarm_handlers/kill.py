import os
from threading import Condition

import rospy
from actionlib import SimpleActionClient, TerminalState
from mil_msgs.msg import BagOnlineAction, BagOnlineGoal
from ros_alarms import Alarm, HandlerBase
from std_srvs.srv import SetBool


class Kill(HandlerBase):
    """
    Generic kill used to represent the state of an alarm. Uses a threading condition
    to manage access

    Attributes:
        alarm_name (str): The name of the alarm. Set to ``kill``.
        condition (threading.Condition): The threading condition used to manage
          the shared access to the motherboard.
        HARDWARE_KILL_GRACE_PERIOD_SECONDS (int): The number of seconds to wait
          for the hardware kill to be removed after asking for it to be removed.
          If the kill has not been removed within this amount of time, the motherboard
          kill is reinstated. Set to ``6 seconds``.
    """

    alarm_name = "kill"
    HARDWARE_KILL_GRACE_PERIOD_SECONDS = 6.0

    def __init__(self):
        # Alarm server will set this as the initial state of kill alarm (starts killed)
        self.initial_alarm = Alarm(
            self.alarm_name,
            True,
            node_name="alarm_server",
            problem_description="Initial kill",
        )
        self._killed = True
        self.first = True
        self._last_mission_killed = False
        self.condition = Condition()
        self.bag_client = SimpleActionClient("/online_bagger/bag", BagOnlineAction)
        self._set_mobo_kill = rospy.ServiceProxy("/set_mobo_kill", SetBool)
        try:
            self._set_mobo_kill.wait_for_service(3.0)
        except rospy.ROSException:
            rospy.logerr("Could not contact kill board! Kills will only be software")
        self.set_mobo_kill(True)  # Tell HW that we started off as killed

    def set_mobo_kill(self, *args, **kwargs):
        """
        Sets a motherboard kill.

        Raises:
            rospy.ServiceException: Error in sending a motherboard kill.
        """
        try:
            return self._set_mobo_kill(*args, **kwargs)
        except rospy.ServiceException as e:
            rospy.logwarn(f"Error sending motherboard kill: {e}")
            return None

    def raised(self, alarm: Alarm) -> None:
        """
        Called when the alarm is raised. Sets the motherboard kill and attempts to
        dump the related logging info to a bag file, using the :attr:`~.condition`.
        At the end of the use of the condition, the condition is notified.

        Parameters:
            alarm (ros_alarms.Alarm): The alarm which was raised.
        """
        with self.condition:
            # Send kill command to kill board when alarm is raised
            self.set_mobo_kill(True)
            if not self._killed:
                self._killed = True
                self.bagger_dump()
            self.first = False
            self.condition.notify()

    def cleared(self, alarm: Alarm):
        """
        Called when the alarm is cleared. Attempts to use `~.condition` to unset
        the motherboard kill. If the motherboard kill can't be deactivated, then
        it is reinstated and a warning is logged.

        Parameters:
            alarm (ros_alarms.Alarm): The alarm which was cleared.
        """
        with self.condition:
            self.set_mobo_kill(False)
            if self.get_alarm("hw-kill").raised:
                self.condition.wait(self.HARDWARE_KILL_GRACE_PERIOD_SECONDS)
                if self.get_alarm("hw-kill").raised:
                    rospy.logwarn("Attempted to clear kill but hw-kill is still raised")
                    self.set_mobo_kill(True)
                    return False
                self._killed = False

    def _bag_done_cb(self, status, result):
        if status == 3:
            rospy.loginfo(f"KILL BAG WRITTEN TO {result.filename}")
        else:
            rospy.logwarn(
                f"KILL BAG {TerminalState.to_string(status)}, status: {result.status}",
            )

    def bagger_dump(self):
        """
        Bags the related log information to a list of relevant topics.
        """
        if self.first:
            return
        if "BAG_ALWAYS" not in os.environ or "bag_kill" not in os.environ:
            rospy.logwarn("BAG_ALWAYS or BAG_KILL not set. Not making kill bag.")
            return
        goal = BagOnlineGoal(bag_name="kill.bag")
        goal.topics = os.environ["BAG_ALWAYS"] + " " + os.environ["BAG_KILL"]
        self.bag_client.send_goal(goal, done_cb=self._bag_done_cb)

    def meta_predicate(self, meta_alarm, sub_alarms):
        ignore = []

        with self.condition:
            if not sub_alarms["hw-kill"].raised:
                self.condition.notify()

        # Stay killed until user clears
        if self._killed:
            return True

        # Battery too low
        if sub_alarms["bus-voltage"].raised and sub_alarms["bus-voltage"].severity == 5:
            return True
        ignore.append("bus-voltage")

        if sub_alarms["odom-kill"].raised and sub_alarms["odom-kill"].severity == 5:
            return True
        ignore.append("odom-kill")

        # If we lose network but don't want to go autonomous
        if sub_alarms["network-loss"].raised and not rospy.get_param(
            "/autonomous",
            False,
        ):
            return True
        ignore.append("network-loss")

        # Severity level of 3 means too many thrusters out (3 thrusters out)
        if (
            sub_alarms["thruster-out"].raised
            and sub_alarms["thruster-out"].severity == 3
        ):
            return True
        ignore.append("thruster-out")

        # If a mission wants us to kill, go ahead and kill
        if sub_alarms["mission-kill"].raised:
            self._last_mission_killed = True
            return True
        elif self._last_mission_killed:
            self._last_mission_killed = False

            # If we weren't killed by another source, clear the kill
            if not self._killed:
                return False
        ignore.append("mission-kill")

        # Raised if any alarms besides the two above are raised
        return any(
            alarm.raised for name, alarm in sub_alarms.items() if name not in ignore
        )
