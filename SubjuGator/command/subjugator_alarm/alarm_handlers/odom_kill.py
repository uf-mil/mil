#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from ros_alarms import Alarm, AlarmBroadcaster, HandlerBase

__author__ = "David Soto"


class OdomKill(HandlerBase):
    """
    Will kill the sub when it stops hearing odometry messages or when there is a large
    discontinuity in state estimation. Only meant to run on the sub as a safety measure.

    Attributes:
        GRACE_PERIOD (rospy.Duration): The amount of time to wait after the class
          comes alive. Used in case odometry disappears just when the class is
          started. Defaults to ``5 seconds``.
        TIMEOUT (rospy.Duration): The amount of time to wait before an error is logged.
          Defaults to ``0.5 seconds``.
        MAX_JUMP (float): The amount of meters that the sub is allowed to jump
          before an error is logged. Defaults to ``0.5 meters``.
        LAUNCH_TIME (rospy.Time): When the class was started. Used to calculate
          when the grace period ends.
        ab (ros_alarms.AlarmBroadcaster): The alarm broadcaster for the node, with a name
          of ``odom-kill``.
    """

    def __init__(self, timeout=0.5):
        self.GRACE_PERIOD = rospy.Duration(
            5.0,
        )  # Alarms won't be raised within grace period
        self.TIMEOUT = rospy.Duration(timeout)
        self.MAX_JUMP = 0.5
        self.LAUNCH_TIME = rospy.Time.now()
        self.last_time = self.LAUNCH_TIME
        self.last_position = None
        self.check_count = 0
        self.odom_discontinuity = False
        self._killed = False
        self.odom_listener = rospy.Subscriber(
            "/odom",
            Odometry,
            self.got_odom_msg,
            queue_size=1,
        )
        self.ab = AlarmBroadcaster("odom-kill", node_name="odom-kill")
        rospy.Timer(rospy.Duration(0.1), self.check)

    def check(self, *args):
        """
        Checks to see if an alarm needs to be raised. Raises either an alarm describing
        that state estimation is not available (if the sub was just started) or
        that a significant amount of state estimation was lost since the sub was started.
        """
        self.check_count += 1
        if not self._killed and self.need_kill():
            if self.last_position is None:
                if self.check_count < 10:  # Probably just haven't received odom yet
                    pass
                else:  # Odom is probably not publishing
                    self._killed = True
                    self.ab.raise_alarm(
                        problem_description="STATE ESTIMATION NOT AVAILABLE: KILLING SUB",
                        severity=5,
                    )
                    rospy.logerr("STATE ESTIMATION NOT AVAILABLE: KILLING SUB")
            else:
                self._killed = True
                self.ab.raise_alarm(
                    problem_description="STATE ESTIMATION LOSS: KILLING SUB",
                    severity=5,
                )
                rospy.logerr("STATE ESTIMATION LOSS: KILLING SUB")

    def got_odom_msg(self, msg):
        if self.last_position is not None:
            self.check_continuity(msg)
        self.last_position = msg.pose.pose.position
        self.last_time = rospy.Time.now()

    def check_continuity(self, new_odom_msg: Odometry):  # True if 'continuous'
        """
        Checks the continuity of the odom system. Calculates the difference (in meters)
        between the current odom message and the previous message. If the sub moved
        more than ``0.5 meters``, a discontinuity is reported.

        Args:
            new_odom_msg (Odometry): The new odometry message to which the old will
              be compared to.
        """
        if self.odom_discontinuity:
            return

        this_p = new_odom_msg.pose.pose.position
        last_p = self.last_position
        jump = (
            (this_p.x - last_p.x) ** 2
            + (this_p.y - last_p.y) ** 2
            + (this_p.z - last_p.z) ** 2
        ) ** 0.5
        if jump > self.MAX_JUMP:
            rospy.logerr("ODOM DISCONTINUITY DETECTED")
            self.ab.raise_alarm(
                problem_description=f"ODOM DISCONTINUITY DETECTED JUMPED {jump} METERS",
                severity=5,
            )
            self.odom_discontinuity = True
            return False

    def need_kill(self):
        """
        Determine if a kill is needed. Checks to see if the odom was lost for a continuous
        amount of time or if there was a discontinuity in odom measurements.
        """
        now = rospy.Time.now()
        in_grace_period = now - self.LAUNCH_TIME < self.GRACE_PERIOD
        odom_loss = now - self.last_time > self.TIMEOUT and not in_grace_period
        if odom_loss:
            rospy.logerr_throttle(
                1,
                f"LOST ODOM FOR {(rospy.Time.now() - self.last_time).to_sec()} SECONDS",
            )
            self.ab.raise_alarm(
                problem_description="LOST ODOM FOR {} SECONDS".format(
                    (rospy.Time.now() - self.last_time).to_sec(),
                ),
                severity=5,
            )
        return odom_loss or self.odom_discontinuity

    def clear_kill(self, alarm: Alarm):
        """
        Clears the kill imposed on the class.

        Args:
            alarm (ros_alarms.Alarm): The alarm which is used to determine if a clear
              needs to occur.
        """
        msg = ""
        if alarm.clear:
            if self._killed:
                self._killed = False
                self.odom_discontinuity = False
                msg = "Odom kill successfully cleared!"
            else:
                msg = "Attempted to clear odom kill, but was already not killed."
            rospy.logwarn(msg)

    def raised(self, alarm: Alarm):
        """
        Runs when an alarm is raised. Sets the class' killed state to ``True``.
        """
        self._killed = True

    def cleared(self, alarm: Alarm):
        """
        Runs when an alarm is raised. Sets the class' killed state to ``False``.
        """
        self._killed = False
