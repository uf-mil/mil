import rospy
from mil_msgs.msg import RangeStamped
from ros_alarms import Alarm, AlarmBroadcaster, HandlerBase


class HeightOverBottom(HandlerBase):
    """
    Alarm (inheriting from :class:`ros_alarms.HandlerBase`) which monitors the height of the
    sub in the water. If too low, then an alarm is triggered. When the sub rises
    above the limit again, the alarm is cleared.

    The height to kill at is obtained from Dynamic Reconfigure, or is assumed to be
    ``0.4``. The height of the sub is checked at 2Hz.

    Attributes:
        alarm_name (str): The name of the alarm. Set to ``height-over-bottom``.
    """

    alarm_name = "height-over-bottom"

    def __init__(self):
        self._killed = False
        self._update_height()

        self.ab = AlarmBroadcaster(self.alarm_name, node_name="height_over_bottom_kill")

        # Keep track of the current height
        self._last_height = 100
        set_last_height = lambda msg: setattr(self, "_last_height", msg.range)
        rospy.Subscriber("/dvl/range", RangeStamped, set_last_height)

        # Every 5 seconds, check for an updated height param. A pseudo dynamic reconfig thing.
        rospy.Timer(rospy.Duration(5), self._update_height)

        # This should smooth out random dips below the limit
        rospy.Timer(rospy.Duration(0.5), self._do_check)

    def _do_check(self, *args):
        if self._last_height <= self._height_to_kill and not self._killed:
            rospy.logwarn("SUB TOO LOW!")
            self.ab.raise_alarm(
                problem_description=f"The sub was too low: {self._last_height}",
                parameters={"height": self._last_height},
                severity=5,
            )
        elif self._last_height >= self._height_to_kill and self._killed:
            rospy.logwarn("REVIVING")
            self.ab.clear_alarm()

    def _update_height(self, *args):
        self._height_to_kill = rospy.get_param("/height_over_bottom", 0.4)

    def raised(self, alarm: Alarm) -> None:
        """
        Triggers when the alarm is raised. Sets the state of the alarm monitor to
        represent that the alarm was killed.

        Parameters:
            alarm (ros_alarms.Alarm): The alarm which was raised.
        """
        self._killed = True

    def cleared(self, alarm: Alarm) -> None:
        """
        Triggers when the alarm is cleared. Sets the state of the alarm monitor to

        Parameters:
            alarm (ros_alarms.Alarm): The alarm which was cleared.
        """
        self._killed = False
