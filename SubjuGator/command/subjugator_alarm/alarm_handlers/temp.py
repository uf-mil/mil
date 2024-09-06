import json
from contextlib import nullcontext
from unittest import mock

import rospy
from ros_alarms import Alarm, AlarmBroadcaster, HandlerBase
from std_msgs.msg import String


class Temperature(HandlerBase):
    """
    Alarm (inheriting from :class:`ros_alarms.HandlerBase`) which monitors the temperature
    of a system.

    Attributes:
        alarm_name (str): The name of the alarm. Set to ``temp``.
    """

    alarm_name = "temp"

    def __init__(self):
        self._killed = False
        self.ab = AlarmBroadcaster(self.alarm_name, node_name="temp_kill")
        self.in_test = rospy.get_param("/subjugator_alarm_temp_test_flag", False)
        self.limit_amounts = {}
        if self.in_test:

            def _set_test_values(call: String):
                self.limit_amounts = json.loads(call.data)

            self.test_values_sub = rospy.Subscriber(
                "/subjugator_alarm_temp_test_values",
                String,
                _set_test_values,
            )
        rospy.Timer(rospy.Duration(0.5), self._temp_check)

    def _temp_check(self, *args):
        # Only run if the user has psutil installed
        try:
            import psutil
        except ModuleNotFoundError:
            return
        tv = self.limit_amounts.copy()
        with (
            mock.patch(
                "psutil._psplatform.sensors_temperatures",
                return_value=tv,
            )
            if self.in_test
            else nullcontext()
        ):
            temps = psutil.sensors_temperatures()
            any_high_or_crit = False
            for name, entries in temps.items():
                for entry in entries:
                    high = (
                        entry.current > (entry.high) if entry.high is not None else None
                    )
                    crit = entry.critical and entry.current > (0.9 * entry.critical)
                    if not self._killed and high:
                        rospy.logwarn(
                            f"Temperature of a sensor ('{name}') is quite high...",
                        )
                        self.ab.raise_alarm(
                            problem_description=f"Temperature of '{name}' too high: {entry.current}*C, high: {entry.critical}*C",
                            parameters={"Temperature": entry.current},
                            severity=5,
                        )
                    elif not self._killed and crit:
                        rospy.logwarn(
                            f"Temperature of a sensor ('{name}') is critical...",
                        )
                        self.ab.raise_alarm(
                            problem_description=f"Temperature of '{name}' too high: {entry.current}*C, critical: {entry.critical}*C",
                            parameters={"Temperature": entry.current},
                            severity=5,
                        )
                    # was not previously killed
                    if crit or high:
                        any_high_or_crit = True
            if self._killed and not any_high_or_crit:
                rospy.logwarn("No temperatures are high enough, unkilling...")
                self.ab.clear_alarm()

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
