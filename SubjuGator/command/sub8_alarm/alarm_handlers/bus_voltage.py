from ros_alarms import HandlerBase, Alarm
import subprocess


class BusVoltage(HandlerBase):
    """
    Alarm used to monitor the voltage of the bus.

    Attributes:
        alarm_name (str): The name of the alarm. For all alarms of this type, this
          is set equal to ``bus-voltage``.
    """
    alarm_name = 'bus-voltage'
    previous_severity = 0

    def __init__(self):
        self.initial_alarm = Alarm(self.alarm_name, False, node_name='alarm_server')

    def raised(self, alarm: Alarm) -> None:
        """
        If the severity of the raised alarm is 3, then an indication is raised to
        all logged-in users about a low bus voltage. If the raised alarm severity is
        5, then a message is displayed to all logged-in users about the bus being
        killed due to low voltage.

        Args:
            alarm (ros_alarms.Alarm): The raised alarm.
        """
        if alarm.severity == 3 and self.previous_severity != 3:
            subprocess.Popen('echo "Warning: bus voltage is low!" | wall', shell=True,
                             stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT)
            self.previous_severity = 3

        if alarm.severity == 5 and self.previous_severity != 5:
            subprocess.Popen('echo "Error: Killing due to low bus voltage" | wall', shell=True,
                             stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT)
            self.previous_severity = 5

    def cleared(self, alarm: Alarm) -> None:
        """
        Called when the alarm is cleared. Does nothing.
        """
        pass
