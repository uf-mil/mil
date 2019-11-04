from ros_alarms import HandlerBase, Alarm
import subprocess


class BusVoltage(HandlerBase):
    alarm_name = 'bus-voltage'
    previous_severity = 0

    def __init__(self):
        self.initial_alarm = Alarm(self.alarm_name, False, node_name='alarm_server')

    def raised(self, alarm):
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

    def cleared(self, alarm):
        pass
