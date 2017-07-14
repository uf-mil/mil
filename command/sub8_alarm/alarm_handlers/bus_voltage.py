import rospy
from ros_alarms import HandlerBase, Alarm
import json

class BusVoltage(HandlerBase):
    alarm_name = 'bus-voltage'

    def __init__(self):
        self.initial_alarm = Alarm(self.alarm_name, False, node_name='alarm_server')

    def raised(self, alarm):
        if alarm.severity == 3:
            subprocess.Popen('echo "Warning: bus voltage is low!" | wall', shell=True,
                             stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT)

        if alarm.severity == 5:
            subprocess.Popen('echo "Error: Killing due to low bus voltage" | wall', shell=True,
                             stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT)

    def cleared(self, alarm):
        pass
