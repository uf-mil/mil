from ros_alarms import HandlerBase, AlarmBroadcaster
from std_msgs.msg import Float32
import rospy


class BatteryVoltage(HandlerBase):
    alarm_name = 'battery-voltage'

    def __init__(self):
        self.broadcaster = AlarmBroadcaster(self.alarm_name)
        self.low_threshold = rospy.get_param('~battery-voltage/low')
        self.critical_threshold = rospy.get_param('~battery-voltage/critical')
        self.voltage_sub = rospy.Subscriber('/battery_monitor', Float32, self._check_voltage, queue_size=3)
        self._raised = False
        self._severity = 0

    def _check_voltage(self, msg):
        voltage = msg.data
        do_raise = voltage < self.low_threshold
        if do_raise:
            severity = 2 if voltage < self.critical_threshold else 1
            if not self._raised or self._severity != severity:
                self.broadcaster.raise_alarm(
                    severity=severity,
                    problem_description='battery critcaly low' if severity == 2 else 'battery low',
                    parameters={'voltage': voltage}
                )

    def raised(self, alarm):
        self._raised = True
        self._severity = alarm.severity

    def cleared(self, alarm):
        self._raised = False
        self._severity = alarm.severity
