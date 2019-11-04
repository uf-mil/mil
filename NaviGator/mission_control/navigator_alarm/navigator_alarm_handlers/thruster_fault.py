from ros_alarms import HandlerBase, AlarmBroadcaster
from roboteq_msgs.msg import Status
import rospy


class ThrusterFault(HandlerBase):
    alarm_name = 'thruster-fault'

    def __init__(self):
        self.broadcaster = AlarmBroadcaster(self.alarm_name)

        # Thruster topics to listen to
        motor_topics = ['/FL_motor', '/FR_motor', '/BL_motor', '/BR_motor']

        # Dictionary for the faults as defined in roboteq_msgs/Status
        self.fault_codes = {1: 'OVERHEAT', 2: 'OVERVOLTAGE', 4: 'UNDERVOLTAGE', 8: 'SHORT_CIRCUIT',
                            16: 'EMERGENCY_STOP', 32: 'SEPEX_EXCITATION_FAULT', 64: 'MOSFET_FAILURE',
                            128: 'STARTUP_CONFIG_FAULT'}

        self._raised = False
        self._raised_alarms = {}

        # Subscribe to the status message for all thruster topics
        [rospy.Subscriber(topic + '/status', Status, self._check_faults, topic) for topic in motor_topics]

    # Return a list that decodes the binary to strings
    def _get_fault_codes(self, fault_id):
        get_fault_codes = []
        for key, value in self.fault_codes.iteritems():
            # Fault message is the sum of binary strings
            decode = fault_id & key
            if decode != 0:
                get_fault_codes.append(value)
        return get_fault_codes

    def _check_faults(self, msg, topic):
        update = False

        # Check if there is a change
        if topic not in self._raised_alarms or self._raised_alarms[topic] != msg.fault:
            # Check if change is to no faults
            if msg.fault == 0:
                # If the topic is there, there delete it
                if topic in self._raised_alarms:
                    del self._raised_alarms[topic]
                    update = True
            # if not a no fault, then update
            else:
                self._raised_alarms[topic] = msg.fault
                update = True

        if update:
            if len(self._raised_alarms) == 0:
                self.broadcaster.clear_alarm()
                return
            self.broadcaster.raise_alarm(
                severity=5,
                problem_description='{} thrusters have faults'.format(
                    len(self._raised_alarms)),
                parameters=dict([(t, self._get_fault_codes(k))
                                 for t, k in self._raised_alarms.iteritems()])
            )

    def raised(self, alarm):
        pass

    def cleared(self, alarm):
        self._raised_alarms = {}
