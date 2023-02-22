#!/usr/bin/env python3

"""
Battery Monitor: A simple script that approximates the battery voltage by
averaging the supply voltage to each of the four thrusters.
"""


import message_filters
import rospy
from roboteq_msgs.msg import Feedback, Status
from ros_alarms import AlarmListener
from ros_alarms.msg import Alarm
from std_msgs.msg import Float32

__author__ = "Anthony Olive"
__maintainer__ = "Kevin Allen"
__email__ = "kma1660@gmail.com"
__copyright__ = "Copyright 2016, MIL"
__license__ = "MIT"


class BatteryMonitor:
    """
    Monitors the battery voltage measured by the 4 motors on Navigator,
    publishing a moving average of these measurements to /battery_monitor
    at a regular interval.

    Attributes:
        voltage (Optional[float]): The average voltage across all of the four motors.
            ``None`` if no reading has occurred yet.
        supply_voltages (List[float]): The most recent 1000 voltage readings.
            Each reading is from a separate motor.
        hw_kill_raised (bool): Whether a hardware kill was raised.
        pub_voltage (rospy.Publisher): A publisher to the ``/battery_monitor``
            topic. Published by the :meth:`.publish_voltage` method.
    """

    def __init__(self):
        # Initialize the average voltage to none for the case of no feedback
        self.voltage = None

        # Initialize a list to hold the 1000 most recent supply voltage readings
        # Holding 1000 values ensures that changes in the average are gradual rather than sharp
        self.supply_voltages = []

        self.hw_kill_raised = None
        self._hw_kill_listener = AlarmListener("hw-kill", self.hw_kill_cb)
        self._hw_kill_listener.wait_for_server()

        # The publisher for the averaged voltage
        self.pub_voltage = rospy.Publisher("/battery_monitor", Float32, queue_size=1)

        # Subscribes to the feedback from each of the four thrusters
        motor_topics = ["/FL_motor", "/FR_motor", "/BL_motor", "/BR_motor"]

        feedback_sub = [
            message_filters.Subscriber(topic + "/feedback", Feedback)
            for topic in motor_topics
        ]

        status_sub = [
            message_filters.Subscriber(topic + "/status", Status)
            for topic in motor_topics
        ]

        [
            message_filters.ApproximateTimeSynchronizer(
                [feedback, status], 1, 10
            ).registerCallback(self.add_voltage)
            for feedback, status in zip(feedback_sub, status_sub)
        ]

    def hw_kill_cb(self, msg: Alarm):
        """
        Serves as a callback when a hardware kill is activated.

        Args:
            msg (~ros_alarms.msg._Alarm.Alarm): The alarm message that activated
                the hardware kill.
        """
        self.hw_kill_raised = msg.raised

    def add_voltage(self, feedback: Feedback, status: Status) -> None:
        """
        This is the callback function for feedback from all four motors.

        It appends the new readings to the end of the list and
        ensures that the list stays under 1000 entries.

        Args:
            feedback (~roboteq_msgs.msg._Feedback.Feedback): The feedback from
                one of the motors.
            status (~roboteq_msgs.msg._Status.Status): The status of the motor.
        """
        # Check if 3rd bit is raised
        if status.fault & 4 == 4 or self.hw_kill_raised is True:
            return
        self.supply_voltages.append(feedback.supply_voltage)

        # Limits the list by removing the oldest readings when it contains more then 1000 readings
        while len(self.supply_voltages) > 1000:
            del self.supply_voltages[0]

    def publish_voltage(self, _) -> None:
        """
        Publishes the average voltage across all four thrusters to the ``battery_voltage`` node
        as a standard Float32 message and runs the voltage_check.

        Currently runs every second.
        """
        if len(self.supply_voltages) > 0:
            self.voltage = sum(self.supply_voltages) / len(self.supply_voltages)
            self.pub_voltage.publish(self.voltage)


if __name__ == "__main__":
    rospy.init_node("battery_monitor")
    monitor = BatteryMonitor()
    rospy.Timer(rospy.Duration(1), monitor.publish_voltage, oneshot=False)
    rospy.spin()
