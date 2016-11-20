#!/usr/bin/env python

'''
Battery Monitor: A simple script that approximates the battery voltage by
averaging the supply voltage to each of the four thrusters.
'''


from __future__ import division

from navigator_alarm import AlarmBroadcaster
from roboteq_msgs.msg import Feedback
import rospy
from std_msgs.msg import Float32


__author__ = "Anthony Olive"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2016, MIL"
__license__ = "MIT"


rospy.init_node("battery_monitor")


class BatteryMonitor():

    def __init__(self):

        # Initialize the average voltage to none for the case of no feedback
        self.voltage = None

        # Initialize a list to hold the 1000 most recent supply voltage readings
        # Holding 1000 values ensures that changes in the average are gradual rather than sharp
        self.supply_voltages = []

        # The publisher for the averaged voltage
        self.pub_voltage = rospy.Publisher("/battery_monitor", Float32, queue_size=1)

        # Subscribes to the feedback from each of the four thrusters
        rospy.Subscriber("/FL_motor/feedback", Feedback, self.add_voltage)
        rospy.Subscriber("/FR_motor/feedback", Feedback, self.add_voltage)
        rospy.Subscriber("/BL_motor/feedback", Feedback, self.add_voltage)
        rospy.Subscriber("/BR_motor/feedback", Feedback, self.add_voltage)

        # Attempts to read the battery voltage parameters (sets them to defaults if they have not been set)
        self.battery_low_voltage = rospy.get_param("~battery_low_voltage", 22.1)
        self.battery_critical_voltage = rospy.get_param("~battery_critical_voltage", 20.6)
        self.battery_kill_voltage = rospy.get_param("~battery_kill_voltage", 19.5)

        # Sets up the battery voltage alarms
        alarm_broadcaster = AlarmBroadcaster()
        self.battery_status_unknown_alarm = alarm_broadcaster.add_alarm(
            name="battery_status_unknown",
            action_required=True,
            severity=2
        )
        self.battery_low_alarm = alarm_broadcaster.add_alarm(
            name="battery_low",
            action_required=False,
            severity=2
        )
        self.battery_critical_alarm = alarm_broadcaster.add_alarm(
            name="battery_critical",
            action_required=True,
            severity=1
        )
        self.battery_kill_alarm = alarm_broadcaster.add_alarm(
            name="kill",
            action_required=True,
            severity=0
        )

    def add_voltage(self, msg):
        '''
        This is the callback function for feedback from all four motors. It appends the new readings to the end of the list and
        ensures that the list stays under 1000 entries.
        '''

        self.supply_voltages.append(msg.supply_voltage)

        # Limits the list by removing the oldest readings when it contains more then 1000 readings
        while (len(self.supply_voltages) > 1000):
            del self.supply_voltages[0]

    def publish_voltage(self, event):
        '''
        Publishes the average voltage across all four thrustersto the battery_voltage node as a standard Float32 message and runs
        the voltage_check
        '''

        if (len(self.supply_voltages) > 0):
            self.voltage = sum(self.supply_voltages) / len(self.supply_voltages)

        self.pub_voltage.publish(self.voltage)
        self.voltage_check()

    def voltage_check(self):
        '''
        Checks the battery voltage and raises alarms when it's level drops below the set thresholds. This is intended to protect
        and extend the life of the batteries.
        '''

        # An unknown battery status warning to inform users that thruster feedback is not being published
        if (self.voltage is None):
            self.battery_status_unknown_alarm.raise_alarm(
                problem_description="Bus voltage is not available because thruster feedback is not being published",
                parameters={
                    "bus_voltage": "{}".format(self.voltage),
                }
            )

        # A fatal battery warning to kill the boat and protect the batteries
        elif (self.voltage < self.battery_kill_voltage):
            self.battery_kill_alarm.raise_alarm(
                problem_description="Bus voltage is at the safety limit; killing the system",
                parameters={
                    "bus_voltage": "{}".format(self.voltage),
                }
            )

        # A high priority battery warning to abort testing
        elif (self.voltage < self.battery_critical_voltage):
            self.battery_critical_alarm.raise_alarm(
                problem_description="Bus voltage is critical; abort this run soon",
                parameters={
                    "bus_voltage": "{}".format(self.voltage),
                }
            )

        # A low priority battery warning to inform user's of the status
        elif (self.voltage < self.battery_low_voltage):
            self.battery_low_alarm.raise_alarm(
                problem_description="Bus voltage is approaching safety limit",
                parameters={
                    "bus_voltage": "{}".format(self.voltage),
                }
            )


if __name__ == "__main__":
    monitor = BatteryMonitor()
    rospy.Timer(rospy.Duration(1), monitor.publish_voltage, oneshot=False)
    rospy.spin()
