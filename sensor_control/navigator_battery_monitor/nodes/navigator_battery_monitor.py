#!/usr/bin/env python

'''
Battery Monitor: A simple script that approximates the battery voltage by
averaging the supply voltage to each of the four thrusters.
'''


from __future__ import division

from roboteq_msgs.msg import Feedback
import rospy
from std_msgs.msg import Float32
from sub8_alarm import AlarmBroadcaster


__author__ = "Anthony Olive"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2016, MIL"
__license__ = "MIT"


# Voltage threshold for the initial low voltage alarm
# 22.1V (approximately 30%) is the recommended value
battery_low = 22.1

# Voltage threshold for the critically low voltage alarm
# 20.6V (approximately 15%) is the recommended value
battery_critical = 20.6

# Voltage threshold for the total system kill alarm
# 19.5V (approximately 5%) is the recommended value
battery_kill = 19.5

rospy.init_node('battery_monitor')


class BatteryMonitor():

    def __init__(self):

        # Initialize the average voltage to max
        self.voltage = None

        # Initialize a list to hold the 1000 most recent supply voltage readings
        # Holding 1000 values ensures that changes in the average are gradual rather than sharp
        self.supply_voltages = []

        # The publisher for the averaged voltage
        self.pub_voltage = rospy.Publisher("/battery_monitor", Float32, queue_size=1)

        # Subscrives to the feedback from each of the four thrusters
        rospy.Subscriber("/FL_motor/feedback", Feedback, self.add_voltage)
        rospy.Subscriber("/FR_motor/feedback", Feedback, self.add_voltage)
        rospy.Subscriber("/BL_motor/feedback", Feedback, self.add_voltage)
        rospy.Subscriber("/BR_motor/feedback", Feedback, self.add_voltage)

        # Sets up the battery voltage alarms
        alarm_broadcaster = AlarmBroadcaster()
        self.battery_low_alarm = alarm_broadcaster.add_alarm(
            name='battery_low',
            action_required=False,
            severity=2
        )
        self.battery_critical_alarm = alarm_broadcaster.add_alarm(
            name='battery_critical',
            action_required=True,
            severity=1
        )
        self.battery_kill_alarm = alarm_broadcaster.add_alarm(
            name='kill',
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
        if (self.voltage != None):
            if (len(self.supply_voltages) > 0):
                self.voltage = sum(self.supply_voltages) / len(self.supply_voltages)

            self.pub_voltage.publish(self.voltage)
            self.voltage_check()

    def voltage_check(self):
        '''
        Checks the battery voltage and raises alarms when it's level drops below the set thresholds. This is intended to protect
        and extend the life of the batteries.
        '''
        # A fatal battery warning to kill the boat and protect the batteries
        if (self.voltage < battery_kill):
            self.battery_kill_alarm.raise_alarm(
                problem_description='Bus voltage is at the safety limit; killing the system',
                parameters={
                    'bus_voltage': '{}'.format(self.voltage),
                }
            )

        # A high priority battery warning to abort testing
        elif (self.voltage < battery_critical):
            self.battery_critical_alarm.raise_alarm(
                problem_description='Bus voltage is critical; abort testing',
                parameters={
                    'bus_voltage': '{}'.format(self.voltage),
                }
            )

        # A low priority battery warning to inform user's of the status
        elif (self.voltage < battery_low):
            self.battery_low_alarm.raise_alarm(
                problem_description='Bus voltage is approaching safety limit',
                parameters={
                    'bus_voltage': '{}'.format(self.voltage),
                }
            )

if __name__ == "__main__":
    monitor = BatteryMonitor()
    rospy.Timer(rospy.Duration(1), monitor.publish_voltage, oneshot=False)
    rospy.spin()
