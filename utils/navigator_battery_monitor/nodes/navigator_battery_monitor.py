#!/usr/bin/env python

'''
Battery Monitor: A simple script that approximates the battery voltage by
averaging the supply voltage to each of the four thrusters.
'''


from __future__ import division

from roboteq_msgs.msg import Feedback
import rospy
from std_msgs.msg import Float32


__author__ = "Anthony Olive"
__maintainer__ = "Kevin Allen"
__email__ = "kma1660@gmail.com"
__copyright__ = "Copyright 2016, MIL"
__license__ = "MIT"


rospy.init_node("battery_monitor")


class BatteryMonitor():
    '''
    Monitors the battery voltage measured by the 4 motors on Navigator,
    publishing a moving average of these measurements to /battery_monitor
    at a regular interval
    '''

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
        Publishes the average voltage across all four thrusters to the battery_voltage node as a standard Float32 message and runs
        the voltage_check
        '''

        if (len(self.supply_voltages) > 0):
            self.voltage = sum(self.supply_voltages) / len(self.supply_voltages)

        self.pub_voltage.publish(self.voltage)

if __name__ == "__main__":
    monitor = BatteryMonitor()
    rospy.Timer(rospy.Duration(1), monitor.publish_voltage, oneshot=False)
    rospy.spin()
