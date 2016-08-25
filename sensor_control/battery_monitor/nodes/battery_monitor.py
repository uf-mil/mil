#!/usr/bin/env python

'''
Battery Monitor: A simple script that approximates the battery voltage by
averaging the supply voltage to each of the four thrusters.
'''


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

        # Initialize the four receiving variables to max voltage
        self.supply_voltage = [29.4, 29.4, 29.4, 29.4]

        # Initialize the average voltage to max
        self.voltage = 29.4

        # Initialize the variables for handling averaging (in an attempt to eliminate outliers)
        self.avg_count = [0, 0, 0, 0]
        self.avg_value = [0, 0, 0, 0]

        # The publisher for the averaged voltage
        self.pub_voltage = rospy.Publisher("/battery_monitor", Float32, queue_size=1)

        # Subscrives to the feedback from each of the four thrusters
        rospy.Subscriber("/FL_motor/feedback", Feedback, self.add_to_fl)
        rospy.Subscriber("/FR_motor/feedback", Feedback, self.add_to_fr)
        rospy.Subscriber("/BL_motor/feedback", Feedback, self.add_to_bl)
        rospy.Subscriber("/BR_motor/feedback", Feedback, self.add_to_br)

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

    def add_to_fl(self, msg):
        '''
        Simple method to connect the FL thruster feedback subscriber to the add_voltage method
        '''
        self.add_voltage(msg, 0)

    def add_to_fr(self, msg):
        '''
        Simple method to connect the FR thruster feedback subscriber to the add_voltage method
        '''
        self.add_voltage(msg, 1)

    def add_to_bl(self, msg):
        '''
        Simple method to connect the BL thruster feedback subscriber to the add_voltage method
        '''
        self.add_voltage(msg, 2)

    def add_to_br(self, msg):
        '''
        Simple method to connect the BR thruster feedback subscriber to the add_voltage method
        '''
        self.add_voltage(msg, 3)

    def add_voltage(self, msg, thruster_id):
        '''
        Averages 42 values from each thruster's feedback (42 is a calculated answer to life, the universe, and everything)
        '''
        if (self.avg_count[thruster_id] != 42):
            self.avg_value[thruster_id] += msg.supply_voltage
            self.avg_count[thruster_id] += 1
        else:
            self.supply_voltage[thruster_id] = self.avg_value[thruster_id] / 42
            self.avg_count[thruster_id] = 0
            self.avg_value[thruster_id] = 0

    def publish_voltage(self):
        '''
        Publishes the average voltage across all four thrustersto the battery_voltage node as a standard Float32 message and runs
        the voltage_check
        '''
        self.voltage = sum(self.supply_voltage) / len(self.supply_voltage)
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
                    'bus_voltage': '%f' % (self.voltage),
                }
            )

        # A high priority battery warning to abort testing
        elif (self.voltage < battery_critical):
            self.battery_critical_alarm.raise_alarm(
                problem_description='Bus voltage is critical; abort testing',
                parameters={
                    'bus_voltage': '%f' % (self.voltage),
                }
            )

        # A low priority battery warning to inform user's of the status
        elif (self.voltage < battery_low):
            self.battery_low_alarm.raise_alarm(
                problem_description='Bus voltage is approaching safety limit',
                parameters={
                    'bus_voltage': '%f' % (self.voltage),
                }
            )


if __name__ == "__main__":
    monitor = BatteryMonitor()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        monitor.publish_voltage()
        rate.sleep()
    rospy.spin()
