#!/usr/bin/env python
from geometry_msgs.msg import WrenchStamped
from navigator_msgs.srv import WrenchSelect
import rospy
from std_msgs.msg import Bool, String, Float32
from visualization_msgs.msg import Marker, MarkerArray
from navigator_alarm import AlarmListener
import numpy as np

class RvizStrings(object):
    ID = 1337
    NS = "boat_info"
    X_POS = -4.0
  
    def __init__(self):
        self.station_holding = False
        self.kill_alarm = False
        self.voltage = None
        self.wrench = None
        rospy.Subscriber("/wrench/current", String, self.wrench_current_cb)
        rospy.Subscriber("/battery_monitor",Float32, self.battery_monitor_cb)
        self.markers_pub = rospy.Publisher('/boat_info', Marker, queue_size=10)
        self.kill_listener = AlarmListener('kill', self.kill_alarm_cb)
        self.station_hold_listner = AlarmListener('station_hold', self.station_alarm_cb)

    def update(self):
        marker = Marker()
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.action = Marker.ADD;
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.ns = self.NS
        marker.pose.position.x = self.X_POS
        marker.id = self.ID
        marker.color.a = 1
        marker.color.b = 1
        if not self.voltage == None:
            marker.text += "Voltage {}".format(self.voltage)
        if not self.wrench == None:
            marker.text += "\nWrench {}".format(self.wrench)
        if self.station_holding:
            marker.text += "\nStation Holding"
        if self.kill_alarm:
            marker.text += "\nKILLED"
        self.markers_pub.publish(marker)  

    def station_alarm_cb(self, alarm):
        self.station_holding = not alarm.clear
        self.update()

    def kill_alarm_cb(self, alarm):
        self.kill_alarm = not alarm.clear
        self.update()

    def wrench_current_cb(self,string):
        self.wrench = string.data
        self.update()

    def battery_monitor_cb(self,voltage):
        self.voltage = np.round(voltage.data,2)
        self.update()


if __name__ == "__main__":
    rospy.init_node('info_markers')
    arb = RvizStrings()
    rospy.spin()
