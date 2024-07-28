#!/usr/bin/env python

import rospy
import psutil  
from ros_alarms_msgs.msg import Alarm as AlarmMsg
from ros_alarms_msgs.srv import AlarmSet, AlarmSetRequest

class CPUMonitor:
    def __init__(self, threshold=75.0):
        self.threshold = threshold
        self.alarm_name = "high_cpu_temp"
        self.node_name = rospy.get_name()
        self.alarm_set = rospy.ServiceProxy("/alarm/set", AlarmSet)

    def get_cpu_temp(self):
        temps = psutil.sensors_temperatures()
        if 'coretemp' in temps:
            return temps['coretemp'][0].current
        return None

    def raise_alarm(self):
        try:
            req = AlarmSetRequest()
            req.alarm.alarm_name = self.alarm_name
            req.alarm.node_name = self.node_name
            req.alarm.raised = True
            req.alarm.problem_description = "CPU temperature is too high!"
            req.alarm.parameters = json.dumps({"temperature": self.get_cpu_temp()})
            req.alarm.severity = 5
            self.alarm_set(req)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def clear_alarm(self):
        try:
            req = AlarmSetRequest()
            req.alarm.alarm_name = self.alarm_name
            req.alarm.node_name = self.node_name
            req.alarm.raised = False
            req.alarm.problem_description = "CPU temperature back to normal"
            req.alarm.parameters = json.dumps({"temperature": self.get_cpu_temp()})
            req.alarm.severity = 0
            self.alarm_set(req)
        except:
            rospy.logerr("Service call failed: %s" % e)

    def monitor(self):
        rate = rospy.Rate(1)  
        while not rospy.is_shutdown():
            temp = self.get_cpu_temp()
            if temp:
                if temp >= self.threshold:
                    self.raise_alarm()
                else:
                    self.clear_alarm()
            rate.sleep()
#  Examples usage
# if __name__ == '__main__':
#     rospy.init_node('cpu_monitor_node')
#     monitor = CPUMonitor(threshold=75.0)
#     monitor.monitor()
