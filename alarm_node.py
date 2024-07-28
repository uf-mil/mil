#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float32

global alarm_pub

from typing import Optional, Union
from ros_alarms_msgs.msg import Alarm as AlarmMsg
from .alarms import Alarm, HandlerBase


class HighTempAlarmHandler(HandlerBase):
    alarm_name = "high_cpu_temp"
    severity_required = (0, 5)

    def raised(self, alarm: AlarmMsg):
        """
        Called whenever the high CPU temperature alarm is raised.
        """
        print(f"ALARM! CPU temperature too high: {alarm.parameters['temperature']}°C")
        return True

    def cleared(self, alarm: AlarmMsg):
        """
        Called whenever the high CPU temperature alarm is cleared.
        """
        print("CPU temperature back to normal.")
        return True

    def meta_predicate(self, meta_alarm: Alarm, alarms) -> Union[bool, Alarm]:
        """
        Called on an update to one of this alarm's meta alarms, if there are any.
        By default, returns True if any meta alarms are raised.
        """
        return any(alarm.raised for name, alarm in alarms.items())



def cpu_temp_callback(temp):

  # Sets of cpu temp alarm
  if float(temp.data) > 75:
    rospy.logwarn(f"CPU TEMPERATURE EXCEEDS: {temp.data}")
    alarm_pub.publish(True)
  
  else:
    alarm_pub.publish(False)



def cpu_temp_alarm():
    global alarm_pub  

    # Initializing node
    rospy.init_node('cpu_temp_alarm', anonymous=True)

    rospy.Subscriber('cpu_temperature', Float32, cpu_temp_callback)
    alarm_pub = rospy.Publisher('cpu_alarm', bool, queue_size=10)

    rospy.spin()


# Example usage
# if __name__ == '__main__':
  
#   
#    cpu_temp_alarm()
#  
