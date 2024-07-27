#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float32

global alarm_pub

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



if __name__ == '__main__':
  
  try:
    cpu_temp_alarm()
  except rospy.InterruptedError:
    pass