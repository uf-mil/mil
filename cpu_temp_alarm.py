#!/usr/bin/env python3

import psutil
import rospy
from std_msgs.msg import Float32


def get_cpu_temp():
    # Checks computer sensors
    temp = psutil.sensors_temperatures()
    if 'coretemp' in temp:
        return temp['coretemp'][0].current
    else:
        raise ValueError("Core temp not found!")



def cpu_temp_ROS():
  
  # ROS initializing 

  rospy.init_node('cpu_temp_node', anonymous=True)
  pub = rospy.Publisher('cpu_temperature', Float32, queue_size=18)
  rate = rospy.Rate(1)

   # Loops through and actively gets cpu temp
   
  while not rospy.is_shutdown():
    cpu_temp = get_cpu_temp()
    rospy.loginfo("CPU TEMP: %f", cpu_temp)
    pub.publish(cpu_temp)
    rate.sleep()


if __name__ == '__main__':
  try:
    cpu_temp_ROS()
  
  # For interruption failure

  except rospy.ROSInterruptException:
    pass
    

  

  

    
