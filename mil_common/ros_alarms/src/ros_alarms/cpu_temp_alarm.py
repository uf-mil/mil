#!/usr/bin/env python3

import psutil
import rospy
from std_msgs.msg import Float32, Bool

def get_cpu_temp():
    # Checks computer sensors
    temp = psutil.sensors_temperatures()
    if 'coretemp' in temp:
        return temp['coretemp'][0].current
    else:
        raise ValueError("Core temp not found!")

def cpu_temp_callback(alarm):
    if alarm.data:
        rospy.logwarn("ALARM: CPU temperature exceeds the threshold!")

def cpu_temp_ROS():
    # ROS initializing
    rospy.init_node('cpu_temp_node', anonymous=True)
    temp_pub = rospy.Publisher('cpu_temperature', Float32, queue_size=10)
    rospy.Subscriber('cpu_alarm', Bool, cpu_temp_callback)
    rate = rospy.Rate(1)

    # Loops through and actively gets CPU temp
    while not rospy.is_shutdown():
        try:
            cpu_temp = get_cpu_temp()
            rospy.loginfo("CPU TEMP: %f", cpu_temp)
            temp_pub.publish(cpu_temp)
        except ValueError as e:
            rospy.logwarn(str(e))
        rate.sleep()

# if __name__ == '__main__':
#     try:
#         cpu_temp_ROS()
#     except rospy.ROSInterruptException:
#         pass

    
