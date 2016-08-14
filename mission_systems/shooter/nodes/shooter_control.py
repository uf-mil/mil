#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String

class ShooterController:
    def __init__(self):
        rospy.init_node("shooter_controller")
        self.rate = rospy.Rate(10)
    def control(self):
        pub = rospy.Publisher("/shooter/control", String,queue_size=5)
        while not rospy.is_shutdown():
            sys.stdout.write("Enter comand: ")
            command = sys.stdin.readline().strip()
            #rospy.loginfo(command)
            pub.publish(command)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        sc = ShooterController()
        sc.control();
    except rospy.ROSInterruptException:
        pass

