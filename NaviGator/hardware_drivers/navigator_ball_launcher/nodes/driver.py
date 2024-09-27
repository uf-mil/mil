#! /usr/bin/env python3
import rospy
from navigator_ball_launcher.driver import BallLauncherDevice

if __name__ == "__main__":
    rospy.init_node("ball_launcher")
    device = BallLauncherDevice(str(rospy.get_param("~port")))
    rospy.spin()
