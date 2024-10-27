#! /usr/bin/env python3
import rospy
from navigator_ball_launcher.driver import BallLauncherDevice

if __name__ == "__main__":
    rospy.init_node("ball_launcher")
    port = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6635C08CB65BC36-if00"
    device = BallLauncherDevice(port)
    rospy.spin()
