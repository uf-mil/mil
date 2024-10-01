#! /usr/bin/env python3
import rospy
from navigator_light_kill_board.driver import KillLightBoardDevice

if __name__ == "__main__":
    rospy.init_node("ball_launcher")
    device = KillLightBoardDevice(str(rospy.get_param("~port")))
    rospy.spin()
