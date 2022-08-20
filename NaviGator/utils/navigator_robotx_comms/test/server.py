#!/usr/bin/env python3

import rospy
from robotx_comms_server import RobotXServer

if __name__ == "__main__":
    rospy.init_node("comms_server")

    tcp_ip = rospy.get_param("/navigator_robotx_comms/td_ip")
    tcp_port = rospy.get_param("/navigator_robotx_comms/td_port")

    server = RobotXServer(tcp_ip, tcp_port)

    server.connect()
    while not rospy.is_shutdown():
        rx_data = None
        while rx_data is None:
            rx_data = server.receive_message()
        rospy.logwarn(rx_data)

    rospy.spin()
