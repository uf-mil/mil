#! /usr/bin/env python3
import rospy
from navigator_drone_comm.driver import DroneCommDevice

if __name__ == "__main__":
    rospy.init_node("navigator_drone_comm")
    port = str(
        rospy.get_param(
            "~port",
            "/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTDCKG37-if00-port0",
        ),
    )
    device = DroneCommDevice(port)
    rospy.spin()
