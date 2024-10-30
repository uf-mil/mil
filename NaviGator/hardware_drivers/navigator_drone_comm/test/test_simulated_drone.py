#!/usr/bin/env python3

import os
import pty
import time
import unittest

import rospy
import rostest
from navigator_drone_comm.driver import DroneCommDevice
from navigator_drone_comm.packets import (
    GPSDronePacket,
    HeartbeatReceivePacket,
    TargetPacket,
)


class SimulatedBasicTest(unittest.TestCase):
    def __init__(self, *args):
        super().__init__(*args)

    @classmethod
    def setUpClass(cls):
        cls.master, cls.slave = pty.openpty()
        serial_name = os.ttyname(cls.slave)
        cls.device = DroneCommDevice(serial_name)

    def test_device_initialization(self):
        self.assertIsNotNone(self.device)

    def test_gps_drone_receive(self):
        gps_packet = GPSDronePacket(lat=37.7749, lon=-122.4194, alt=30.0)
        self.device.on_packet_received(gps_packet)

    def test_target_receive(self):
        target_packet = TargetPacket(lat=-67.7745, lon=12.654, color="r")
        self.device.on_packet_received(target_packet)

    def test_z_heartbeat_receive(self):
        rospy.loginfo("Testing receiving heartbeats for 5 secs...")
        for i in range(5):
            heartbeat_packet = HeartbeatReceivePacket()
            self.device.on_packet_received(heartbeat_packet)
            time.sleep(1)

    def test_z_longrun(self):
        rospy.loginfo("Starting long test (ctl+c to end)...")
        start_time = time.time()
        duration = 200
        while time.time() - start_time < duration:
            rospy.rostime.wallsleep(0.1)

    @classmethod
    def tearDownClass(cls):
        os.close(cls.master)
        os.close(cls.slave)


if __name__ == "__main__":
    rospy.init_node("test_simulated_drone")
    rostest.rosrun(
        "navigator_drone_comm",
        "test_simulated_drone",
        SimulatedBasicTest,
    )
    unittest.main()
