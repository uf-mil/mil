#!/usr/bin/env python3

import os
import pty
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Point
from navigator_drone_comm.driver import DroneCommDevice
from navigator_drone_comm.packets import (
    EStopPacket,
    GPSDronePacket,
    HeartbeatReceivePacket,
    StartPacket,
    StopPacket,
)
from navigator_msgs.msg import DroneTarget
from navigator_msgs.srv import DroneMission, DroneMissionRequest
from std_msgs.msg import Int8
from std_srvs.srv import Empty, EmptyRequest


class SimulatedBasicTest(unittest.TestCase):
    def __init__(self, *args):
        super().__init__(*args)

    @classmethod
    def gps_callback(cls, data):
        cls.drone_gps = data

    @classmethod
    def target_callback(cls, data):
        cls.target = data

    @classmethod
    def heartbeat_drone_callback(cls, data):
        cls.last_drone_heartbeat = data

    @classmethod
    def setUpClass(cls):
        cls.master, cls.slave = pty.openpty()
        serial_name = os.ttyname(cls.slave)
        cls.device = DroneCommDevice(serial_name)
        cls.estop_proxy = rospy.ServiceProxy(
            "/test_simulated_drone/estop",
            Empty,
        )
        cls.estop_proxy.wait_for_service()
        cls.stop_proxy = rospy.ServiceProxy(
            "/test_simulated_drone/stop",
            Empty,
        )
        cls.stop_proxy.wait_for_service()
        cls.start_proxy = rospy.ServiceProxy(
            "/test_simulated_drone/start",
            DroneMission,
        )
        cls.start_proxy.wait_for_service()
        cls.drone_gps = None
        cls.target = None
        cls.last_drone_heartbeat = None
        rospy.Subscriber("~gps", Point, cls.gps_callback)
        rospy.Subscriber("~target", DroneTarget, cls.target_callback)
        rospy.Subscriber("~drone_heartbeat", Int8, cls.heartbeat_drone_callback)

    def test_device_initialization(self):
        self.assertIsNotNone(self.device)

    def test_estop(self):
        self.estop_proxy(EmptyRequest())
        # give up to 3 tries to hear the estop packet and not just heartbeat
        for i in range(3):
            try:
                packet = EStopPacket.from_bytes(os.read(self.master, 8))
                break
            except RuntimeError:
                # ignore heartbeat packets here
                pass
        self.assertIsInstance(packet, EStopPacket)

    def test_stop(self):
        self.stop_proxy(EmptyRequest())
        # give up to 3 tries to hear the stop packet and not just heartbeat
        for i in range(3):
            try:
                packet = StopPacket.from_bytes(os.read(self.master, 8))
                break
            except RuntimeError:
                # ignore heartbeat packets here
                pass
        self.assertIsInstance(packet, StopPacket)

    def test_start_mission(self):
        self.start_proxy(DroneMissionRequest("STARTB"))
        # give up to 3 tries to hear the start packet and not just heartbeat
        for i in range(3):
            try:
                packet = StartPacket.from_bytes(os.read(self.master, 28))
                break
            except RuntimeError:
                # ignore heartbeat packets here
                pass
        self.assertIsInstance(packet, StartPacket)

    def test_gps_drone_receive(self):
        self.assertEqual(self.drone_gps, None)
        gps_packet = GPSDronePacket(lat=37.77, lon=-122.4194, alt=30.0)
        os.write(self.master, bytes(gps_packet))
        rospy.sleep(0.5)
        self.assertAlmostEqual(round(self.drone_gps.x, 3), 37.77)

    # Disabling this test because it was changed at competition and shouldn't work anymore!
    # def test_target_receive(self):
    #     target_packet = TargetPacket(lat=-67.7745, lon=12.654, logo="R")
    #     os.write(self.master, bytes(target_packet))
    #     rospy.sleep(1)
    #     rospy.loginfo(self.target.logo)
    #     self.assertEqual(self.target.logo, "R_LOGO")

    # tests that a heartbeat is sent from the boat every second
    # def test_z_sending_heartbeats(self):
    #     start_time = time.time()
    #     for i in range(1, 4):
    #         packet = HeartbeatSetPacket.from_bytes(os.read(self.master, 8))
    #         self.assertIsInstance(packet, HeartbeatSetPacket)
    #         self.assertLess(time.time() - start_time, 1 * i + 0.1)

    # test that the boat can receive drone heartbeats
    def test_z_heartbeat_receive(self):
        for i in range(3):
            heartbeat_packet = HeartbeatReceivePacket(status=1)
            self.device.on_packet_received(heartbeat_packet)
            time.sleep(1)
            self.assertEqual(self.last_drone_heartbeat.data, 1)

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
