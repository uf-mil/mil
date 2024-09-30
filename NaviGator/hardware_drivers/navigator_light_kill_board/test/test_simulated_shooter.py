#!/usr/bin/env python3

import os
import pty
import unittest

import rospy
import rostest
from electrical_protocol import AckPacket
from navigator_ball_launcher.driver import BallLauncherDevice
from navigator_ball_launcher.packets import ReleaseBallPacket, SetSpinPacket
from navigator_msgs.srv import (
    BallLauncherDrops,
    BallLauncherDropsRequest,
)
from std_srvs.srv import Empty, EmptyRequest, SetBool, SetBoolRequest


class SimulatedBasicTest(unittest.TestCase):
    def __init__(self, *args):
        super().__init__(*args)

    @classmethod
    def setUpClass(cls):
        cls.master, cls.slave = pty.openpty()
        serial_name = os.ttyname(cls.slave)
        cls.device = BallLauncherDevice(serial_name)
        cls.drop_ball_proxy = rospy.ServiceProxy(
            "/test_simulated_shooter/drop_ball",
            Empty,
        )
        cls.spin_service_proxy = rospy.ServiceProxy(
            "/test_simulated_shooter/spin",
            SetBool,
        )
        cls.drops_service_proxy = rospy.ServiceProxy(
            "/test_simulated_shooter/number_of_drops",
            BallLauncherDrops,
        )
        cls.drop_ball_proxy.wait_for_service()
        cls.spin_service_proxy.wait_for_service()
        cls.drops_service_proxy.wait_for_service()

    def test_drop(self):
        before_drops_resp = self.drops_service_proxy(BallLauncherDropsRequest())
        self.drop_ball_proxy(EmptyRequest())
        # Ensure that we hear the packet
        ReleaseBallPacket.from_bytes(os.read(self.master, 100))
        # and then send back the simulated ACK
        import time

        print(time.time(), "sending")
        os.write(self.master, bytes(AckPacket()))
        print(time.time(), "sent!")
        after_drops_resp = self.drops_service_proxy(BallLauncherDropsRequest())
        self.assertEqual(before_drops_resp.count + 1, after_drops_resp.count)

    def test_spin(self):
        before_drops_resp = self.drops_service_proxy(BallLauncherDropsRequest())
        self.spin_service_proxy(SetBoolRequest(True))
        # Ensure that we hear the packet
        packet = SetSpinPacket.from_bytes(os.read(self.master, 100))
        self.assertEqual(packet.spin_up, True)
        # and then send back the simulated ACK
        os.write(self.master, bytes(AckPacket()))
        self.spin_service_proxy(SetBoolRequest(False))
        # Ensure that we hear the packet
        packet = SetSpinPacket.from_bytes(os.read(self.master, 100))
        self.assertEqual(packet.spin_up, False)
        # and then send back the simulated ACK
        os.write(self.master, bytes(AckPacket()))
        after_drops_resp = self.drops_service_proxy(BallLauncherDropsRequest())
        self.assertEqual(before_drops_resp.count, after_drops_resp.count)

    @classmethod
    def tearDownClass(cls):
        os.close(cls.master)
        os.close(cls.slave)


if __name__ == "__main__":
    rospy.init_node("test_simulated_shooter")
    rostest.rosrun(
        "navigator_ball_launcher",
        "test_simulated_shooter",
        SimulatedBasicTest,
    )
    unittest.main()
