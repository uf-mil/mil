#!/usr/bin/env python3
import unittest

import rospy
from ros_alarms import AlarmListener
from std_srvs.srv import SetBool, SetBoolRequest
from sub9_thrust_and_kill_board.packets import (
    HeartbeatReceivePacket,
    HeartbeatSetPacket,
    KillReceivePacket,
    KillSetPacket,
    KillStatus,
    ThrusterId,
    ThrustSetPacket,
)


class SimulatedBoardTest(unittest.TestCase):
    """
    Integration test for CAN2USB board driver. Talks
    to a simulated CAN device which should add two integers
    """

    def __init__(self, *args):
        self.kill_srv = rospy.ServiceProxy("/set_mobo_kill", SetBool)
        self.hw_alarm_listener = AlarmListener("hw-kill")
        super().__init__(*args)

    def test_correct_response(self):
        """
        Test we can get correct data through CAN bus at least ten times.
        """
        self.kill_srv.wait_for_service(5)
        self.hw_alarm_listener.wait_for_server()
        self.assertTrue(self.kill_srv(SetBoolRequest(True)).success)
        self.assertTrue(self.hw_alarm_listener.is_raised(True))
        self.assertTrue(self.kill_srv(SetBoolRequest(False)).success)
        self.assertTrue(self.hw_alarm_listener.is_raised(False))

    def test_packet(self):
        # ThrustSetPacket
        thrust_set_packet = ThrustSetPacket(ThrusterId.FLH, 0.5)
        self.assertEqual(thrust_set_packet.thruster_id, ThrusterId.FLH)
        self.assertEqual(
            bytes(thrust_set_packet),
            b"7\x01\x02\x02\x05\x00\x00\x00\x00\x00?H\x84",
        )
        # HeartbeatSetPacket
        heartbeat_set_packet = HeartbeatSetPacket()
        self.assertEqual(bytes(heartbeat_set_packet), b"7\x01\x02\x00\x00\x00\x02\x08")
        # HeartbeatReceivePacket
        heartbeat_receive_packet = HeartbeatReceivePacket()
        self.assertEqual(
            bytes(heartbeat_receive_packet),
            b"7\x01\x02\x01\x00\x00\x03\x0b",
        )
        # KillSetPacket
        kill_set_packet = KillSetPacket(True, KillStatus.BATTERY_LOW)
        self.assertEqual(bytes(kill_set_packet), b"7\x01\x02\x03\x02\x00\x01\x04\x0c)")
        # KillReceivePacket
        kill_receive_packet = KillReceivePacket(True, KillStatus.BATTERY_LOW)
        self.assertEqual(
            bytes(kill_receive_packet),
            b"7\x01\x02\x04\x02\x00\x01\x04\r.",
        )


if __name__ == "__main__":
    rospy.init_node("simulated_board_test", anonymous=True)
    import rostest

    rostest.rosrun(
        "sub9_thrust_and_kill_board",
        "simulated_board_test",
        SimulatedBoardTest,
    )
    unittest.main()
