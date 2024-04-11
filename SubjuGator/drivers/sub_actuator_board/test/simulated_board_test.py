#!/usr/bin/env python3
import unittest

import rospy
from sub_actuator_board.packets import (
    ActuatorPacketId,
    ActuatorPollRequestPacket,
    ActuatorPollResponsePacket,
    ActuatorSetPacket,
)
from sub_actuator_board.srv import GetValve, GetValveRequest, SetValve, SetValveRequest


class SimulatedBoardTest(unittest.TestCase):
    """
    Integration test for CAN2USB board driver. Talks
    to a simulated CAN device which should add two integers
    """

    def __init__(self, *args):
        self.set_srv = rospy.ServiceProxy("/set_valve", SetValve)
        self.get_srv = rospy.ServiceProxy("/get_valve", GetValve)
        super().__init__(*args)

    def test_correct_response(self):
        """
        Test we can get correct data through CAN bus at least ten times.
        """
        self.set_srv.wait_for_service(1)
        self.get_srv.wait_for_service(1)
        self.assertTrue(self.set_srv(SetValveRequest(0, True)).success)
        self.assertTrue(self.get_srv(GetValveRequest(0)).opened)
        self.assertTrue(self.set_srv(SetValveRequest(0, False)).success)
        self.assertFalse(self.get_srv(GetValveRequest(0)).opened)
        self.assertTrue(self.set_srv(SetValveRequest(0, False)).success)
        self.assertFalse(self.get_srv(GetValveRequest(0)).opened)
        self.assertTrue(self.set_srv(SetValveRequest(1, True)).success)
        self.assertTrue(self.get_srv(GetValveRequest(1)).opened)
        self.assertFalse(self.get_srv(GetValveRequest(0)).opened)
        self.assertTrue(self.set_srv(SetValveRequest(2, False)).success)
        self.assertFalse(self.get_srv(GetValveRequest(2)).opened)
        self.assertFalse(self.get_srv(GetValveRequest(0)).opened)

    def test_packet(self):
        """
        Test that the bytes representation of all packets is correct.
        """
        # ActuatorPollRequestPacket
        packet = ActuatorPollRequestPacket()
        self.assertEqual(bytes(packet), b"7\x01\x03\x01\x00\x00\x04\x0f")
        # ActuatorPollResponsePacket
        packet = ActuatorPollResponsePacket(0b00111)
        self.assertEqual(packet.ball_drop_opened, True)
        self.assertEqual(packet.gripper_opened, True)
        self.assertEqual(packet.torpedo_launcher_opened, True)
        self.assertEqual(bytes(packet), b"7\x01\x03\x02\x01\x00\x07\r!")
        # ActuatorSetPacket
        packet = ActuatorSetPacket(ActuatorPacketId.TORPEDO_LAUNCHER, True)
        self.assertEqual(bytes(packet), b"7\x01\x03\x00\x02\x00\x01\x01\x07\x1d")


if __name__ == "__main__":
    rospy.init_node("simulated_board_test", anonymous=True)
    import rostest

    rostest.rosrun("sub_actuator_board", "simulated_board_test", SimulatedBoardTest)
    unittest.main()
