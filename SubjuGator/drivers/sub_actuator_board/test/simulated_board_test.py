#!/usr/bin/env python3
import unittest

import rospy
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
        self.assertTrue(self.set_srv(SetValveRequest(3, True)).success)
        self.assertTrue(self.get_srv(GetValveRequest(3)).opened)
        self.assertFalse(self.get_srv(GetValveRequest(0)).opened)
        self.assertTrue(self.set_srv(SetValveRequest(4, False)).success)
        self.assertFalse(self.get_srv(GetValveRequest(4)).opened)
        self.assertFalse(self.get_srv(GetValveRequest(0)).opened)


if __name__ == "__main__":
    rospy.init_node("simulated_board_test", anonymous=True)
    import rostest

    rostest.rosrun("sub_actuator_board", "simulated_board_test", SimulatedBoardTest)
    unittest.main()
