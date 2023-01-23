#!/usr/bin/env python3
import unittest

import rospy
from sub8_actuator_board.srv import SetValve, SetValveRequest


class SimulatedBoardTest(unittest.TestCase):
    """
    Integration test for CAN2USB board driver. Talks
    to a simulated CAN device which should add two integers
    """

    def __init__(self, *args):
        self.srv = rospy.ServiceProxy("/set_valve", SetValve)
        super().__init__(*args)

    def test_correct_response(self):
        """
        Test we can get correct data through CAN bus at least ten times.
        """
        self.srv.wait_for_service(1)
        self.srv(SetValveRequest(0, True))
        self.srv(SetValveRequest(0, False))
        self.srv(SetValveRequest(0, False))
        self.srv(SetValveRequest(3, False))
        self.srv(SetValveRequest(4, False))


if __name__ == "__main__":
    rospy.init_node("simulated_board_test", anonymous=True)
    import rostest

    rostest.rosrun("sub8_actuator_board", "simulated_board_test", SimulatedBoardTest)
    unittest.main()
