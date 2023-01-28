#!/usr/bin/env python3
import unittest

import rospy
from ros_alarms import AlarmListener
from std_srvs.srv import SetBool, SetBoolRequest


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
        self.kill_srv.wait_for_service(1)
        self.hw_alarm_listener.wait_for_server()
        self.assertTrue(self.kill_srv(SetBoolRequest(True)).success)
        self.assertTrue(self.hw_alarm_listener.is_raised(True))
        self.assertTrue(self.kill_srv(SetBoolRequest(False)).success)
        self.assertTrue(self.hw_alarm_listener.is_raised(False))


if __name__ == "__main__":
    rospy.init_node("simulated_board_test", anonymous=True)
    import rostest

    rostest.rosrun(
        "sub9_thrust_and_kill_board", "simulated_board_test", SimulatedBoardTest
    )
    unittest.main()
