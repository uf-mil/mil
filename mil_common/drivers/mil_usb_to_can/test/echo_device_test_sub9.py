#!/usr/bin/env python3
import unittest

import rospy
from std_srvs.srv import Trigger, TriggerRequest


class EchoDeviceTestSub9(unittest.TestCase):
    """
    Integration test for CAN2USB board driver. Talks
    to a simulated CAN device which should add two integers
    """

    def __init__(self, *args):
        self.srv = rospy.ServiceProxy("start_echo", Trigger)
        super().__init__(*args)

    def test_correct_response(self):
        """
        Test we can get correct data through CAN bus at least ten times.
        """
        self.srv.wait_for_service(1)
        self.assertTrue(self.srv(TriggerRequest()))


if __name__ == "__main__":
    rospy.init_node("echo_device_test_sub9", anonymous=True)
    import rostest

    rostest.rosrun("mil_usb_to_can", "echo_device_test_sub9", EchoDeviceTestSub9)
    unittest.main()
