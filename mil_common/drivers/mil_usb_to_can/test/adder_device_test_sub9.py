#!/usr/bin/env python3
import unittest

import rospy
from rospy_tutorials.srv import AddTwoInts, AddTwoIntsRequest


class AdderDeviceTestSub9(unittest.TestCase):
    """
    Integration test for CAN2USB board driver. Talks
    to a simulated CAN device which should add two integers
    """

    def __init__(self, *args):
        self.srv = rospy.ServiceProxy("add_two_ints", AddTwoInts)
        super().__init__(*args)

    def test_1service_exists(self):
        """
        Test raising/clearing kill alarm (user kill) will cause same change in hw-kill
        """
        try:
            self.srv.wait_for_service(5)
        except rospy.ServiceException as e:
            self.fail(f"Service error: {e}")

    def test_2service_works(self):
        a = 3
        b = 6
        correct_sum = a + b
        res = self.srv(AddTwoIntsRequest(a=a, b=b))
        self.assertEqual(res.sum, correct_sum)


if __name__ == "__main__":
    rospy.init_node("adder_device_test_sub9", anonymous=True)
    import rostest

    rostest.rosrun("mil_usb_to_can", "adder_device_test_sub9", AdderDeviceTestSub9)
    unittest.main()
