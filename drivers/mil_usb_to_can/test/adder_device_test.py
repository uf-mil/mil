#!/usr/bin/env python
import unittest
import rospy
from rospy_tutorials.srv import AddTwoInts, AddTwoIntsRequest


class AdderDeviceTest(unittest.TestCase):
    '''
    Integration test for CAN2USB board driver. Talks
    to a simualted CAN device which should add two integers
    '''
    def __init__(self, *args):
        self.srv = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        super(AdderDeviceTest, self).__init__(*args)

    def test_1service_exists(self):
        '''
        Test raising/clearing kill alarm (user kill) will cause same change in hw-kill
        '''
        try:
            self.srv.wait_for_service(5)
        except rospy.ServiceException as e:
            self.fail('Service error: {}'.format(e))

    def test_2service_works(self):
        a = 3
        b = 6
        correct_sum = a + b
        res = self.srv(AddTwoIntsRequest(a=a, b=b))
        self.assertEquals(res.sum, correct_sum)


if __name__ == "__main__":
    rospy.init_node('adder_device_test', anonymous=True)
    import rostest
    rostest.rosrun('mil_usb_to_can', 'adder_device_test', AdderDeviceTest)
    unittest.main()
