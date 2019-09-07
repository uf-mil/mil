#!/usr/bin/env python
import unittest
import rostest
import rospy
from geometry_msgs.msg import PointStamped
from mil_tools import numpy_to_point
from mil_bounds import BoundsClient
import numpy as np


class BoundsTest(unittest.TestCase):
    '''
    Tests mil_bounds by simulating rviz clicked points and ensuring that the bounds
    are changed and correctly transformed.
    '''
    def setUp(self):
        pub = rospy.Publisher('clicked_point', PointStamped, queue_size=4)
        rospy.sleep(1)
        points = np.array([[0, 0, 0],
                           [10, 0, 0],
                           [10, 10, 0],
                           [0, 10, 0]])
        for point in points:
            ps = PointStamped()
            ps.header.frame_id = 'a'
            ps.point = numpy_to_point(point)
            rospy.loginfo(ps)
            pub.publish(ps)
            rospy.sleep(1)
        rospy.sleep(1)

    def test_it_works(self):
        c = BoundsClient()
        bounds = c.get_bounds()
        correct = np.array([[-1, 0, 0],
                            [9, 0, 0],
                            [9, 10, 0],
                            [-1, 10, 0]], dtype=float)
        self.assertTrue(np.all(bounds == correct))


if __name__ == '__main__':
    rospy.init_node('test_bounds')
    rostest.rosrun('mil_bounds', 'bounds_test', BoundsTest)
    unittest.main()
