#!/usr/bin/env python
import unittest
import numpy as np
from geometry_msgs.msg import Quaternion
from mil_ros_tools import quat_to_euler, euler_to_quat, normalize
from mil_ros_tools import compose_transformation


class TestROSTools(unittest.TestCase):

    def test_normalize_vector(self):
        '''Test vector normalization'''
        for k in range(10):
            rand_vec = np.random.random(k)  # Make a random k-length vector

            # Ignore the astronomically unlikely case that a vector has near 0 norm
            if not np.isclose(np.sum(rand_vec), 0):
                normalized = normalize(rand_vec)
                norm = np.linalg.norm(normalized)

                # Test that the norm is 1
                np.testing.assert_almost_equal(norm, 1.0, err_msg="The normalized vector did not have length 1")

    def test_quat_to_euler(self):
        ''' Test quaternion to euler angle '''

        q = Quaternion(x=0.70711, y=0.0, z=0.0, w=0.70711)
        numpy_array = quat_to_euler(q)
        truth = np.array(([1.57079633, 0.0, 0.0]))
        np.testing.assert_almost_equal(numpy_array, truth, err_msg="Quaternion to euler conversion incorrect")

    def test_compose_transformation(self):
        ''' Test quaternion to euler angle '''
        R = np.array(([3, 6, 1],
                      [2, 5, 2],
                      [1, 4, 3]))

        t = np.array((1, 2, 3))

        truth = np.array(([3, 6, 1, 0],
                          [2, 5, 2, 0],
                          [1, 4, 3, 0],
                          [1, 2, 3, 1]))

        test = compose_transformation(R, t)
        np.testing.assert_almost_equal(test, truth, err_msg="Error composing transformation")

    def test_euler_to_quat(self):
        ''' Test quaternion to euler angle '''

        e = np.array(([1.57079633, 0.0, 0.0]))
        testing = euler_to_quat(e)
        # strip away ROS data to just test return values
        # because unittest doesn't support ROS message operandsss
        testing = np.array(([testing.x, testing.y, testing.z, testing.w]))
        truth = np.array(([0.70710678, 0.0, 0.0, 0.70710678]))
        np.testing.assert_almost_equal(testing, truth, err_msg="Incorrect euler to quaternion conversion")


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestROSTools)
    unittest.TextTestRunner(verbosity=2).run(suite)
