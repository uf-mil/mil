#!/usr/bin/env python
import unittest
import numpy as np
from geometry_msgs.msg import Quaternion, Vector3, Pose2D
from sensor_msgs.msg import Image
from sub8_ros_tools.image_helpers import make_image_msg, get_image_msg
from sub8_ros_tools.msg_helpers import rosmsg_to_numpy, make_wrench_stamped
from sub8_ros_tools.threading_helpers import thread_lock
from sub8_ros_tools.geometry_helpers import skew_symmetric_cross, make_rotation, normalize


class TestROSTools(unittest.TestCase):
    def test_rosmsg_to_numpy_quaternion(self):
        '''Test a rosmsg conversion for a geometry_msgs/Quaternion'''
        # I know this is not a unit quaternion
        q = Quaternion(x=0.7, y=0.7, z=0.1, w=0.2)
        numpy_array = rosmsg_to_numpy(q)

        np.testing.assert_allclose(
            np.array([0.7, 0.7, 0.1, 0.2]),
            numpy_array
        )

    def test_rosmsg_to_numpy_vector(self):
        '''Test a rosmsg conversion for a geometry_msgs/Vector'''
        v = Vector3(x=0.1, y=99., z=21.)
        numpy_array = rosmsg_to_numpy(v)

        np.testing.assert_allclose(
            np.array([0.1, 99., 21.]),
            numpy_array
        )

    def test_rosmsg_to_numpy_custom(self):
        '''Test a rosmsg conversion for a custom msg'''
        pose_2d = Pose2D(x=1.0, y=2.0, theta=3.14)
        numpy_array = rosmsg_to_numpy(pose_2d, ['x', 'y', 'theta'])

        np.testing.assert_allclose(
            np.array([1.0, 2.0, 3.14]),
            numpy_array
        )

    def test_make_wrench_stamped(self):
        '''Test that wrenchmaking works'''
        for k in range(10):
            force = np.random.random(3) * 10
            torque = np.random.random(3) * 10
        wrench_msg = make_wrench_stamped(force, torque, frame='/enu')

        msg_force = rosmsg_to_numpy(wrench_msg.wrench.force)  # noqa
        msg_torque = rosmsg_to_numpy(wrench_msg.wrench.torque)  # noqa
        self.assertIsNotNone(msg_force)
        self.assertIsNotNone(msg_torque)

    def test_make_image_msg(self):
        '''Test that make ros image message doesn't fail'''
        im = np.ones((255, 255, 3)).astype(np.uint8)
        im_msg = make_image_msg(im)
        self.assertIsInstance(im_msg, Image)

    def test_get_image_msg(self):
        '''Test that a made ros image can be returned to its original form'''
        im = np.ones((255, 255, 3)).astype(np.uint8)
        im_msg = make_image_msg(im)

        cv_im = get_image_msg(im_msg)
        np.testing.assert_array_equal(im, cv_im)

    def test_thread_lock(self):
        '''Test that the thread lock decorator correctly locks, in the correct order'''
        class FakeLock(object):
            def __init__(self):
                self.entry = False
                self.exit = False

            def __enter__(self):
                self.entry = True

            def __exit__(self, *args):
                self.exit = True

        fake_lock = FakeLock()

        @thread_lock(fake_lock)
        def lock_test(a):
            return (fake_lock.entry is True) and (fake_lock.exit is False)

        result = lock_test(1)

        self.assertTrue(fake_lock.entry, msg='Thread was never locked')
        self.assertTrue(fake_lock.exit, msg='Thread was never released')
        self.assertTrue(result, msg='Thread was not locked while the function was executed')

    def test_skew_symmetric_cross(self):
        '''Test that the skew symmetric cross product matrix produces the definition
            [1] https://en.wikipedia.org/wiki/Cross_product#Skew-symmetric_matrix
        '''
        skew_sym = skew_symmetric_cross([1, 2, 3])
        truth = np.array([
            [+0, -3, +2],
            [+3, +0, -1],
            [-2, +1, +0],
        ])
        np.testing.assert_allclose(skew_sym, truth, err_msg="Did not make a Skew-symmetric matrix. Pretty big screw-up imho.")

    def test_make_rotation(self):
        '''Test several random vector pairs, and see if we can generate a valid alignment'''
        scaling = 10
        for k in range(10):
            p = (np.random.random(3) - 0.5) * scaling
            q = (np.random.random(3) - 0.5) * scaling

            R = make_rotation(p, q)
            p_rotated = R.dot(p)

            # Test that the matrix actually aligns p with q
            np.testing.assert_allclose(
                [0.0, 0.0, 0.0], np.cross(p_rotated, q), atol=1e-5,
                err_msg="The generated rotation matrix did not align the input vectors, {} to {}".format(
                    p, q
                )
            )
            self.assertGreater(np.dot(p_rotated, q), 0.0, msg="The rotation did wacky inversion")

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


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestROSTools)
    unittest.TextTestRunner(verbosity=2).run(suite)
