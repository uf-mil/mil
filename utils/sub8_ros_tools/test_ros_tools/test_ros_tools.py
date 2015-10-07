#!/usr/bin/env python
import unittest
import numpy as np
from geometry_msgs.msg import Quaternion, Vector3, Pose2D
from sensor_msgs.msg import Image
from sub8_ros_tools import make_image_msg, get_image_msg
from sub8_ros_tools import rosmsg_to_numpy
from sub8_ros_tools import thread_lock
from sub8_ros_tools import AlarmBroadcaster
from sub8_ros_tools import skew_symmetric_cross, make_rotation


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
        ran = False

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

    def test_instantiate_alarm_broadcaster(self):
        '''Ensure that the alarm broadcaster instantiates without errors'''
        broadcaster = AlarmBroadcaster()

    def test_add_alarm(self):
        '''Ensure that adding an alarm succeeds without errors'''
        broadcaster = AlarmBroadcaster()
        alarm = broadcaster.add_alarm(
            name='wake-up',
            action_required=True,
            severity=1,
            problem_description='This is a problem',
            json_parameters='{"concern": ["a", "b", "c"]}' 
        )

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
        np.testing.assert_allclose(skew_sym, truth)

    def test_make_rotation(self):
        '''Test several random vector pairs, and see if we can generate a valid alignment'''
        for k in range(10):
            p = np.random.random(3) * 10
            q = np.random.random(3) * 10

            R = make_rotation(p, q)
            p_rotated = R.dot(p)

            np.testing.assert_allclose([0.0, 0.0, 0.0], np.cross(p_rotated, q), atol=1e-5)

if __name__ == '__main__':
    unittest.main()