#!/usr/bin/env python
from __future__ import division

import txros
from std_srvs.srv import Empty, EmptyRequest
import tf
from sub8_ros_tools import msg_helpers
import missions

from twisted.internet import defer
import numpy as np

from diagnostics.gazebo_tests import common


class Job(common.Job):
    _job_name = 'Marker test'

    @txros.util.cancellableInlineCallbacks
    def setup(self):
        self.reset_grid = yield self.nh.get_service_client('/reset_occ_grid', Empty)

        # Set sub position
        sub_point = np.random.uniform(-10, 10, size=3)
        sub_point[2] = -np.abs(sub_point[2]) / 2.1
        sub_quat = tf.transformations.quaternion_from_euler(0, 0, np.random.uniform(-7, 7, size=1))
        sub_pose = msg_helpers.numpy_quat_pair_to_pose(sub_point, sub_quat)
        yield self.set_model_pose(sub_pose)
        print "Sub at {0}".format(sub_pose)

        # Reset grid
        yield self.nh.sleep(.2)
        yield self.reset_grid(EmptyRequest())
        yield self.nh.sleep(1)

    @txros.util.cancellableInlineCallbacks
    def run(self, sub):
        print "Running Mission"
        # Set marker position
        marker_point = np.random.uniform(-8, 8, size=2)
        marker_quat = tf.transformations.quaternion_from_euler(0, 0, np.random.uniform(-7, 7, size=1))
        marker_pose = msg_helpers.numpy_quat_pair_to_pose(np.append(marker_point, -4.9), marker_quat)
        print "Marker at {0}".format(marker_pose)
        self.set_model_pose(marker_pose, model='channel_marker_1')

        # Call actual mission
        start_time = self.nh.get_time()
        try:
            resp = yield txros.util.wrap_timeout(missions.align_channel.run(sub), 600)
        except txros.util.TimeoutError:
            print "TIMEOUT"
            defer.returnValue((False, "Timeout"))
        else:
            print "No timeout."

        print "MISSION TIME:", self.nh.get_time() - start_time

        sub_odom = msg_helpers.odometry_to_numpy((yield self.true_pose.get_next_message()))[0]

        print
        print np.linalg.norm(sub_odom[0][:2] - marker_point)
        print
        print np.dot(sub_odom[1], marker_quat)
        print

        distance_err = np.linalg.norm(sub_odom[0][:2] - marker_point)
        ang_err = np.dot(sub_odom[1], marker_quat)
        resp_msg = str(msg_helpers.numpy_quat_pair_to_pose(*sub_odom)) + '\n' + str(marker_pose) + '\n' + \
            'Distance Error: ' + str(distance_err) + '\n' + \
            'Angle Error: ' + str(ang_err) + '\n'

        # 1 degree
        if 0.01 < np.abs(np.dot(sub_odom[1], marker_quat)) < 0.99:
            print "FAIL"
            defer.returnValue((False, resp_msg))

        # 1 foot
        if np.linalg.norm(sub_odom[0][:2] - marker_point) > .3:
            print "FAIL"
            defer.returnValue((False, 'POSITION\n' + resp_msg))

        print "PASS"
        print

        defer.returnValue((True, resp_msg))
