#!/usr/bin/env python
from __future__ import division

import tf
import txros
from std_srvs.srv import Empty, EmptyRequest
from sub8_ros_tools import msg_helpers
from gazebo_msgs.srv import DeleteModelRequest, SpawnModelRequest
import missions

from twisted.internet import defer
import numpy as np

from diagnostics.gazebo_tests import common


class Job(common.Job):
    _job_name = 'Marker test'

    @txros.util.cancellableInlineCallbacks
    def initial_setup(self):
        '''
        Remove all the models in the sim and spawn a ground plane to run tests with.
        TODO: Sometimes all the models don't get deleted correctly - probably a gazeb problem.
        '''
        models = yield self.model_states.get_next_message()

        if 'ground_plane' not in models.name:
            s = SpawnModelRequest()
            s.model_name = 'ground_plane'
            s.model_xml = ground_plane
            s.initial_pose = msg_helpers.numpy_quat_pair_to_pose([0, 0, -5], [0, 0, 0, 1])
            yield self.spawn_model(s)

        for model in models.name:
            if model == "channel_marker_1" or model == "sub8" or model == 'ground_plane':
                continue
            print "MARKER_TEST - Deleting {}".format(model)
            self.delete_model(DeleteModelRequest(model_name=model))

        self.nh.sleep(1)

    @txros.util.cancellableInlineCallbacks
    def setup(self):
        # Set sub position
        sub_point = np.random.uniform(-5, 5, size=3)
        sub_point[2] = -np.abs(sub_point[2]) / 2.1
        sub_quat = tf.transformations.quaternion_from_euler(0, 0, np.random.uniform(-7, 7, size=1))
        sub_pose = msg_helpers.numpy_quat_pair_to_pose(sub_point, sub_quat)
        yield self.set_model_pose(sub_pose)
        print "MARKER_TEST - Sub at {0}".format(sub_pose)

        # Set marker position
        self.marker_point = np.random.uniform(-3, 3, size=2)
        self.marker_quat = tf.transformations.quaternion_from_euler(0, 0, np.random.uniform(-7, 7, size=1))
        self.marker_pose = msg_helpers.numpy_quat_pair_to_pose(np.append(self.marker_point, -4.9), self.marker_quat)
        print "MARKER_TEST - Marker at {0}".format(self.marker_pose)
        self.set_model_pose(self.marker_pose, model='channel_marker_1')

    @txros.util.cancellableInlineCallbacks
    def run(self, sub):
        print "Running Mission"

        # Call actual mission
        start_time = self.nh.get_time()
        try:
            resp = yield txros.util.wrap_timeout(missions.align_channel.run(sub), 600)  # 10 minute timeout
        except txros.util.TimeoutError:
            print "MARKER_TEST - TIMEOUT"
            defer.returnValue((False, "Timeout"))
        else:
            if resp is None:
                print "MARKER_TEST - ERROR"
                defer.returnValue((False, "Error"))
            print "MARKER_TEST - No timeout."

        print "MARKER_TEST - MISSION TIME:", (self.nh.get_time() - start_time) / 1000000000.0

        sub_odom = msg_helpers.odometry_to_numpy((yield self.true_pose))[0]

        print
        print "MARKER_TEST - Distance Error: {}".format(np.linalg.norm(sub_odom[0][:2] - self.marker_point))
        print
        print "MARKER_TEST - Angle Error: {}".format(np.dot(sub_odom[1], self.marker_quat))
        print

        distance_err = np.linalg.norm(sub_odom[0][:2] - self.marker_point)
        ang_err = np.dot(sub_odom[1], self.marker_quat)
        resp_msg = str(msg_helpers.numpy_quat_pair_to_pose(*sub_odom)) + '\n' + str(self.marker_pose) + '\n' + \
            'Distance Error: ' + str(distance_err) + '\n' + \
            'Angle Error: ' + str(ang_err) + '\n'

        if 0.01 < np.abs(np.dot(sub_odom[1], self.marker_quat)) < 0.99:
            print "MARKER_TEST - FAIL"
            defer.returnValue((False, resp_msg))

        if np.linalg.norm(sub_odom[0][:2] - self.marker_point) > .5:
            print "MARKER_TEST - FAIL"
            defer.returnValue((False, 'POSITION\n' + resp_msg))

        print "MARKER_TEST - PASS"
        print

        defer.returnValue((True, resp_msg))

ground_plane='<?xml version="1.0" ?> <sdf version="1.5"> <model name="ground_plane"> <static>true</static> <link name="link"> <collision name="collision"> <geometry> <plane> <normal>0 0 1</normal> <size>100 100</size> </plane> </geometry> <surface> <friction> <ode> <mu>100</mu> <mu2>50</mu2> </ode> </friction> </surface> </collision> <visual name="visual"> <cast_shadows>false</cast_shadows> <geometry> <plane> <normal>0 0 1</normal> <size>1000 1000</size> </plane> </geometry> <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/Grey</name> </script> </material> </visual> </link> </model> </sdf>'