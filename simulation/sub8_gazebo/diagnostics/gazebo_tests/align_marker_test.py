#!/usr/bin/env python
from __future__ import division

import rospy
import txros
from std_srvs.srv import Empty, EmptyRequest
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sub8_msgs.srv import SearchPose, SearchPoseRequest
from sub8_gazebo.srv import RunJob, RunJobResponse
import tf
from sub8_ros_tools import msg_helpers, geometry_helpers
import job_runner
import missions
from sub8 import tx_sub

from twisted.internet import reactor, defer
import numpy as np
import time


@txros.util.cancellableInlineCallbacks
def run_mission(srv, world_position_actual, sub, nh):
    print "Running Mission"

    reset_grid = yield nh.get_service_client('/reset_occ_grid', Empty)

    # Set sub position
    sub_point = np.random.uniform(-10, 10, size=3)
    sub_point[2] = -np.abs(sub_point[2]) / 2.1
    sub_quat = tf.transformations.quaternion_from_euler(0, 0, np.random.uniform(-7, 7, size=1))
    sub_pose = msg_helpers.numpy_quat_pair_to_pose(sub_point, sub_quat)
    print "Sub at {0}".format(sub_pose)
    yield job_runner.JobManager.set_model_position(nh, sub_pose)

    yield nh.sleep(.2)
    yield reset_grid(EmptyRequest())
    yield nh.sleep(1)

    # Set marker position
    marker_point = np.random.uniform(-8, 8, size=2)
    marker_quat = tf.transformations.quaternion_from_euler(0, 0, np.random.uniform(-7, 7, size=1))
    marker_pose = msg_helpers.numpy_quat_pair_to_pose(np.append(marker_point, -4.9), marker_quat)
    print "Marker at {0}".format(marker_pose)
    yield job_runner.JobManager.set_model_position(nh, marker_pose, model='channel_marker_1')

    # Call actual mission
    start_time = nh.get_time()
    try:
        resp = yield txros.util.wrap_timeout(missions.align_channel.run(sub), 600)
    except txros.util.TimeoutError:
        print "TIMEOUT"
        defer.returnValue(RunJobResponse(success=False, message='TIMEOUT\n'))
    else:
        print "No timeout."

    print "MISSION TIME:", nh.get_time() - start_time

    sub_odom = msg_helpers.odometry_to_numpy((yield world_position_actual.get_next_message()))[0]

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
        defer.returnValue(RunJobResponse(success=False, message='ROTATION\n' + resp_msg))

    # 1 foot
    if np.linalg.norm(sub_odom[0][:2] - marker_point) > .3:
        print "FAIL"
        defer.returnValue(RunJobResponse(success=False, message='POSITION\n' + resp_msg))

    print "PASS"
    print

    defer.returnValue(RunJobResponse(success=True, message=resp_msg))


@txros.util.cancellableInlineCallbacks
def main():
    nh = yield txros.NodeHandle.from_argv('align_channel_tester')
    world_position_actual = yield nh.subscribe('/world_odom', Odometry)
    sub = yield tx_sub.get_sub(nh)

    nh.advertise_service('/gazebo/job_runner/mission_start', RunJob, lambda srv: run_mission(srv, world_position_actual, sub, nh))
    print "Waiting for service call."

if __name__ == '__main__':
    reactor.callWhenRunning(main)
    reactor.run()