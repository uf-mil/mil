#!/usr/bin/env python
from __future__ import division

import rospy
import txros
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sub8_gazebo.srv import RunJob, RunJobResponse
from sub8_ros_tools import msg_helpers
import job_runner
import tf

from twisted.internet import reactor, defer
import numpy as np


@txros.util.cancellableInlineCallbacks
def run_mission(srv, sub, nh):
    print "Running Mission"
    # Generate point to go to
    point = np.random.uniform(-20, 20, size=3)
    point[2] = -np.abs(point[2]) / 4

    quat = tf.transformations.quaternion_from_euler(0, 0, np.random.uniform(-7, 7, size=1))

    pose = msg_helpers.numpy_quat_pair_to_pose(point, quat)
    print pose

    # Go to point
    j = yield job_runner.JobManager.set_model_position(nh, pose)
    yield txros.util.wall_sleep(5.0)
    print "Movement should be complete."

    excpeted_position = np.array([point, quat])

    actual_odom = msg_helpers.odometry_to_numpy((yield sub.get_next_message()))
    actual_position = np.array(actual_odom[0])

    print excpeted_position, actual_position
    print "Done, err:"
    print np.abs(excpeted_position - actual_position)
    print

    if (np.abs(excpeted_position[0] - actual_position[0]) < .5).all() and \
       (np.abs(excpeted_position[1] - actual_position[1]) < .2).all():
        print "PASS"
        print
        defer.returnValue(RunJobResponse(success=True, message=str(pose)))
    else:
        print "FAIL"
        print
        err_msg = str(excpeted_position) + '\n' + str(actual_position)
        defer.returnValue(RunJobResponse(success=False, message=err_msg))


@txros.util.cancellableInlineCallbacks
def main():
    nh = yield txros.NodeHandle.from_argv('test')
    world_position_actual = yield nh.subscribe('/world_odom', Odometry)
    yield run_mission(None, world_position_actual, nh)
    #nh.advertise_service('/gazebo/job_runner/mission_start', RunJob, lambda srv: run_mission(srv, world_position_actual, nh))
    print "Waiting for service call."

if __name__ == '__main__':
    reactor.callWhenRunning(main)
    reactor.run()