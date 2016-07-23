from __future__ import division

import tf
from twisted.internet import defer
from txros import util
from sub8 import Searcher
from sub8_ros_tools import pose_to_numpy
import numpy as np


SEARCH_DEPTH = .65
SPEED = .1

def catch_error(failure):
    print failure.printTraceback()

@util.cancellableInlineCallbacks
def run(sub):
    # Go to max height to see as much as we can.
    yield sub.move.depth(SEARCH_DEPTH).zero_roll_and_pitch().go(speed=SPEED)
    yield sub._node_handle.sleep(1.0)

    # Do a little jig - change this for the pool.
    pattern = [sub.move.right(1), sub.move.forward(1), sub.move.left(1), sub.move.backward(1),
               sub.move.right(2), sub.move.forward(2), sub.move.left(2), sub.move.backward(2)]
    s = Searcher(sub, sub.channel_marker.get_pose, pattern)
    resp = None
    try:
        resp = yield s.start_search(loop=True, timeout=600)
    except:
        print "MARKER_MISSION - Timed out probably."
        defer.returnValue(None)

    if resp is None:
        print "MARKER_MISSION - Marker not found."
        defer.returnValue(None)

    yield sub.move.set_position(pose_to_numpy(resp.pose.pose)[0]).go(speed=SPEED)

    # How many times should we attempt to reposition ourselves
    iterations = 3
    # To make sure we don't go too far off.
    est_target_rotations = []
    for i in range(iterations):
        print "Iteration {}.".format(i + 1)
        response = yield sub.channel_marker.get_pose()
        if response is None:
            continue

        response = pose_to_numpy(response.pose.pose)

        # Using euler - shoot me.
        yaw = tf.transformations.euler_from_quaternion(response[1])[2]

        est_target_rotations.append(yaw)
        avg_rotation = sum(est_target_rotations) / len(est_target_rotations)

        yield sub.move.set_position(response[0]).yaw_left(yaw).zero_roll_and_pitch().go(speed=SPEED)

        yield sub._node_handle.sleep(3.0)

    print "MARKER_MISSION - Done!"
    defer.returnValue(True)
