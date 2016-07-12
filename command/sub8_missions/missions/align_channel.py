from __future__ import division

import tf
from sub8 import Searcher
from sub8_ros_tools import pose_to_numpy
import numpy as np

SEARCH_DEPTH = .65

@util.cancellableInlineCallbacks
def run(sub):
    # Go to max height to see as much as we can.
    yield sub.move.depth(SEARCH_DEPTH).zero_roll_and_pitch().go()
    yield sub._node_handle.sleep(1.0)

    # Sit still for now - no searching
    pattern = [np.array([0, 0, 0])]
    s = Searcher(sub, sub.channel_marker.get_pose, pattern)
    resp = yield s.start_search(loop=True)
    if resp is None:
        print "MARKER_MISSION - ERROR."
        return
    yield sub.move.set_position(pose_to_numpy(resp.pose.pose)[0])

    # How many times should we attempt to reposition ourselves
    iterations = 3
    # To make sure we don't go too far off.
    est_target_rotations = []
    for i in range(iterations):
        print "Iteration {}.".format(i + 1)
        response = yield sub.channel_marker.get_pose()
        
        if resp is None:
            continue

        response = pose_to_numpy(response.pose.pose)

        # Using euler - shoot me.
        yaw = tf.transformations.euler_from_quaternion(response[1])[2]

        est_target_rotations.append(yaw)
        avg_rotation = sum(est_target_rotations) / len(est_target_rotations)

        yield sub.move.set_position(response[0]).yaw_left(yaw).zero_roll_and_pitch().go()

        yield sub._node_handle.sleep(3.0)

    print "MARKER_MISSION - Done!"