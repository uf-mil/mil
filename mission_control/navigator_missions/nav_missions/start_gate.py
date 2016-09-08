#!/usr/bin/env python
import txros
import tf
import numpy as np
import navigator_tools
from twisted.internet import defer

@txros.util.cancellableInlineCallbacks
def main(navigator):
    '''
    Once we see the gate - move to a point 10m in front of the gate.
    When we get there, make another observation of the gate to make
        sure we are lined up and then go through.

    '''
    navigator.change_wrench("autonomous")
    # Only try to get the point 5 times
    for _try in range(5):
        print "Waiting for points..."
        m = yield navigator.vision_request("start_gate")
        print m

        if not m.success:
            continue

        p, q = navigator_tools.pose_to_numpy(m.target.pose)

        # Move backwards a bit from the waypoint
        target_r = tf.transformations.quaternion_matrix(q)
        back_up = target_r.dot(np.array([-10, 0, 0,  1]))  # Homogeneous
        target_p = p + back_up[:3]

        yield navigator.move.set_orientation(target_r).go()
        yield navigator.move.set_position(target_p).go()

        break

    if not m.success:
        print "No objects found. Exiting."
        # Raise an alarm
        defer.returnValue(False)

    for _try in range(5):
        print "Waiting for points..."
        m = yield navigator.vision_request("start_gate")
        print m

        if not m.success:
            continue

        p, q = navigator_tools.pose_to_numpy(m.target.pose)

        # Average the two rotation and positions together to get a "better" guess
        weight_factor = .8  # Weight for the newer data over the old stuff
        target_r = weight_factor * tf.transformations.quaternion_matrix(q) + \
                   (1 - weight_factor) * target_r
        target_p = weight_factor * p + \
                   (1 - weight_factor) * target_p

        move_through = target_r.dot([10, 0, 0,  1])
        target_p += move_through[:3]

        yield navigator.move.set_orientation(target_r).go()
        yield navigator.move.set_position(target_p).go()

        break
