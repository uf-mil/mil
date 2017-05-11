from __future__ import division

import genpy
import tf
import numpy as np
from twisted.internet import defer
from txros import util
from sub8 import Searcher
from mil_ros_tools import pose_to_numpy, rosmsg_to_numpy
from mil_misc_tools import text_effects
import numpy as np

SEARCH_DEPTH = .65
SEARCH_RADIUS_METERS = 1.0
TIMEOUT_SECONDS = 60
DO_PATTERN=True
SPEED = 0.5
MISSION="Align Path Marker"

forward_vec = np.array([1, 0, 0, 0])
@util.cancellableInlineCallbacks
def run(sub):
    print_info = text_effects.FprintFactory(title=MISSION).fprint
    print_bad = text_effects.FprintFactory(title=MISSION, msg_color="red").fprint
    print_good = text_effects.FprintFactory(title=MISSION, msg_color="green").fprint
    print_info("STARTING")

    # Wait for vision services, enable perception
    print_info("ACTIVATING PERCEPTION SERVICE")
    sub.vision_proxies.path_marker.start()

    pattern = []
    if DO_PATTERN:
        pattern = [sub.move.right(SEARCH_RADIUS_METERS), sub.move.forward(SEARCH_RADIUS_METERS),
                   sub.move.left(SEARCH_RADIUS_METERS), sub.move.backward(SEARCH_RADIUS_METERS),
                   sub.move.right(2.0*SEARCH_RADIUS_METERS), sub.move.forward(2.0*SEARCH_RADIUS_METERS),
                   sub.move.left(2.0*SEARCH_RADIUS_METERS), sub.move.backward(2.0*SEARCH_RADIUS_METERS)]
    s = Searcher(sub, sub.vision_proxies.path_marker.get_pose, pattern)
    resp = None
    print_info("RUNNING SEARCH PATTERN")
    resp = yield s.start_search(loop=False, timeout=TIMEOUT_SECONDS, spotings_req=1)

    if resp is None or not resp.found:
        print_bad("MARKER NOT FOUND")
        defer.returnValue(None)

    print_good("PATH MARKER POSE FOUND")
    assert(resp.pose.header.frame_id == "/map")

    move = sub.move
    position = rosmsg_to_numpy(resp.pose.pose.position)
    position[2] = move._pose.position[2] # Leave Z alone!
    orientation = rosmsg_to_numpy(resp.pose.pose.orientation)

    move = move.set_position(position).set_orientation(orientation).zero_roll_and_pitch()

    # Ensure SubjuGator continues to face same general direction as before (doesn't go in opposite direction)
    odom_forward = tf.transformations.quaternion_matrix(sub.move._pose.orientation).dot(forward_vec)
    marker_forward = tf.transformations.quaternion_matrix(orientation).dot(forward_vec)
    if np.sign(odom_forward[0]) != np.sign(marker_forward[0]):
        move = move.yaw_right(np.pi)

    print_info("MOVING TO MARKER AT {}".format(move._pose.position))
    yield move.go(speed=SPEED)
    print_good("ALIGNED TO PATH MARKER. MOVE FORWARD TO NEXT CHALLENGE!")
    sub.vision_proxies.path_marker.stop()
    defer.returnValue(True)
