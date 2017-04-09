from __future__ import division

import genpy
import tf
from twisted.internet import defer
from txros import util
from sub8 import Searcher
from sub8_ros_tools import pose_to_numpy, rosmsg_to_numpy
from sub8_misc_tools import text_effects
import numpy as np

SEARCH_DEPTH = .65
TIMEOUT_SECONDS = 60
MISSION="Align Path Marker"
@util.cancellableInlineCallbacks
def run(sub):
    print_info = text_effects.FprintFactory(title=MISSION)
    print_bad = text_effects.FprintFactory(title=MISSION, msg_color="red")
    print_good = text_effects.FprintFactory(title=MISSION, msg_color="green")
    print_info.fprint("STARTING")

    # Wait for vision services, enable perception
    print_info.fprint("ACTIVATING PERCEPTION SERVICE")
    sub.vision_proxies.path_marker.start()

    pattern = []
    #pattern = [sub.move.right(1), sub.move.forward(1), sub.move.left(1), sub.move.backward(1),
     #          sub.move.right(2), sub.move.forward(2), sub.move.left(2), sub.move.backward(2)]
    s = Searcher(sub, sub.vision_proxies.path_marker.get_pose, pattern)
    resp = None
    print_info.fprint("RUNNING SEARCH PATTERN")
    resp = yield s.start_search(loop=False, timeout=TIMEOUT_SECONDS)

    if resp is None:
        print_bad.fprint("MARKER NOT FOUND")
        defer.returnValue(None)
    
    print_good.fprint("PATH MARKER POSE FOUND")
    print_info.fprint("TRANFORMING MARKER POSE TO /map FROM {}".format(resp.pose.header.frame_id))
    cam_to_baselink = yield sub._tf_listener.get_transform('/base_link', '/'+resp.pose.header.frame_id)
    cam_to_map = yield sub._tf_listener.get_transform('/map', '/'+resp.pose.header.frame_id)
    marker_position = cam_to_map.transform_point(rosmsg_to_numpy(resp.pose.pose.position))
    marker_orientation = cam_to_map.transform_quaternion(rosmsg_to_numpy(resp.pose.pose.orientation))
    move = sub.move.set_orientation(marker_orientation).zero_roll_and_pitch()
    position = marker_position.copy()
    position[2] = move._pose.position[2]
    position[0] = position[0] + cam_to_baselink._p[0]
    position[1] = position[1] + cam_to_baselink._p[1]
    move = move.set_position(position)
    print_info.fprint("MOVING TO MARKER POSE")
    yield move.go(speed=0.2)
    print_good.fprint("ALIGNED TO PATH MARKER, DONE!")
    sub.vision_proxies.path_marker.stop()
    defer.returnValue(True)
