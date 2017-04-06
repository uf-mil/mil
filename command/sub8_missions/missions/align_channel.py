from __future__ import division

import genpy
import tf
from twisted.internet import defer
from txros import util
from sub8 import Searcher
from sub8_ros_tools import pose_to_numpy, rosmsg_to_numpy
import numpy as np

SEARCH_DEPTH = .65

@util.cancellableInlineCallbacks
def run(sub):
    print "MARKER MISSION - STARTING"
    # Do a little jig - change this for the pool.
    pattern = []
    #pattern = [sub.move.right(1), sub.move.forward(1), sub.move.left(1), sub.move.backward(1),
     #          sub.move.right(2), sub.move.forward(2), sub.move.left(2), sub.move.backward(2)]
    s = Searcher(sub, sub.vision_proxies.path_marker.get_pose, pattern)
    resp = None
    print "running pattern"
    resp = yield s.start_search(loop=False, timeout=60)

    if resp is None:
        print "MARKER_MISSION - Marker not found."
        defer.returnValue(None)
    print "MARKER_MISSION: transforing marker pose", resp.pose.header.frame_id
    cam_to_map = yield sub._tf_listener.get_transform('/map', '/'+resp.pose.header.frame_id)
    marker_position = cam_to_map.transform_point(rosmsg_to_numpy(resp.pose.pose.position))
    marker_orientation = cam_to_map.transform_quaternion(rosmsg_to_numpy(resp.pose.pose.orientation))
    move = sub.move.set_orientation(marker_orientation).zero_roll_and_pitch()
    position = marker_position.copy()
    position[2] = move._pose.position[2]
    move = move.set_position(position).go(speed=0.2)
    print "MARKER_MISSION: moving to ", marker_position, marker_orientation
    #yield move.go()
    print "MARKER_MISSION - Done!"
    defer.returnValue(True)
