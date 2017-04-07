from __future__ import division

import genpy
import tf
from twisted.internet import defer
from txros import util
from sub8 import Searcher
from sub8_ros_tools import pose_to_numpy, rosmsg_to_numpy
import numpy as np

SEARCH_DEPTH = .65
TIMEOUT_SECONDS = 60
@util.cancellableInlineCallbacks
def run(sub):
    print "MARKER MISSION - STARTING"

    # Wait for vision services, enable perception
    print "MARKER MISSION - WAITING FOR PERCEPTION SERVICES"
    rospy.wait_for_service('/vision/path_marker/pose', 3.0)
    rospy.wait_for_service('/vision/path_marker/enable', 3.0)
    enable = rospy.ServiceProxy('/vision/path_marker/enable', SetBool)
    enable(SetBoolRequest(data=True))

    pattern = []
    #pattern = [sub.move.right(1), sub.move.forward(1), sub.move.left(1), sub.move.backward(1),
     #          sub.move.right(2), sub.move.forward(2), sub.move.left(2), sub.move.backward(2)]
    s = Searcher(sub, sub.vision_proxies.path_marker.get_pose, pattern)
    resp = None
    print "MARKER MISSION - RUNNING SEARCH PATTERN, TIMEOUT={}".format(TIMEOUT_SECONDS)
    resp = yield s.start_search(loop=False, timeout=TIMEOUT_SECONDS)

    if resp is None:
        print "MARKER_MISSION - Marker not found."
        defer.returnValue(None)
    
    print "MARKER MISSION: MARKER POSE FOUND"
    print "MARKER MISSION: TRANFORMING MARKER POSE TO /map FROM {}".format(resp.pose.header.frame_id)
    cam_to_map = yield sub._tf_listener.get_transform('/map', '/'+resp.pose.header.frame_id)
    marker_position = cam_to_map.transform_point(rosmsg_to_numpy(resp.pose.pose.position))
    marker_orientation = cam_to_map.transform_quaternion(rosmsg_to_numpy(resp.pose.pose.orientation))
    move = sub.move.set_orientation(marker_orientation).zero_roll_and_pitch()
    position = marker_position.copy()
    position[2] = move._pose.position[2]
    move = move.set_position(position).go(speed=0.2)
    print "MARKER_MISSION: MOVING TO MARKER POSE"
    #yield move.go()
    print "MARKER_MISSION - DONE!"
    defer.returnValue(True)
