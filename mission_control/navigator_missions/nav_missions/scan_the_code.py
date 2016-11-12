#!/usr/bin/env python
"""Scan The Code Mission Script."""
import txros
import numpy as np
from navigator_scan_the_code import ScanTheCodeMission
from geometry_msgs.msg import PoseStamped
import navigator_tools as nt
___author___ = "Tess Bianchi"


def _publish_pose(pub, pose):
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "enu"
    pose = nt.numpy_quat_pair_to_pose(pose.pose[0], pose.pose[1])
    pose_stamped.pose = pose
    pub.publish(pose_stamped)


@txros.util.cancellableInlineCallbacks
def main(navigator):
    print "sanasd"
    """Main Script of Scan The Code."""
    # UNCOMMENT
    # navigator.change_wrench("autonomous")

    pub = yield navigator.nh.advertise("/stc/pose", PoseStamped)
    mission = ScanTheCodeMission(navigator.nh)
    yield mission.init_(navigator.tf_listener)
    pose, look_at = yield mission.initial_position()
    initial_pose = navigator.move.set_position(pose).look_at(look_at)
    yield navigator.nh.sleep(1)
    _publish_pose(pub, initial_pose)

    # UNCOMMENT
    # yield navigator.move.set_position(pose).look_at(look_at).go()
    yield mission.correct_pose(pose)
    if not mission.stc_correct:
        circle = navigator.move.circle_point(look_at, 8, granularity=30)
        for p in circle:
            if mission.stc_correct:
                break
            # UNCOMMENT
            # yield p.go()

    colors = yield mission.find_colors()
    print colors
    if colors is None:
        navigator.nh.set_param('mission/detect_deliver/Shape', 'CROSS')
        navigator.nh.set_param('mission/detect_deliver/Color', 'ANY')
    if np.equal(colors, np.array(['r', 'g', 'b'])):
        navigator.nh.set_param('mission/detect_deliver/Shape', 'CIRCLE')
        navigator.nh.set_param('mission/detect_deliver/Color', 'ANY')
    else:
        navigator.nh.set_param('mission/detect_deliver/Shape', 'TRIANGLE')
        navigator.nh.set_param('mission/detect_deliver/Color', 'ANY')


@txros.util.cancellableInlineCallbacks
def safe_exit(navigator):
    """Safe exit of the Scan The Code mission."""
    navigator.nh.set_param('mission/detect_deliver/Shape', 'CROSS')
    navigator.nh.set_param('mission/detect_deliver/Color', 'ANY')
