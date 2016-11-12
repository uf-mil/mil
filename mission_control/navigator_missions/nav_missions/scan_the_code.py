#!/usr/bin/env python
"""Scan The Code Mission Script."""
import txros
import numpy as np
from navigator_scan_the_code import ScanTheCodeMission
from geometry_msgs.msg import PoseStamped
import navigator_tools as nt
from navigator_tools import fprint, MissingPerceptionObject
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
    navigator.change_wrench("autonomous")
    fprint("Moving to stc", msg_color='green') 
    pub = yield navigator.nh.advertise("/stc/pose", PoseStamped)
    mission = ScanTheCodeMission(navigator.nh)
    yield mission.init_(navigator.tf_listener)
    pose, look_at = yield mission.initial_position()
    initial_pose = navigator.move.set_position(pose).look_at(look_at)
    yield navigator.nh.sleep(1)
    _publish_pose(pub, initial_pose)
    fprint("Finished getting the initial position", msg_color='green')
    # UNCOMMENT
    yield navigator.move.set_position(pose).look_at(look_at).go()
    myerr = mission.correct_pose(pose)
    if not mission.stc_correct:
        circle = navigator.move.circle_point(look_at, 8, granularity=30)
        for p in circle:
            if mission.stc_correct:
                break
            # UNCOMMENT
            yield p.go(move_type='skid')
    fprint("Finished getting the correct stc face", msg_color='green')
    colors = yield mission.find_colors()
    if colors is None:
        navigator.nh.set_param('mission/detect_deliver/Shape', 'CROSS')
        navigator.nh.set_param('mission/detect_deliver/Color', 'ANY')
    c1, c2, c3 = colors
    if c1 == 'r' and c2 == 'b' and c3 == 'g':
        navigator.nh.set_param('mission/detect_deliver/Shape', 'CIRCLE')
        navigator.nh.set_param('mission/detect_deliver/Color', 'ANY')
    else:
        navigator.nh.set_param('mission/detect_deliver/Shape', 'TRIANGLE')
        navigator.nh.set_param('mission/detect_deliver/Color', 'ANY')


@txros.util.cancellableInlineCallbacks
def safe_exit(navigator, err):
    """Safe exit of the Scan The Code mission."""
    navigator.nh.set_param('mission/detect_deliver/Shape', 'CROSS')
    navigator.nh.set_param('mission/detect_deliver/Color', 'ANY')
