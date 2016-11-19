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


def _get_color(c):
    if c == 'r':
        return "RED"
    if c == 'b':
        return 'BLUE'
    if c == 'y':
        return 'YELLOW'
    if c == 'g':
        return 'GREEN'


@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    # attempts = kwargs["attempts"]
    """Main Script of Scan The Code."""
    # UNCOMMENT
    # navigator.change_wrench("autonomous")
    fprint("Moving to stc", msg_color='green')
    pub = yield navigator.nh.advertise("/stc/pose", PoseStamped)
    mission = ScanTheCodeMission(navigator)
    yield mission.init_(navigator.tf_listener)
    pose, look_at = yield mission.initial_position()
    initial_pose = navigator.move.set_position(pose).look_at(look_at)
    yield navigator.nh.sleep(1)
    _publish_pose(pub, initial_pose)
    fprint("Finished getting the initial position", msg_color='green')
    # UNCOMMENT
    # yield navigator.move.set_position(pose).look_at(look_at).go()
    # circle = navigator.move.circle_point(look_at, 8, granularity=30).go()
    # circle.addErrback(lambda x: x)

    yield mission.correct_pose(pose)

    # UNCOMMENT
    # circle.cancel()

    fprint("Finished getting the correct stc face", msg_color='green')
    colors = yield mission.find_colors()
    if colors is None:
        navigator.nh.set_param('mission/detect_deliver/Shape', 'CROSS')
        navigator.nh.set_param('mission/detect_deliver/Color', 'ANY')
    c1, c2, c3 = colors
    navigator.nh.set_param('mission/scan_the_code/color1', _get_color(c1))
    navigator.nh.set_param('mission/scan_the_code/color2', _get_color(c2))
    navigator.nh.set_param('mission/scan_the_code/color3', _get_color(c3))

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
