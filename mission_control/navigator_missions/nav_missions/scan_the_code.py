#!/usr/bin/env python
"""Scan The Code Mission Script."""
import txros
import numpy as np
from navigator_scan_the_code import ScanTheCodeMission
from geometry_msgs.msg import PoseStamped
import navigator_tools as nt
from navigator_tools import fprint, MissingPerceptionObject

___author___ = "Tess Bianchi"


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
    """Main Script of Scan The Code."""
    # attempts = kwargs["attempts"]
    # fprint("ATTEMPTS {}".format(attempts), msg_color="green")
    navigator.change_wrench("autonomous")
    fprint("Moving to stc", msg_color='green')
    mission = ScanTheCodeMission(navigator)
    yield mission.init_(navigator.tf_listener)
    pose, look_at = yield mission.initial_position()
    yield navigator.move.set_position(pose).look_at(look_at).go()
    yield navigator.nh.sleep(1)
    fprint("Finished getting the initial position", msg_color='green')

    # circle = navigator.move.circle_point(look_at).go()
    # circle.addErrback(lambda x: x)
    # yield mission.correct_pose()
    # circle.cancel()

    mission.correct_pose(pose)
    circle = navigator.move.d_circle_point(look_at, radius=8, granularity=30, direction='cw')
    # print list(circle)
    for p in list(circle)[::-1]:
        if mission.stc_correct:
            break
        yield p.go()

    fprint("Finished getting the correct stc face", msg_color='green')
    circle = navigator.move.d_circle_point(look_at, radius=8, granularity=30, direction='cw')
    colors = None
    for i in circle:
        defer = mission.find_colors()
        try:
            colors = yield txros.util.wrap_timeout(defer, 30)
            break
        except txros.util.TimeoutError:
            yield i.go()

    if colors is None:
        colors = "r", "g", "b"
    c1, c2, c3 = colors
    print colors
    yield navigator.mission_params["scan_the_code_color1"].set(_get_color(c1))
    yield navigator.mission_params["scan_the_code_color2"].set(_get_color(c2))
    yield navigator.mission_params["scan_the_code_color3"].set(_get_color(c3))


@txros.util.cancellableInlineCallbacks
def safe_exit(navigator, err):
    """Safe exit of the Scan The Code mission."""
    yield navigator.mission_params["scan_the_code_color1"].set("RED")
    yield navigator.mission_params["scan_the_code_color2"].set("GREEN")
    yield navigator.mission_params["scan_the_code_color3"].set("BLUE")
