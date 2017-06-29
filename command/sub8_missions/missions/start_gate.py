#!/usr/bin/env python
from __future__ import division

import txros
import txros.tf as tf
from twisted.internet import defer

from mil_misc_tools import text_effects

from sub8 import Searcher
from mil_ros_tools import rosmsg_to_numpy

import numpy as np

fprint = text_effects.FprintFactory(
    title="START_GATE", msg_color="cyan").fprint

FACTOR_DISTANCE_BEFORE = 0.5
FACTOR_DISTANCE_AFTER = 1
SPEED = 0.3


@txros.util.cancellableInlineCallbacks
def run(sub):
    yield sub.vision_proxies.start_gate.start()

    search_pattern = []
    search = Searcher(
        sub,
        sub.vision_proxies.start_gate.get_pose,
        search_pattern)

    resp = None
    fprint('Searching...')
    resp = yield search.start_search(loop=False, timeout=100, spotings_req=1)

    if resp is None or not resp.found:
        fprint("No gate found...", msg_color="red")
        defer.returnValue(None)

    position = rosmsg_to_numpy(resp.pose.pose.position)
    orientation = rosmsg_to_numpy(resp.pose.pose.orientation)
    fprint('Gate\'s position in map is: {}'.format(position))
    fprint('Gate\'s orientation in map is: {}'.format(orientation))

    normal = tf.transformations.quaternion_matrix(
        orientation).dot(np.array([1, 0, 0, 0]))[0:3]
    fprint('Computed normal vector: {}'.format(normal))

    point_before = np.array(
        [position[0], position[1], position[2]]) + FACTOR_DISTANCE_BEFORE * normal
    fprint('Moving infront of gate {}'.format(point_before))
    sub.move.set_position(point_before).go(speed=SPEED)

    point_after = np.array(
        [position[0], position[1], position[2]]) - FACTOR_DISTANCE_AFTER * normal
    fprint('YOLO! Going through gate {}'.format(point_after))
    sub.move.set_position(point_after).go(speed=SPEED)

    defer.returnValue(True)
