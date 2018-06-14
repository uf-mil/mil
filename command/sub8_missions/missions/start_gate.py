#!/usr/bin/env python
from __future__ import division

import txros
from twisted.internet import defer

from mil_misc_tools import text_effects

from sub8 import SonarObjects
from mil_ros_tools import rosmsg_to_numpy
from scipy.spatial import distance

import numpy as np

fprint = text_effects.FprintFactory(
    title="START_GATE", msg_color="cyan").fprint

SPEED = 0.3
# How many meters to pass the gate by
DIST_AFTER_GATE = 3


@txros.util.cancellableInlineCallbacks
def run(sub):
    fprint('Begin search for gates')
    rotate_start = sub.move.zero_roll_and_pitch()
    for i in range(4):
        # Search 4 quadrants deperated by 90 degrees for the gate
        fprint('Searching {} degrees'.format(90 * i))
        yield rotate_start.yaw_right_deg(90 * i).go(speed=SPEED)
        start = sub.move.zero_roll_and_pitch()
        # Pitch up and down to populate pointcloud
        so = SonarObjects(sub, [start.pitch_down_deg(7), start] * 10)
        transform = yield sub._tf_listener.get_transform('/map', '/base_link')
        # [1, 0, 0] is front vector for sub
        ray = transform._q_mat.dot(np.array([1, 0, 0]))
        # Start scan and search
        res = yield so.start_search_in_cone(
            transform._p,
            ray,
            angle_tol=60,
            distance_tol=11,
            speed=0.1,
            clear=True)
        fprint('Found {} objects'.format(len(res.objects)))
        # Didn't find enough objects
        if len(res.objects) < 2:
            fprint('No objects')
            del so
            continue
        # Search for two objects that satisfy the gate
        gate_points = find_gate(res.objects, ray)
        if gate_points is None:
            fprint('No valid gates')
            del so
            continue
        # Break if all the checks passed for gate
        break
    if gate_points is None:
        fprint('Returning')
        yield rotate_start.go()
        defer.returnValue(False)
    # Find midpoint between the two poles/objects
    mid_point = gate_points[0] + gate_points[1]
    mid_point = mid_point / 2
    # Offset z so we don't hit the bar
    mid_point[2] = mid_point[2] - 0.5
    fprint('Midpoint: {}'.format(mid_point))
    yield sub.move.look_at(mid_point).go(speed=SPEED)
    fprint('Moving!', msg_color='yellow')
    yield sub.move.set_position(mid_point).go(speed=SPEED)
    normal = mid_point - sub.pose.position
    normal[2] = 0
    normal = normal / np.linalg.norm(normal)
    fprint('Normal {} '.format(normal))
    fprint('Moving past the gate', msg_color='yellow')
    yield sub.move.set_position(mid_point + DIST_AFTER_GATE * normal).go(
        speed=SPEED)
    defer.returnValue(True)


def find_gate(objects,
              ray,
              max_distance_away=4.5,
              perp_threshold=0.5,
              depth_threshold=1):
    '''
    find_gate: search for two objects that satisfy critria

    Parameters:
    ray: direction we expect gate to be near
    max_distance_away: max distance the two objects can be away from each other
    perp_threshold: max dot product value for perpindicular test with ray
    depth_threshold: make sure the two objects have close enough depth
    '''
    for o in objects:
        p = rosmsg_to_numpy(o.pose.position)
        for o2 in objects:
            if o2 is o:
                continue
            p2 = rosmsg_to_numpy(o2.pose.position)
            print('Distance {}'.format(distance.euclidean(p, p2)))
            if distance.euclidean(p, p2) > max_distance_away:
                print('far away')
                continue
            line = p - p2
            perp = line.dot(ray)
            perp = perp / np.linalg.norm(perp)
            print('Dot {}'.format(perp))
            if not (-perp_threshold <= perp <= perp_threshold):
                print('perp threshold')
                # continue
            print('Dist {}'.format(line))
            if abs(line[2] > depth_threshold):
                print('not same height')
                continue
            if abs(line[0]) < 1 and abs(line[1]) < 1:
                print('on top of each other')
                continue
            return (p, p2)
    return None
