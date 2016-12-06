#!/usr/bin/env python
import txros
from twisted.internet import defer
import numpy as np
import navigator_tools
from navigator_tools import fprint, MissingPerceptionObject
from sensor_msgs.msg import PointCloud
import datetime


BF_WIDTH = 60.0  # m
BF_EST_COFIDENCE = 10.0  # How percisly can they place the waypoints? (m)
TOTEM_SAFE_DIST = 7  # How close do we go to the totem
ROT_SAFE_DIST = 3.5  # How close to rotate around it

@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    # rgb color map to param vlues
    color_map = {'BLUE': [0, 0, 1], 'RED': [1, 0, 0], 'YELLOW': [1, 1, 0], 'GREEN': [0, 1, 0]}

    explored_ids = []
    all_found = False

    # Get colors of intrest and directions
    c1 = navigator.mission_params['totem_color_1'].get()
    d1 = navigator.mission_params['totem_direction_1'].get()
    c2 = navigator.mission_params['totem_color_2'].get()
    d2 = navigator.mission_params['totem_direction_2'].get()
    
    colors = [c1, c2]
    directions = [d1, d2]

    buoy_field = yield navigator.database_query("BuoyField")
    buoy_field_point = navigator_tools.point_to_numpy(buoy_field.objects[0].position)

    _dist_from_bf = lambda pt: np.linalg.norm(buoy_field_point - pt)

    # We want to go to an observation point based on solar position
    # TODO: Check which side to go to based on relative distance
    center = navigator.move.set_position(buoy_field_point).set_orientation(get_solar_q())
    obs_point = center.backward(BF_WIDTH / 2 + BF_EST_COFIDENCE) 
    yield obs_point.go()

    # Next jig around to see buoys, first enforce that we're looking at the buoy field
    buoy_field_point[2] = 1
    #yield navigator.move.look_at(buoy_field_point).go(move_type='skid', focus=buoy_field_point)
    yield navigator.nh.sleep(3)
    #yield navigator.move.yaw_right(.5).go(move_type='skid')
    #yield navigator.nh.sleep(3)
    #yield navigator.move.yaw_left(1).go(move_type='skid')

    
    # TODO: What if we don't see the colors?
    for color, direction in zip(colors, directions):
        color = yield color
        direction = yield direction

        target = yield get_colored_buoy(navigator, color_map[color])
        if target is None or _dist_from_bf(navigator_tools.point_to_numpy(target.position)) > (BF_WIDTH / 2 + BF_EST_COFIDENCE):
            # Need to do something
            fprint("No suitable totems found.", msg_color='red', title="CIRCLE_TOTEM")
            defer.returnValue(None)

        target_np = navigator_tools.point_to_numpy(target.position)
        set_up = navigator.move.look_at(target_np).set_position(target_np).backward(TOTEM_SAFE_DIST)
        
        # Approach totem, making sure we actually get there.
        res = yield set_up.go(initial_plan_time=2)
        while res.failure_reason is not '':
            set_up = set_up.backward(.1)
            res = yield set_up.go(move_type='skid')
        
        fprint("Going {}".format(direction), title="CIRCLE_TOTEM")

        print TOTEM_SAFE_DIST - ROT_SAFE_DIST
        if direction == "COUNTER-CLOCKWISE":
            rot_move = navigator.move.yaw_right(1.57).left(TOTEM_SAFE_DIST - ROT_SAFE_DIST)
            while (yield rot_move.go(move_type='skid')).failure_reason is not '':
               rot_move = rot_move.right(.25)
               print rot_move

            circle = navigator.move.d_circle_point(target_np, radius=ROT_SAFE_DIST, direction='ccw', theta_offset=-1.57)
            for c in circle:
                yield c.go(move_type='skid')

        elif direction == "CLOCKWISE":
            rot_move = navigator.move.yaw_left(1.57).right(TOTEM_SAFE_DIST - ROT_SAFE_DIST)
            while (yield rot_move.go(move_type='skid')).failure_reason is not '':
               rot_move = rot_move.left(.25)
               print rot_move

            res = yield navigator.move.circle_point(target_np, direction='cw').go()

        print "Mission result:", res

        
    
    defer.returnValue(None)

@txros.util.cancellableInlineCallbacks
def get_colored_buoy(navigator, color):
    """
    Returns the closest colored buoy with the specified color
    """
    buoy_field = yield navigator.database_query("BuoyField")
    buoy_field_point = navigator_tools.point_to_numpy(buoy_field.objects[0].position)

    _dist_from_bf = lambda pt: np.linalg.norm(buoy_field_point - pt)

    totems = yield navigator.database_query("totem")
    correct_colored = [totem for totem in totems.objects if np.all(np.round(navigator_tools.rosmsg_to_numpy(totem.color, keys=['r', 'g', 'b'])) == color)]
    if len(correct_colored) == 0:
        closest = None 
    else:
        closest = sorted(correct_colored, key=lambda totem: _dist_from_bf(navigator_tools.point_to_numpy(totem.position)))[0]

    defer.returnValue(closest)

@txros.util.cancellableInlineCallbacks
def get_closest_buoy(navigator, explored_ids):
    pose = yield navigator.tx_pose
    buoy_field = yield navigator.database_query("BuoyField")
    buoy_field_np = navigator_tools.point_to_numpy(buoy_field.objects[0].position)

    # Find which totems we haven't explored yet
    totems = yield navigator.database_query("totem", raise_exception=False)
    if not totems.found:
        # Need to search for more totems
        defer.returnValue([None, explored_ids])

    u_totems = [totem for totem in totems.objects if totem.id not in explored_ids]
    u_totems_np = map(lambda totem: navigator_tools.point_to_numpy(totem.position), u_totems)

    if len(u_totems_np) == 0:
        defer.returnValue([None, explored_ids])

    # Find the closest buoys, favoring the ones closest to the boat.
    #   J = wa * ca ** 2 + wb * c2 ** 2
    #   a := boat
    #   b := buoy field ROI
    wa = .7
    wb = .3
    ca = np.linalg.norm(u_totems_np - pose[0], axis=1)
    cb = np.linalg.norm(u_totems_np - buoy_field_np, axis=1)
    J = wa * ca + wb * cb

    target_totem = u_totems[np.argmin(J)]
    explored_ids.append(target_totem.id)

    defer.returnValue([target_totem, explored_ids])

def get_solar_q():
    """Returns a quaternion to rotate to in order to keep the sun at our back"""
    now = datetime.datetime.now()
    now_time = now.time()
    if now_time < datetime.time(12, 00):
        # Sun in east, look west
        return [0, 0, 1, 0]
    else:
        # Sun in west, look east
        return [0, 0, 0, 1] 


