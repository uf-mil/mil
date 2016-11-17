#!/usr/bin/env python
import txros
from twisted.internet import defer
import numpy as np
import navigator_tools
from navigator_tools import fprint, MissingPerceptionObject
from sensor_msgs.msg import PointCloud
import datetime

@txros.util.cancellableInlineCallbacks
def main(navigator):
    res = navigator.fetch_result()

    buoy_field = yield navigator.database_query("BuoyField")
    buoy_field_point = navigator_tools.point_to_numpy(buoy_field.objects[0])

    #yield navigator.move.set_position(buoy_field_point).go()

    circle_colors = ['blue', 'red']
    color_map = {'blue': (255, 0, 0), 'red': (0, 0, 255)}

    explored_ids = []
    all_found = False

    while not all_found:
        target_totem, explored_ids = yield get_closest_buoy(navigator, explored_ids)

        if target_totem is None:
            fprint("No suitable totems found.", msg_color='red', title="CIRCLE_TOTEM")
            continue

        # Visualization
        points = [target_totem.position]
        pc = PointCloud(header=navigator_tools.make_header(frame='/enu'),
                        points=points)
        yield navigator._point_cloud_pub.publish(pc)

        # Let's go there
        target_distance = 7  # m
        target_totem_np = navigator_tools.point_to_numpy(target_totem.position)
        q = get_sun_angle()
        lookat = navigator.move.set_position(target_totem_np).set_orientation(q).backward(target_distance)
        yield lookat.go()

        # Now that we're looking him in the eyes, aim no higher.
        # Check the color and see if it's one we want.
        fprint("Color request", title="CIRCLE_TOTEM")

        #if target_totem is not None:
        #   all_found = True
            
    defer.returnValue(res)

    pattern = navigator.move.circle_point(focus, radius=5)

    for p in pattern:
        yield p.go(move_type='skid', focus=focus)
        print "Nexting"

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

def get_sun_angle():
    """Returns a quaternion to rotate to in order to keep the sun at our back"""
    now = datetime.datetime.now()
    now_time = now.time()
    if now_time < datetime.time(12, 00):
        # Sun in east, look west
        return [0, 0, 1, 0]
    else:
        # Sun in west, look east
        return [0, 0, 0, 1] 


