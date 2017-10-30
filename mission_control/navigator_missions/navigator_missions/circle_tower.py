#!/usr/bin/env python
import txros
from twisted.internet import defer
from mil_misc_tools.text_effects import fprint
import mil_tools
from nav_msgs.msg import OccupancyGrid

import cv2
import numpy as np
import tf.transformations as trns

import datetime
from navigator import Navigator

BF_WIDTH = 60.0  # m
BF_EST_COFIDENCE = 10.0  # How percisly can they place the waypoints? (m)
TOTEM_SAFE_DIST = 6  # How close do we go to the totem
ROT_SAFE_DIST = 6  # How close to rotate around it
CIRCLE_OFFSET = 1.5  # To fix the circleness of the circle
SPEED_FACTOR = 3.0
DEFAULT_COLOR = "RED"


class CircleTotem(Navigator):
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        # rgb color map to param vlues
        color_map = {'BLUE': [0, 0, 1], 'RED': [1, 0, 0], 'YELLOW': [1, 1, 0], 'GREEN': [0, 1, 0]}
        directions = {'RED': 'cw', 'GREEN': 'ccw', 'BLUE': 'cw', 'YELLOW': 'ccw'}

        ogrid = OccupancyGridFactory(self)

        # Get colors of intrest and directions
        c1 = self.mission_params['scan_the_code_color1'].get()
        c2 = self.mission_params['scan_the_code_color2'].get()
        c3 = self.mission_params['scan_the_code_color3'].get()

        colors = [c1, c2, c3]

        buoy_field = yield self.database_query("BuoyField")
        buoy_field_point = mil_tools.rosmsg_to_numpy(buoy_field.objects[0].position)

        def _dist_from_bf(pt):
            return np.linalg.norm(buoy_field_point - pt)

        # We want to go to an observation point based on solar position
        # ??? There should be a .go here probably. Idk im just the pep8 guy
        self.move.set_position(buoy_field_point).set_orientation(get_solar_q())

        searched = []

        for color in colors:
            target = None
            need_recolor = False

            color = yield color
            direction = directions[color]

            fprint("Going to totem colored {} in direction {}".format(color, direction), title="CIRCLE_TOTEM")
            target = yield self.get_colored_buoy(color_map[color])
            dist = _dist_from_bf(mil_tools.rosmsg_to_numpy(target.position))
            if target is None or dist > (BF_WIDTH / 2 + BF_EST_COFIDENCE):
                # Need to do something
                fprint("No suitable totems found, going to circle any nearby totems.",
                       msg_color='red', title="CIRCLE_TOTEM")
                target, searched = yield self.get_closest_totem(searched)
                need_recolor = True

            target_np = mil_tools.rosmsg_to_numpy(target.position)
            circler = self.move.d_circle_point(point=target_np, radius=TOTEM_SAFE_DIST, direction=direction)

            # Give the totem a look at
            for p in circler:
                res = yield p.go(speed_factor=SPEED_FACTOR)
                if res.failure_reason is '':
                    break

            if need_recolor:
                fprint("Recoloring...")
                # Check for color?
                yield self.nh.sleep(5)
                target = yield self.database_query(str(target.id), raise_exception=False)
                if len(target.objects) == 0:
                    direction = directions[DEFAULT_COLOR]
                else:
                    direction = directions[check_color(target.objects[0], color_map)]

            mult = 1 if direction == 'cw' else -1
            left_offset = mult * CIRCLE_OFFSET

            tangent_circler = self.move.d_circle_point(
                point=target_np, radius=ROT_SAFE_DIST, theta_offset=mult * 1.57, direction=direction)

            # Point tangent
            for p in tangent_circler:
                res = yield p.go(speed_factor=SPEED_FACTOR)
                if res.failure_reason is '':
                    break

            msg, goal = ogrid.circle_around(target_np, direction=direction)
            latched_pub = ogrid.latching_publisher(msg)

            yield self.nh.sleep(2)
            goal = self.move.set_position(np.append(goal, 1)).look_at(
                self.pose[0]).left(left_offset).backward(2)
            yield goal.go(move_type='drive!', initial_plan_time=10)

            latched_pub.cancel()

            fprint("Canceling ogrid")
            yield self.nh.sleep(3)

            print "Mission result:", res

        defer.returnValue(None)

    @txros.util.cancellableInlineCallbacks
    def get_colored_buoy(self, color):
        """
        Returns the closest colored buoy with the specified color
        """
        buoy_field = yield self.database_query("BuoyField")
        buoy_field_point = mil_tools.rosmsg_to_numpy(buoy_field.objects[0].position)

        def _dist_from_bf(pt):
            return np.linalg.norm(buoy_field_point - pt)

        totems = yield self.database_query("totem")
        correct_colored = [totem for totem in totems.objects if np.all(
            np.round(mil_tools.rosmsg_to_numpy(totem.color, keys=['r', 'g', 'b'])) == color)]
        if len(correct_colored) == 0:
            closest = None
        else:
            closest = sorted(correct_colored, key=lambda totem: _dist_from_bf(
                mil_tools.rosmsg_to_numpy(totem.position)))[0]

        defer.returnValue(closest)

    @txros.util.cancellableInlineCallbacks
    def get_closest_totem(self, explored_ids):
        pose = yield self.tx_pose
        buoy_field = yield self.database_query("BuoyField")
        buoy_field_np = mil_tools.rosmsg_to_numpy(buoy_field.objects[0].position)

        # Find which totems we haven't explored yet
        totems = yield self.database_query("totem", raise_exception=False)
        if not totems.found:
            # Need to search for more totems
            defer.returnValue([None, explored_ids])

        u_totems = [totem for totem in totems.objects if totem.id not in explored_ids]
        u_totems_np = map(lambda totem: mil_tools.rosmsg_to_numpy(totem.position), u_totems)

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

    '''
    # UNUSED in main code. Not sure what it was for???
    @txros.util.cancellableInlineCallbacks
    def get_closest_object(self):
        pose = yield self.tx_pose
        buoy_field = yield self.database_query("BuoyField")
        buoy_field_np = mil_tools.rosmsg_to_numpy(buoy_field.objects[0].position)

        # Find which totems we haven't explored yet
        totems = yield self.database_query("all", raise_exception=False)
        if not totems.found:
            # Need to search for more totems
            defer.returnValue([None, explored_ids])

        u_totems = [totem for totem in totems.objects if totem.id not in explored_ids]
        u_totems_np = map(lambda totem: mil_tools.rosmsg_to_numpy(totem.position), u_totems)

        if len(u_totems_np) == 0:
            defer.returnValue([None, explored_ids])

        # Find the closest buoys, favoring the ones closest to the boat.
        #   J = wa * ca ** 2 + wb * c2 ** 2
        #   a := boat
        #   b := buoy field ROI
        wa = .6
        wb = .4
        ca = np.linalg.norm(u_totems_np - pose[0], axis=1)
        cb = np.linalg.norm(u_totems_np - buoy_field_np, axis=1)
        J = wa * ca + wb * cb

        target_totem = u_totems[np.argmin(J)]
        explored_ids.append(target_totem.id)

        defer.returnValue([target_totem, explored_ids])
     '''


def check_color(totem, color_map):
    for name, color in color_map:
        if mil_tools.rosmsg_to_numpy(totem.color, keys=['r', 'g', 'b']) == color:
            return name

    return DEFAULT_COLOR


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


class OccupancyGridFactory(object):
    def __init__(self, navigator):
        self.navigator = navigator
        self.resolution = 0.3
        self.buoy_buffer = 2  # m

        self.pub = self.navigator.nh.advertise("/mission_ogrid", OccupancyGrid)

    def circle_around(self, point, radius=15, direction='cw'):
        """ Returns an ogrid message and a waypoint to go to """
        point = np.array(point)
        fprint("Circling around {}".format(point))
        grid = np.zeros((2 * radius / self.resolution, 2 * radius / self.resolution))
        center = np.array([radius, radius]) / self.resolution
        target = self.orient_division(grid, point, -1 if direction == 'cw' else 1)

        # Outer circle
        cv2.circle(grid, tuple(center.astype(np.int32)), int(radius / self.resolution), 255, 3)

        # Buffer the buoy a bit
        cv2.circle(grid, tuple(center.astype(np.int32)), int(self.buoy_buffer / self.resolution), 255, -1)

        return [self.make_ogrid(grid, 2 * radius, point), target]

    def orient_division(self, grid, point, angle_offset_mult=1):
        """ Returns a position to go to on the other side of the line """
        angle_offset = 1 * angle_offset_mult  # rads

        point = np.array(point)
        r_pos = self.navigator.pose[0][:2] - point[:2]
        angle = np.arctan2(*r_pos[::-1]) - angle_offset

        center = np.array(grid.shape) / 2

        line_pt2 = trns.euler_matrix(0, 0, angle)[:3, :3].dot(np.array([1E3, 0, 0]))
        cv2.line(grid, tuple(center), tuple(line_pt2[:2].astype(np.int32)), 255, 2)

        # Let's put ourselves on the other side of that there line
        return self.generate_target(angle, r_pos) + point[:2]

    def generate_target(self, angle, r_pos):
        r = trns.euler_matrix(0, 0, angle)[:2, :2]
        r_i = np.linalg.inv(r)
        f = np.array([[1, 0], [0, -1]])
        return r.dot(f.dot(r_i.dot(r_pos[:2])))

    def make_ogrid(self, np_arr, size, center):
        np_center = np.array(center)
        np_origin = np.append((np_center - size / 2)[:2], 0)
        origin = mil_tools.numpy_quat_pair_to_pose(np_origin, [0, 0, 0, 1])

        ogrid = OccupancyGrid()
        ogrid.header = mil_tools.make_header(frame='enu')
        ogrid.info.origin = origin
        ogrid.info.width = size / self.resolution
        ogrid.info.height = size / self.resolution
        ogrid.info.resolution = self.resolution

        ogrid.data = np.clip(np_arr.flatten() - 1, -100, 100).astype(np.int8)
        return ogrid

    @txros.util.cancellableInlineCallbacks
    def latching_publisher(self, msg, prd=1):
        while True:
            msg.header.stamp = self.navigator.nh.get_time()
            self.pub.publish(msg)
            yield self.navigator.nh.sleep(prd)
