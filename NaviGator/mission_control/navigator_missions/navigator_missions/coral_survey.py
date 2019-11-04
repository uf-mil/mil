#!/usr/bin/env python
from __future__ import division

import txros
import tf.transformations as trns
import numpy as np
import mil_tools
from random import shuffle
from navigator import Navigator
import pose_editor
from twisted.internet import defer
from nav_msgs.msg import OccupancyGrid
import cv2

WEST = trns.quaternion_matrix(pose_editor.WEST)
EAST = trns.quaternion_matrix(pose_editor.EAST)
NORTH = trns.quaternion_matrix(pose_editor.NORTH)
SOUTH = trns.quaternion_matrix(pose_editor.SOUTH)

# What should we pick this time?
shapes = ['CIRCLE', 'TRIANGLE', 'CROSS']
shuffle(shapes)


class CoralSurvey(Navigator):
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        # middle_point = np.array([-10, -70, 0])
        est_coral_survey = yield self.database_query("CoralSurvey")

        # Going to get all the objects and using the closest one as the totem
        totem = yield self.database_query("all")

        # Get the closest totem object to the boat
        totems_np = map(lambda obj: mil_tools.rosmsg_to_numpy(obj.position), totem.objects)
        dist = map(lambda totem_np: np.linalg.norm(
            totem_np - mil_tools.rosmsg_to_numpy(est_coral_survey.objects[0].position)), totems_np)
        middle_point = totems_np[np.argmin(dist)]

        print "Totem sorted:", totems_np
        print "Totem selected: ", totems_np[0]
        quad = yield self.mission_params["acoustic_pinger_active_index_correct"].get()

        waypoint_from_center = np.array([10 * np.sqrt(2)])

        # Publish ogrid with boundaries to stay inside
        ogrid = OgridFactory(middle_point, draw_borders=True)
        msg = ogrid.get_message()
        latched = self.latching_publisher("/mission_ogrid", OccupancyGrid, msg)

        # Construct waypoint list along NSEW directions then rotate 45 degrees to get a good spot to go to.
        directions = [EAST, NORTH, WEST, SOUTH]
        # for quad in quads_to_search:
        mid = self.move.set_position(middle_point).set_orientation(directions[quad - 1])
        search_center = mid.yaw_left(45, "deg").forward(waypoint_from_center).set_orientation(NORTH)
        yield search_center.left(6).go()
        yield self.move.circle_point(search_center.position, direction='cw').go()

        yield self.mission_params["coral_survey_shape1"].set(shapes[0])
        latched.cancel()
        defer.returnValue(None)

# This really shouldn't be here - it should be somewhere behind the scenes


class OgridFactory():
    def __init__(self, center, draw_borders=False):
        self.resolution = .3
        self.height, self.width = 60, 60  # meters
        self.center = center

        self.wall_length = 20  # meters
        self.walls = None

        # Sets x,y upper and lower bounds and the left and right wall bounds
        self.make_ogrid_transform()
        self.make_walls()
        self.draw_lines(self.walls, -2)

        if draw_borders:
            self.draw_borders()

    def draw_borders(self):
        borders = ((-1, -1), (self.grid.shape))
        cv2.rectangle(self.grid, borders[0], borders[1], 100, 3)

    def make_ogrid_transform(self):
        origin_x = self.center[0] - 30
        origin_y = self.center[1] - 30
        self.origin = mil_tools.numpy_quat_pair_to_pose([origin_x, origin_y, 0],
                                                        [0, 0, 0, 1])

        # The grid needs to have it's axes swaped since its row major
        self.grid = np.zeros((self.height / self.resolution, self.width / self.resolution)) - 1

        # Transforms points from ENU to ogrid frame coordinates
        self.t = np.array([[1 / self.resolution, 0, -origin_x / self.resolution],
                           [0, 1 / self.resolution, -origin_y / self.resolution],
                           [0, 0, 1]])

        self.transform = lambda point: self.t.dot(np.append(point[:2], 1))[:2]

    def make_walls(self):
        vect = np.array([self.wall_length, 0, 0])

        # Dotting with the negatives seemed to produce cleaner ogrids
        west_point = EAST[:3, :3].dot(-vect) + self.center
        east_point = EAST[:3, :3].dot(vect) + self.center
        north_point = NORTH[:3, :3].dot(vect) + self.center
        south_point = NORTH[:3, :3].dot(-vect) + self.center

        self.walls = [self.center, west_point,
                      self.center, east_point,
                      self.center, north_point,
                      self.center, south_point]

    def draw_lines(self, points, value):
        last_wall = None
        for wall in points:
            if last_wall is None:
                last_wall = tuple(self.transform(wall).astype(np.int32))
                continue

            this_wall = tuple(self.transform(wall).astype(np.int32))
            cv2.line(self.grid, this_wall, last_wall, value, 1)
            last_wall = this_wall

    def get_message(self):
        ogrid = OccupancyGrid()
        ogrid.header = mil_tools.make_header(frame="enu")
        ogrid.info.resolution = self.resolution
        ogrid.info.height, ogrid.info.width = self.grid.shape
        ogrid.info.origin = self.origin
        grid = np.copy(self.grid)
        ogrid.data = np.clip(grid.flatten(), -100, 100).astype(np.int8)

        return ogrid
