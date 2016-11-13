#!/usr/bin/env python
from __future__ import division

import txros
import tf
import tf.transformations as trns
import numpy as np
import navigator_tools
from navigator_tools import fprint, MissingPerceptionObject
from navigator_singleton import pose_editor
from twisted.internet import defer

WEST = trns.quaternion_matrix(pose_editor.WEST)
EAST = trns.quaternion_matrix(pose_editor.EAST)
NORTH = trns.quaternion_matrix(pose_editor.NORTH)
SOUTH = trns.quaternion_matrix(pose_editor.SOUTH)

@txros.util.cancellableInlineCallbacks
def main(navigator):
    result = navigator.fetch_result()

    #middle_point = np.array([-10, -70, 0]) 
    est_coral_survey = yield navigator.database_query("Coral_Survey")
    if not est_coral_survey.found:
        raise MissingPerceptionObject("Coral_Survey")
    
    yield navigator.move.set_position(est_coral_survey.objects[0]).go()

    totem = yield navigator.database_query("totem")
    if not totem.found:
        raise MissingPerceptionObject("totem")
    
    # Get the closest totem object to the boat
    totem_np = map(lambda obj: navigator_tools.point_to_numpy(obj), totem.objects)
    dist = map(lambda totem_np: np.linalg.norm(totem_np - navigator_tools.point_to_numpy(est_coral_survey.objects[0])), totems_np)
    
    # Complain if the buoy is too far away
    if not min(dist) > 20:  # m
        raise MissingPerceptionObject("totem")
    
    middle_point = navigator_tools.point_to_numpy(totem.objects[0].position)
    quads_to_search = [1, 2, 3, 4]
    if (yield navigator.nh.has_param("/mission/coral_survey/quadrants")):
        quads_to_search = yield navigator.nh.get_param("/mission/coral_survey/quadrants")

    waypoint_from_center = np.array([10 * np.sqrt(2)])

    # Construct waypoint list along NSEW directions then rotate 45 degrees to get a good spot to go to.
    directions = [EAST, NORTH, WEST, SOUTH]
    waypoints = []
    for quad in quads_to_search:
        mid = navigator.move.set_position(middle_point).set_orientation(directions[quad - 1])
        waypoints.append(mid.yaw_left(45, "deg").forward(waypoint_from_center).set_orientation(NORTH))

    # Get into the coral survey area
    yield waypoints[0].go()

    # Publish ogrid with boundaries to stay inside
    ogrid = OgridFactory(middle_point, draw_borders=True)
    msg = ogrid.get_message()
    latched = navigator.latching_publisher("/mission_ogrid", OccupancyGrid, msg)

    searcher = navigator.search("coral_survey", waypoints)
    yield searcher.start_search(move_type='skid', spotings_req=1)

    fprint("Centering over the thing!", title="CORAL_SURVEY")

    # TODO: Center over the thing.

    boat_position = (yield navigator.tx_pose)[0]

    # TODO: make this easier
    quad = np.argmin(np.linalg.norm(boat_position - [[w.pose[0][0][0], w.pose[0][1][0],w.pose[0][2][0]] for w in waypoints], axis=1))
    quad = quads_to_search[quad]
    fprint("Estimated quadrant: {}".format(quad), title="CORAL_SURVEY", msg_color='green')

    yield navigator.nh.sleep(5)
    defer.returnValue(result)


# This really shouldn't be here - it should be somewhere behind the scenes
from nav_msgs.msg import OccupancyGrid
import cv2

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
        self.origin = navigator_tools.numpy_quat_pair_to_pose([origin_x, origin_y, 0],
                                                              [0, 0, 0, 1])

        # The grid needs to have it's axes swaped since its row major
        self.grid = np.zeros((self.height / self.resolution, self.width / self.resolution)) - 1

        # Transforms points from ENU to ogrid frame coordinates
        self.t = np.array([[1 / self.resolution, 0, -origin_x / self.resolution],
                           [0, 1 / self.resolution, -origin_y / self.resolution],
                           [0,               0,            1]])

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
        ogrid.header = navigator_tools.make_header(frame="enu")
        ogrid.info.resolution = self.resolution
        ogrid.info.height, ogrid.info.width = self.grid.shape
        ogrid.info.origin = self.origin
        grid = np.copy(self.grid)
        ogrid.data = np.clip(grid.flatten(), -100, 100).astype(np.int8)

        return ogrid
