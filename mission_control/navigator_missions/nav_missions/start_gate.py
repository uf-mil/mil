#!/usr/bin/env python
from __future__ import division

import txros
import tf
import tf.transformations as trns
import numpy as np
import navigator_tools
from sensor_msgs.msg import PointCloud
from twisted.internet import defer
from navigator_msgs.srv import ObjectDBSingleQuery, ObjectDBSingleQueryRequest


class Buoy(object):
    @classmethod
    def from_srv(cls, srv):
        obj = srv.object
        return cls(obj.position, "Black")

    def __init__(self, position, color):
        self.position = np.array(position)
        self.color = color
        self.odom = np.array([0, 0, 0])

    def __repr__(self):
        return "BUOY at: {} with color {}".format(self.position, self.color)

    def distance(self, odom):
        dist = np.linalg.norm(self.position - odom)
        #print dist
        return dist


def get_buoys():
    right = Buoy([0, 7, 0], "green")
    left = Buoy([20, 7, 0], "red")

    #rand_1 = Buoy([-23, 48, 0], "green")
    #rand_2 = Buoy([33, -8, 0], "green")

    buoys = [right, left] #[rand_1, right, rand_2, left]

    return buoys


def get_path(buoy_l, buoy_r):
    # Vector between the two buoys
    between_vector = buoy_r.position - buoy_l.position

    # Rotate that vector to point through the buoys
    r = trns.euler_matrix(0, 0, np.radians(90))[:3, :3]
    direction_vector = r.dot(between_vector)
    direction_vector /= np.linalg.norm(direction_vector)

    return between_vector, direction_vector


return_with = defer.returnValue

@txros.util.cancellableInlineCallbacks
def main(navigator):
    '''
    This assumes we have ALL FOUR buoys
    '''
    result = navigator.fetch_result()

    #navigator.change_wrench("autonomous")
    buoys = get_buoys()
    #serv = navigator.nh.get_service_client("/database/single", ObjectDBSingleQuery)
    #req = ObjectDBSingleQueryRequest()
    #req.name = 'l_gate'
    #left = yield serv(req)
    #req.name = 'r_gate'
    #right = yield serv(req)
    #buoys = [Buoy.from_srv(left), Buoy.from_srv(right)]
    print buoys
    # Display points of the buoys]
    points = [navigator_tools.numpy_to_point(b.position) for b in buoys]

    pose = yield navigator.tx_pose
    buoys = sorted(buoys, key=lambda buoy: buoy.distance(pose[0]))

    # print pose
    # print buoys

    print "\nSTART_GATE: Be advised... Doing checks"

    # See if the closest two buoys are the right colors ============================================
    if not (buoys[0].color in ['green', 'red'] and buoys[1].color in ['green', 'red']):
        print "START_GATE: Error 1 - Not the right colors. ({},{})".format(buoy[0].color,
                                                                           buoy[1].color)
        result.success = False
        #return_with(result)

    # Okay, now we need to see if the buoys are approx 10m apart ===================================
    nominal_dist = 10  # m
    variance = 5  #m
    dist_btwn_buoys = np.linalg.norm(buoys[0].position[:2] - buoys[1].position[:2])
    if not (nominal_dist - variance < dist_btwn_buoys < nominal_dist + variance):
        print "START_GATE: Error 2 - Not the right distance apart. {}".format(dist_btwn_buoys)
        #result.success = False
        #return_with(result)

    # Made it this far, make sure the red one is on the left and green on the right ================
    #t = yield navigator.tf_listener.get_transform("/base_link", "/enu", navigator.nh.get_time())
    class t(object):
        # Temp
        @classmethod
        def transform_point(self, point):
            return point
    bl_buoys = [t.transform_point(buoy.position) for buoy in buoys[:2]]

    # Angles are w.r.t positive x-axis. Positive is CCW around the z-axis.
    angle_buoys = np.array([np.arctan2(buoy[1], buoy[0]) for buoy in bl_buoys])
    left, right = buoys[np.argmax(angle_buoys)], buoys[np.argmin(angle_buoys)]
    if left.color == "green" and right.color == "red":
        print "START_GATE: Error 3 - Not the right colors. (l-{}, r-{})".format(left.color,
                                                                                right.color)
        #result.success = False
        #return_with(result)

    # Lets get a path to go to
    between_vector, direction_vector = get_path(left, right)
    mid_point = left.position + between_vector / 2

    #print mid_point
    setup = mid_point - direction_vector * 20
    target = setup + direction_vector * 90

    ogrid = OgridFactory(left.position, right.position, pose[0], target)
    msg = ogrid.get_message()

    # Put the target into the point cloud as well
    points.append(navigator_tools.numpy_to_point(target))
    pc = PointCloud(header=navigator_tools.make_header(frame='/enu'),
                    points=np.array(points))
    for i in range(50):
        yield navigator._point_cloud_pub.publish(pc)

    print "publishing"
    latched = navigator.latching_publisher("/mission_ogrid", OccupancyGrid, msg)
    print latched

    yield navigator.nh.sleep(5)

    print "START_GATE: Moving!"

    yield navigator.move.set_position(target).go()

    return_with(result)


# This really shouldn't be here - it should be somewhere behind the scenes
from nav_msgs.msg import OccupancyGridBut
import cv2

class OgridFactory():
    def __init__(self, left_f_pos, right_f_pos, boat_pos, target, left_b_pos=None, right_b_pos=None):
        # Front buoys
        self.left_f = left_f_pos
        self.right_f = right_f_pos
        # Back buoys
        self.left_b = left_b_pos
        self.right_b = right_b_pos
        # Other info
        self.boat_pos = boat_pos
        self.target = target

        # Some parameters
        self.buffer = 20  # length of the "walls" extending outwards from each buoy (m)
        self.resolution = 10  # cells/meter for ogrid
        self.channel_length = 30  # Not sure what this acutally is for the course (m)

        # Sets x,y upper and lower bounds and the left and right wall bounds
        self.get_size_and_build_walls()

        self.origin = navigator_tools.numpy_quat_pair_to_pose([self.x_lower, self.y_lower, 0],
                                                              [0, 0, 0, 1])
        dx = self.x_upper - self.x_lower
        dy = self.y_upper - self.y_lower
        # The grid needs to have it's axes swaped since its row major
        self.grid = np.zeros((dy * self.resolution, dx * self.resolution))

        # Transforms points from ENU to ogrid frame coordinates
        self.t = np.array([[self.resolution, 0, -self.x_lower * self.resolution],
                           [0, self.resolution, -self.y_lower * self.resolution],
                           [0,               0,            1]])
        self.transform = lambda point: self.t.dot(np.append(point[:2], 1))

        self.draw_walls()

    def get_size_and_build_walls(self):
        # Get size of the ogrid ==============================================
        # Get some useful vectors
        between_vector = self.left_f - self.right_f
        mid_point = self.right_f + between_vector / 2
        target_vector = self.target - mid_point
        self.mid_point = mid_point

        # For rotations of the `between_vector` and the enu x-axis
        b_theta = np.arctan2(between_vector[1], between_vector[0])
        b_rot_mat = trns.euler_matrix(0, 0, b_theta)[:3, :3]

        # Make the endpoints
        rot_buffer = b_rot_mat.dot([self.buffer, 0, 0])
        endpoint_left_f = self.left_f + rot_buffer
        endpoint_right_f = self.right_f - rot_buffer

        # Define bounds for the grid
        self.x_lower = min(self.boat_pos[0], endpoint_left_f[0], endpoint_right_f[0], self.target[0])
        self.x_upper = max(self.boat_pos[0], endpoint_left_f[0], endpoint_right_f[0], self.target[0])
        self.y_lower = min(self.boat_pos[1], endpoint_left_f[1], endpoint_right_f[1], self.target[1])
        self.y_upper = max(self.boat_pos[1], endpoint_left_f[1], endpoint_right_f[1], self.target[1])

        # Now lets build some wall points ======================================

        if self.left_b is None:
            # For rotations of the `target_vector` and the enu x-axis
            t_theta = np.arctan2(target_vector[1], target_vector[0])
            t_rot_mat = trns.euler_matrix(0, 0, t_theta)[:3, :3]

            rot_channel = t_rot_mat.dot([self.channel_length, 0, 0])
            self.left_b = self.left_f + rot_channel
            self.right_b = self.right_f + rot_channel

        endpoint_left_b = self.left_b + rot_buffer
        endpoint_right_b = self.right_b - rot_buffer

        # These are in ENU by the way
        self.left_wall_points = [self.left_f, self.left_b, endpoint_left_b, endpoint_left_f]
        self.right_wall_points = [self.right_f, self.right_b, endpoint_right_b, endpoint_right_f]

    def draw_walls(self):
        left_wall_points = np.array([self.transform(point) for point in self.left_wall_points])
        right_wall_points = np.array([self.transform(point) for point in self.right_wall_points])

        rect = cv2.minAreaRect(left_wall_points[:,:2].astype(np.float32))
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(self.grid, [box], 0, 128, -1)

        rect = cv2.minAreaRect(right_wall_points[:,:2].astype(np.float32))
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(self.grid, [box], 0, 128, -1)

        # So I dont have to comment abunch of stuff out for debugging
        dont_display = True
        if dont_display:
            return

        # Bob Ross it up (just for display)
        left_f, right_f = self.transform(self.left_f), self.transform(self.right_f)
        left_b, right_b = self.transform(self.left_b), self.transform(self.right_b)

        boat = self.transform(self.boat_pos)
        target = self.transform(self.target)

        cv2.circle(self.grid, tuple(boat[:2].astype(np.int32)), 8, 255)
        cv2.circle(self.grid, tuple(target[:2].astype(np.int32)), 15, 255)
        cv2.circle(self.grid, tuple(self.transform(self.mid_point)[:2].astype(np.int32)), 5, 255)
        cv2.circle(self.grid, tuple(left_f[:2].astype(np.int32)), 10, 255)
        cv2.circle(self.grid, tuple(right_f[:2].astype(np.int32)), 10, 255)
        cv2.circle(self.grid, tuple(left_b[:2].astype(np.int32)), 3, 125)
        cv2.circle(self.grid, tuple(right_b[:2].astype(np.int32)), 3, 128)
        cv2.imshow("test", self.grid)
        cv2.waitKey(0)

    def get_message(self):
        ogrid = OccupancyGrid()
        ogrid.header = navigator_tools.make_header(frame="enu")
        ogrid.info.resolution = 1 / self.resolution
        ogrid.info.height, ogrid.info.width = self.grid.shape
        ogrid.info.origin = self.origin
        grid = np.copy(self.grid)
        ogrid.data = np.clip(grid.flatten() - 1, -100, 100).astype(np.int8)

        return ogrid
