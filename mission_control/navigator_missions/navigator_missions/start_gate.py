#!/usr/bin/env python
from __future__ import division
from navigator import Navigator
import txros
import tf.transformations as trns
import numpy as np
import mil_tools
from twisted.internet import defer
from nav_msgs.msg import OccupancyGrid
import cv2


class Buoy(object):
    @classmethod
    def from_srv(cls, srv):
        return cls(mil_tools.rosmsg_to_numpy(srv.position), srv.color)

    def __init__(self, position, color):
        self.position = np.array(position)
        self.color = color
        self.odom = np.array([0, 0, 0])

    def __repr__(self):
        return "BUOY at: {} with color {}.".format(self.position, self.color)

    def distance(self, odom):
        return np.linalg.norm(self.position - odom)


class StartGate(Navigator):
    def get_path(self, buoy_l, buoy_r):
        # Vector between the two buoys
        between_vector = buoy_r.position - buoy_l.position

        # Rotate that vector to point through the buoys
        r = trns.euler_matrix(0, 0, np.radians(90))[:3, :3]
        direction_vector = r.dot(between_vector)
        direction_vector /= np.linalg.norm(direction_vector)

        return between_vector, direction_vector

    @txros.util.cancellableInlineCallbacks
    def two_buoys(self, buoys, setup_dist, channel_length=30):
        # We only have the front two buoys in view
        pose = yield self.tx_pose
        # Get the ones closest to us and assume those are the front
        front = buoys

        t = txros.tf.Transform.from_Pose_message(mil_tools.numpy_quat_pair_to_pose(*pose))
        t_mat44 = np.linalg.inv(t.as_matrix())
        f_bl_buoys = [t_mat44.dot(np.append(buoy.position, 1)) for buoy in front]

        # print f_bl_buoys

        # Angles are w.r.t positive x-axis. Positive is CCW around the z-axis.
        angle_buoys = np.array([np.arctan2(buoy[1], buoy[0]) for buoy in f_bl_buoys])
        # print angle_buoys, front
        f_left_buoy, f_right_buoy = front[np.argmax(angle_buoys)], front[np.argmin(angle_buoys)]
        between_v = f_right_buoy.position - f_left_buoy.position
        f_mid_point = f_left_buoy.position + between_v / 2

        setup = self.move.set_position(f_mid_point).look_at(f_left_buoy.position).yaw_right(1.57).backward(setup_dist)
        target = setup.forward(2 * setup_dist + channel_length)

        ogrid = OgridFactory(f_left_buoy.position, f_right_buoy.position,
                             target.position, channel_length=channel_length)
        defer.returnValue([ogrid, setup, target])

    @txros.util.cancellableInlineCallbacks
    def four_buoys(self, buoys, setup_dist):
        pose = yield self.tx_pose
        # Get the ones closest to us and assume those are the front
        distances = np.array([b.distance(pose[0]) for b in buoys])
        back = buoys[np.argsort(distances)[-2:]]
        front = buoys[np.argsort(distances)[:2]]

        # print "front", front
        # print "back", back

        # Made it this far, make sure the red one is on the left and green on the right ================
        t = txros.tf.Transform.from_Pose_message(mil_tools.numpy_quat_pair_to_pose(*pose))
        t_mat44 = np.linalg.inv(t.as_matrix())
        f_bl_buoys = [t_mat44.dot(np.append(buoy.position, 1)) for buoy in front]
        b_bl_buoys = [t_mat44.dot(np.append(buoy.position, 1)) for buoy in back]

        # Angles are w.r.t positive x-axis. Positive is CCW around the z-axis.
        angle_buoys = np.array([np.arctan2(buoy[1], buoy[0]) for buoy in f_bl_buoys])
        # print angle_buoys, front
        f_left_buoy, f_right_buoy = front[np.argmax(angle_buoys)], front[np.argmin(angle_buoys)]

        angle_buoys = np.array([np.arctan2(buoy[1], buoy[0]) for buoy in b_bl_buoys])
        b_left_buoy, b_right_buoy = back[np.argmax(angle_buoys)], back[np.argmin(angle_buoys)]

        # Lets plot a course, yarrr
        f_between_vector, f_direction_vector = self.get_path(f_left_buoy, f_right_buoy)
        f_mid_point = f_left_buoy.position + f_between_vector / 2

        b_between_vector, _ = self.get_path(b_left_buoy, b_right_buoy)
        b_mid_point = b_left_buoy.position + b_between_vector / 2

        setup = self.move.set_position(f_mid_point).look_at(b_mid_point).backward(setup_dist)
        target = setup.set_position(b_mid_point).forward(setup_dist)

        ogrid = OgridFactory(f_left_buoy.position, f_right_buoy.position, target.position,
                             left_b_pos=b_left_buoy.position, right_b_pos=b_right_buoy.position)

        defer.returnValue([ogrid, setup, target])

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        result = self.fetch_result()

        found_buoys = yield self.database_query("start_gate")

        buoys = np.array(map(Buoy.from_srv, found_buoys.objects))

        if len(buoys) == 2:
            ogrid, setup, target = yield self.two_buoys(buoys, 10)
        elif len(buoys) == 4:
            ogrid, setup, target = yield self.four_buoys(buoys, 10)
        else:
            raise Exception

        print "Waiting 3 seconds to move..."
        yield self.nh.sleep(3)
        yield setup.go(move_type='skid')

        pose = yield self.tx_pose
        ogrid.generate_grid(pose)
        msg = ogrid.get_message()

        print "publishing"
        self.latching_publisher("/mission_ogrid", OccupancyGrid, msg)

        print "START_GATE: Moving in 5 seconds!"
        yield target.go(initial_plan_time=5)
        defer.returnValue(result)


class OgridFactory():
    def __init__(self, left_f_pos, right_f_pos, target, left_b_pos=None, right_b_pos=None, channel_length=30):
        # Front buoys
        self.left_f = left_f_pos
        self.right_f = right_f_pos
        # Back buoys
        self.left_b = left_b_pos
        self.right_b = right_b_pos
        # Other info
        self.target = target

        # Some parameters
        self.resolution = .3  # meters/cell for ogrid
        self.channel_length = channel_length  # Not sure what this acutally is for the course (m)
        self.edge_buffer = 10

    def generate_grid(self, pose):
        self.boat_pos = pose[0]

        # Sets x,y upper and lower bounds and the left and right wall bounds
        self.get_size_and_build_walls()
        self.make_ogrid_transform()
        self.draw_lines(self.walls)

    def make_ogrid_transform(self):
        self.origin = mil_tools.numpy_quat_pair_to_pose([self.x_lower, self.y_lower, 0],
                                                        [0, 0, 0, 1])
        dx = self.x_upper - self.x_lower
        dy = self.y_upper - self.y_lower
        _max = max(dx, dy)
        dx = dy = _max
        # The grid needs to have it's axes swaped since its row major
        self.grid = np.zeros((dy / self.resolution, dx / self.resolution))

        # Transforms points from ENU to ogrid frame coordinates
        self.t = np.array([[1 / self.resolution, 0, -self.x_lower / self.resolution],
                           [0, 1 / self.resolution, -self.y_lower / self.resolution],
                           [0, 0, 1]])
        self.transform = lambda point: self.t.dot(np.append(point[:2], 1))[:2]

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
        rot_buffer = b_rot_mat.dot([20, 0, 0])
        endpoint_left_f = self.left_f + rot_buffer
        endpoint_right_f = self.right_f - rot_buffer

        # Now lets build some wall points ======================================

        if self.left_b is None:
            # For rotations of the `target_vector` and the enu x-axis
            t_theta = np.arctan2(target_vector[1], target_vector[0])
            t_rot_mat = trns.euler_matrix(0, 0, t_theta)[:3, :3]

            rot_channel = t_rot_mat.dot([self.channel_length, 0, 0])
            self.left_b = self.left_f + rot_channel
            self.right_b = self.right_f + rot_channel

        # Now draw contours from the boat to the start gate ====================
        # Get vector from boat to mid_point
        mid_point_vector = self.boat_pos - self.mid_point

        b_theta = np.arctan2(mid_point_vector[1], mid_point_vector[0])
        b_rot_mat = trns.euler_matrix(0, 0, b_theta)[:3, :3]

        rot_buffer = b_rot_mat.dot([np.linalg.norm(mid_point_vector) * 1.5, 0, 0])
        left_cone_point = self.left_f + rot_buffer
        right_cone_point = self.right_f + rot_buffer

        # Define bounds for the grid
        self.x_lower = min(left_cone_point[0], right_cone_point[0],
                           endpoint_left_f[0], endpoint_right_f[0], self.target[0]) - self.edge_buffer
        self.x_upper = max(left_cone_point[0], right_cone_point[0],
                           endpoint_left_f[0], endpoint_right_f[0], self.target[0]) + self.edge_buffer
        self.y_lower = min(left_cone_point[1], right_cone_point[1],
                           endpoint_left_f[1], endpoint_right_f[1], self.target[1]) - self.edge_buffer
        self.y_upper = max(left_cone_point[1], right_cone_point[1],
                           endpoint_left_f[1], endpoint_right_f[1], self.target[1]) + self.edge_buffer

        self.walls = [self.left_b, self.left_f, left_cone_point, right_cone_point, self.right_f, self.right_b]

    def draw_lines(self, points):
        last_wall = None
        for wall in points:
            if last_wall is None:
                last_wall = tuple(self.transform(wall).astype(np.int32))
                continue

            this_wall = tuple(self.transform(wall).astype(np.int32))
            cv2.line(self.grid, this_wall, last_wall, 128, 3)
            last_wall = this_wall

    def draw_walls(self):
        left_wall_points = np.array([self.transform(point) for point in self.left_wall_points])
        right_wall_points = np.array([self.transform(point) for point in self.right_wall_points])

        rect = cv2.minAreaRect(left_wall_points[:, :2].astype(np.float32))
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(self.grid, [box], 0, 128, -1)

        rect = cv2.minAreaRect(right_wall_points[:, :2].astype(np.float32))
        box = cv2.boxPoints(rect)
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
        ogrid.header = mil_tools.make_header(frame="enu")
        ogrid.info.resolution = self.resolution
        ogrid.info.height, ogrid.info.width = self.grid.shape
        ogrid.info.origin = self.origin
        grid = np.copy(self.grid)
        ogrid.data = np.clip(grid.flatten() - 1, -100, 100).astype(np.int8)

        return ogrid
