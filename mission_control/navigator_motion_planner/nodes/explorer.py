#!/usr/bin/env python
from __future__ import division

import tf
import rospy
import actionlib
import navigator_tools
from navigator_singleton.pose_editor import PoseEditor2
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from uf_common.msg import MoveToAction
from std_srvs.srv import SetBool

import cv2
import math
import numpy as np

class DatabaseInterface(object):
    def get_closest_unknown(self):
        return np.append(np.random.random(2) * 60, 0)

class OGridManager(object):
    def __init__(self):
        # Ogrid used to keep track of where we have explored
        self.map_metadata = MapMetaData()
        self.map_metadata.resolution = .5
        self.map_metadata.width = 500
        self.map_metadata.height = 500
        origin = navigator_tools.numpy_quat_pair_to_pose([-125, -125, 0], [0, 0, 0, 1])
        self.map_metadata.origin = origin

        self.reset_grid()
        self.ogrid_pub = rospy.Publisher("/explorer/ogrid", OccupancyGrid, queue_size=1)

        self.draw_shape = self._lidar_view

        rospy.Timer(self.pub_ogrid, rospy.Duration(2))

    def _circle(self, pose):
        ''' ENU center and meter radius '''
        radius = 50  # m
        position = pose[0]

        cpm = 1 / self.map_metadata.resolution  # cells per meter
        origin = navigator_tools.pose_to_numpy(self.map_metadata.origin)[0]

        # Convert from meters to grid measurements
        center_grid = (cpm * (position - origin)).astype(np.int64)
        radius_grid = int(radius * cpm)

        cv2.circle(self.grid, tuple(center_grid[:2]), radius, 1, -1)

    def _lidar_view(self, pose):
        radius = 30  # m
        position, quat = pose
        yaw = tf.transformations.euler_from_quaternion([quat[0], quat[1], quat[2], quat[3]])[2]

        # Need to get the position on the ogrid to put the shape
        cpm = 1 / self.map_metadata.resolution  # cells per meter
        origin = navigator_tools.pose_to_numpy(self.map_metadata.origin)[0]

        # Convert from meters to grid measurements
        center_grid = (cpm * (position - origin - radius)).astype(np.int64)[:2]
        radius_grid = int(radius * cpm)

        # Draw the shape on it's own canvas then combine
        shape = np.zeros((2 * radius_grid, 2 * radius_grid))  # Blank place to put the cone
        cv2.circle(shape, (radius_grid, radius_grid), radius_grid, 1, -1)

        # Get rid of that triangle of unseen space
        pts = np.array([[radius_grid, radius_grid], shape.shape, [0, shape.shape[1]]])
        pts = pts.reshape((-1,1,2)).astype(np.int32)
        cv2.fillPoly(shape, [pts], 0)

        # Then rotate so that we are oriented the same as the boat
        M = cv2.getRotationMatrix2D((radius_grid, radius_grid), np.degrees(yaw) - 90, 1.0)
        shape = cv2.warpAffine(shape, M, shape.shape[:2][::-1])

        # Add the shape to the grid (you have to do some funky flipping here)
        self.grid[center_grid[1]:center_grid[1] + int(2 * radius_grid),
                  center_grid[0]:center_grid[0] + int(2 * radius_grid)] += np.flipud(shape)

    def reset_grid(self):
        print "Resetting Grid."
        self.grid = np.zeros((self.map_metadata.height, self.map_metadata.width)) - 1

    def pub_ogrid(self, *args):
        ogrid = OccupancyGrid()
        ogrid.header = navigator_tools.make_header(frame="enu")
        ogrid.info = self.map_metadata

        # Over saturate and clip to get range of [-1, 1]
        self.grid = np.clip(self.grid * 1000, -1, 1)
        ogrid.data = self.grid.flatten().astype(np.int8).tolist()

        self.ogrid_pub.publish(ogrid)

class Explorer(object):
    def __init__(self):
        self.navigator = Navig

        #self.db = DatabaseInterface()

        self.ogrid = OGridManager()
        self.ogrid.draw_shape = self.ogrid._lidar_view

        self.has_control = False
        self.move_result = None

        rospy.Subscriber("/odom", Odometry, self.got_odom)
        rospy.Service("/explorer/explore", SetBool, self.search_request)

        #while self._nav_dummy.pose is None and not rospy.is_shutdown():
        #    rospy.sleep(.1)

        print "Taking control"
        #self.take_control()

    def take_control(self):
        closest = self.db.get_closest_unknown()
        if closest is not None:
            ps = PoseEditor2(self._nav_dummy, self._nav_dummy.pose)
            self.move_result = ps.set_position(closest).go()
            print "running"

    def got_odom(self, msg):
        pose, twist = navigator_tools.odometry_to_numpy(msg)[:2]
        #self.draw_shape(pose)
        self._nav_dummy.pose = pose

    def search_request(self, srv):
        self.has_control = srv.data
        return SetBoolResponse()


rospy.init_node("navigator_explorer")
e = Explorer()
rospy.spin()