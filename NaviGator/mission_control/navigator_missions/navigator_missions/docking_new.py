#!/usr/bin/env python3
import asyncio
import os

import cv2
import numpy as np
import txros
from cv_bridge import CvBridge
from dynamic_reconfigure.msg import DoubleParameter
from image_geometry import PinholeCameraModel
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from mil_tools import numpy_to_pointcloud2 as np2pc2
from mil_tools import pose_to_numpy, rosmsg_to_numpy
from mil_vision_tools.cv_tools import contour_mask, rect_from_roi, roi_enclosing_points
from navigator_vision import VrxStcColorClassifier
from rospkg import RosPack
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Empty
from std_srvs.srv import SetBoolRequest
from tf import transformations
from tf.transformations import quaternion_matrix

from .navigator import Navigator

PANNEL_MAX = 0
PANNEL_MIN = 2

CAMERA_LINK_OPTICAL = "wamv/front_left_camera_link_optical"

COLOR_SEQUENCE_SERVICE = "/vrx/scan_dock/color_sequence"

TIMEOUT_SECONDS = 30


class Dock(Navigator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    async def run(self, args):

        self.bridge = CvBridge()

        # debug image for occupancy grid
        self.image_debug_pub = self.nh.advertise("/dock_mask_debug", Image)

        print("entered docking task", self.color, self.shape)

        pcodar_cluster_tol = DoubleParameter()
        pcodar_cluster_tol.name = "cluster_tolerance_m"
        pcodar_cluster_tol.value = 10
        await self.pcodar_set_params(doubles=[pcodar_cluster_tol])
        await self.nh.sleep(5)

        # find dock approach it
        pos = await self.find_dock_poi()

        print("going towards dock")
        await self.move.look_at(pos).set_position(pos).backward(20).go()

        # Decrease cluster tolerance as we approach dock since lidar points are more dense
        # This helps scenario where stc buoy is really close to dock
        pcodar_cluster_tol = DoubleParameter()
        pcodar_cluster_tol.name = "cluster_tolerance_m"
        pcodar_cluster_tol.value = 4
        await self.pcodar_set_params(doubles=[pcodar_cluster_tol])
        await self.nh.sleep(5)

        await self.find_dock()

        # get a vector to the longer side of the dock
        dock, pos = await self.get_sorted_objects(name="dock", n=1)
        dock = dock[0]
        position, quat = pose_to_numpy(dock.pose)
        rotation = quaternion_matrix(quat)
        bbox = rosmsg_to_numpy(dock.scale)
        bbox[2] = 0
        max_dim = np.argmax(bbox[:2])
        bbox[max_dim] = 0
        bbox_enu = np.dot(rotation[:3, :3], bbox)
        # this black magic uses the property that a rotation matrix is just a
        # rotated cartesian frame and only gets the vector that points towards
        # the longest side since the vector pointing that way will be at the
        # same index as the scale for the smaller side. This is genius!
        # - Andrew Knee

        # move to first attempt
        print("moving in front of dock")
        goal_pos = None
        curr_pose = await self.tx_pose()
        side_a_bool = False
        side_b_bool = False
        side_a = bbox_enu + position
        side_b = -bbox_enu + position

        correct_side = await self.calculate_ogrid_mass_center(side_a, side_b)

        await self.move.set_position(correct_side).look_at(position).go()

        # define how far left and right we want to do
        rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

        side_vect = 0.80 * bbox_enu
        if side_a_bool:
            print("creating side a left and right position")
            self.left_position = np.dot(rot, -side_vect) + side_a
            self.right_position = np.dot(rot, side_vect) + side_a
            self.dock_point_left = np.dot(rot, -side_vect) + position
            self.dock_point_right = np.dot(rot, side_vect) + position
        else:
            print("creating side b left and right position")
            self.left_position = np.dot(rot, side_vect) + side_b
            self.right_position = np.dot(rot, -side_vect) + side_b
            self.dock_point_left = np.dot(rot, side_vect) + position
            self.dock_point_right = np.dot(rot, -side_vect) + position

    async def calculate_ogrid_mass_center(
        self, side_a: [float], side_b: [float]
    ) -> [float]:
        print("Finding ogrid center of mass")

    async def find_dock_poi(self, hint: Pose | None = None):
        print("Finding poi")

    # This function is used to find the position of the dock after the boat is near a POI
    async def find_dock(self):

        msgs = None
        while msgs is None:
            try:
                # msgs, poses = await self.get_sorted_objects(name="UNKNOWN", n=-1)
                msg = await self.get_largest_object()
            except Exception as e:
                await self.move.forward(10).go()
        await self.pcodar_label(msg.id, "dock")
        # if no pcodar objects, throw error, exit mission
        # pose = poses[0]
        pose = msg.pose.position
        pose = [pose.x, pose.y, pose.z]

        return pose
