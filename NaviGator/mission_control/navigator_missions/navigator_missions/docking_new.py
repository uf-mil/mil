#!/usr/bin/env python3
import asyncio
import copy
import os
import random
from typing import Optional

import cv2
import numpy as np
import rospy
import txros
from cv_bridge import CvBridge
from dynamic_reconfigure.msg import DoubleParameter
from geometry_msgs.msg import Pose
from image_geometry import PinholeCameraModel
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from mil_tools import numpy_to_pointcloud2 as np2pc2
from mil_tools import pose_to_numpy, rosmsg_to_numpy, thread_lock
from mil_vision_tools.cv_tools import contour_mask, rect_from_roi, roi_enclosing_points
from nav_msgs.msg import OccupancyGrid
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
        self.ogrid = None
        self.ogrid_origin = None
        self.ogrid_cpm = None
        self.ogrid_sub = self.nh.subscribe(
            "/ogrid", OccupancyGrid, callback=self.ogrid_cb
        )
        self.image_sub = self.nh.subscribe(
            "/wamv/sensors/camera/front_left_cam/image_raw", Image
        )
        self.cam_frame = None
        self.image_info_sub = self.nh.subscribe(
            "/wamv/sensors/camera/front_left_cam/camera_info", CameraInfo
        )
        self.model = PinholeCameraModel()
        self.center = None
        self.intup = lambda arr: tuple(np.array(arr, dtype=np.int64))
        self.last_image = None
        self.setBool = False

    @classmethod
    async def init(cls):
        cls.image_debug_pub = cls.nh.advertise("/dock_mask_debug", Image)
        await cls.image_debug_pub.setup()
        cls.contour_pub = cls.nh.advertise("/contour_pub", Image)
        await cls.contour_pub.setup()

    @classmethod
    async def shutdown(cls):
        await cls.image_debug_pub.shutdown()
        await cls.contour_pub.shutdown()
        await cls.ogrid_sub.shutdown()
        await cls.image_sub.shutdown()
        await cls.image_info_sub.shutdown()

    async def run(self, args):
        await self.ogrid_sub.setup()
        await self.image_sub.setup()
        await self.image_info_sub.setup()

        self.bridge = CvBridge()
        msg = await self.image_info_sub.get_next_message()
        self.model.fromCameraInfo(msg)

        self.cam_frame = (await self.image_sub.get_next_message()).header.frame_id

        # debug image for occupancy grid

        print("starting docking task")

        pcodar_cluster_tol = DoubleParameter()
        pcodar_cluster_tol.name = "cluster_tolerance_m"
        pcodar_cluster_tol.value = 10
        await self.pcodar_set_params(doubles=[pcodar_cluster_tol])
        await self.nh.sleep(5)

        pos = await self.poi.get("dock")
        await self.move.look_at(pos).set_position(pos).go()
        # find dock approach it
        # pos = await self.find_dock_poi()

        # print("going towards dock")
        # await self.move.look_at(pos).set_position(pos).backward(20).go()

        # Decrease cluster tolerance as we approach dock since lidar points are more dense
        # This helps scenario where stc buoy is really close to dock
        pcodar_cluster_tol = DoubleParameter()
        pcodar_cluster_tol.name = "cluster_tolerance_m"
        pcodar_cluster_tol.value = 4
        await self.pcodar_set_params(doubles=[pcodar_cluster_tol])
        await self.nh.sleep(5)

        await self.find_dock()

        # get a vector to the longer side of the dock
        dock, pos = None, None
        while dock is None or pos is None:
            try:
                dock, pos = await self.get_sorted_objects(name="dock", n=1)
            except:
                await self.find_dock()
                dock, pos = await self.get_sorted_objects(name="dock", n=1)
        # dock is PerceptionObject
        dock = dock[0]
        position, quat = pose_to_numpy(dock.pose)
        rotation = quaternion_matrix(quat)
        bbox = rosmsg_to_numpy(dock.scale)
        bbox2 = copy.deepcopy(bbox)
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
        # curr_pose = await self.tx_pose()
        side_a_bool = False
        side_a = bbox_enu + position
        side_b = -bbox_enu + position

        side_a_bool = self.calculate_correct_side(
            copy.deepcopy(side_a),
            copy.deepcopy(side_b),
            position,
            rotation[:3, :3],
            bbox2,
        )

        # TODO, add check to recaluclate position once we get on either side of the dock
        await self.move.set_position(side_a if side_a_bool else side_b).look_at(
            position
        ).go()

        # recalculate dock position once on correct side
        await self.find_dock()

        dock, pos = await self.get_sorted_objects(name="dock", n=1)
        # dock is PerceptionObject
        dock = dock[0]
        position, quat = pose_to_numpy(dock.pose)
        rotation = quaternion_matrix(quat)
        points = rosmsg_to_numpy(dock.points)
        centers = self.get_cluster_centers(points)
        enu_to_boat = await self.tf_listener.get_transform("wamv/base_link", "enu")
        left = enu_to_boat.transform_point(centers[0])
        left[0] = 0
        forward = enu_to_boat.transform_point(centers[0])
        forward[0] = forward[0] - 5
        boat_to_enu = await self.tf_listener.get_transform("enu", "wamv/base_link")
        centers[0] = boat_to_enu.transform_point(left)
        nextPt = boat_to_enu.transform_point(forward)
        await self.move.set_position(centers[0]).go(blind=True, move_type="skid")
        await self.move.set_position(nextPt).go(blind=True, move_type="skid")

        await self.crop_image()

        # define how far left and right we want to do
        rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

        side_vect = 0.80 * bbox_enu
        # if side_a_bool:
        #     print("creating side a left and right position")
        #     self.left_position = np.dot(rot, -side_vect) + side_a
        #     self.right_position = np.dot(rot, side_vect) + side_a
        #     self.dock_point_left = np.dot(rot, -side_vect) + position
        #     self.dock_point_right = np.dot(rot, side_vect) + position
        # else:
        #     print("creating side b left and right position")
        #     self.left_position = np.dot(rot, side_vect) + side_b
        #     self.right_position = np.dot(rot, -side_vect) + side_b
        #     self.dock_point_left = np.dot(rot, side_vect) + position
        #     self.dock_point_right = np.dot(rot, -side_vect) + position

        # await self.move.set_position(self.left_position).look_at(
        #     self.dock_point_left
        # ).go(blind=True, move_type="skid")

    def get_cluster_centers(self, data):
        mean = np.mean(data, axis=0)[2]
        data = data[data[:, 2] > mean]
        centroids = []

        # Sample initial centroids
        random_indices = random.sample(range(data.shape[0]), 3)
        for i in random_indices:
            centroids.append(data[i])

        # Create a list to store which centroid is assigned to each dataset
        assigned_centroids = [0] * len(data)

        def compute_l2_distance(x, centroid):
            # Initialise the distance to 0
            dist = 0

            # Loop over the dimensions. Take squared difference and add to dist

            for i in range(len(x)):
                dist += (centroid[i] - x[i]) ** 2

            return dist

        def get_closest_centroid(x, centroids):
            # Initialise the list to keep distances from each centroid
            centroid_distances = []

            # Loop over each centroid and compute the distance from data point.
            for centroid in centroids:
                dist = compute_l2_distance(x, centroid)
                centroid_distances.append(dist)

            # Get the index of the centroid with the smallest distance to the data point
            closest_centroid_index = min(
                range(len(centroid_distances)), key=lambda x: centroid_distances[x]
            )

            return closest_centroid_index

        def compute_sse(data, centroids, assigned_centroids):
            # Initialise SSE
            sse = 0

            # Compute the squared distance for each data point and add.
            for i, x in enumerate(data):
                # Get the associated centroid for data point
                centroid = centroids[assigned_centroids[i]]

                # Compute the Distance to the centroid
                dist = compute_l2_distance(x, centroid)

                # Add to the total distance
                sse += dist

            sse /= len(data)
            return sse

        # Number of dimensions in centroid
        num_centroid_dims = data.shape[1]

        # List to store SSE for each iteration
        sse_list = []

        # Loop over iterations
        for n in range(10):

            # Loop over each data point
            for i in range(len(data)):
                x = data[i]

                # Get the closest centroid
                closest_centroid = get_closest_centroid(x, centroids)

                # Assign the centroid to the data point.
                assigned_centroids[i] = closest_centroid

            # Loop over centroids and compute the new ones.
            for c in range(len(centroids)):
                # Get all the data points belonging to a particular cluster
                cluster_data = [
                    data[i] for i in range(len(data)) if assigned_centroids[i] == c
                ]

                # Initialise the list to hold the new centroid
                new_centroid = [0] * len(centroids[0])

                # Compute the average of cluster members to compute new centroid
                # Loop over dimensions of data
                for dim in range(num_centroid_dims):
                    dim_sum = [x[dim] for x in cluster_data]
                    dim_sum = sum(dim_sum) / len(dim_sum)
                    new_centroid[dim] = dim_sum

                # assign the new centroid
                centroids[c] = new_centroid

            # Compute the SSE for the iteration
            sse = compute_sse(data, centroids, assigned_centroids)
            sse_list.append(sse)

        cluster_members = []

        for c in range(len(centroids)):
            cluster_member = [
                data[i] for i in range(len(data)) if assigned_centroids[i] == c
            ]
            cluster_members.append(np.array(cluster_member))

        means = []

        for c in range(len(cluster_members)):
            means.append(np.mean(cluster_members[c], axis=0))

        print(means)
        return np.asarray(means)

    async def crop_image(self):
        image = await self.image_sub.get_next_message()
        enu_to_cam = await self.tf_listener.get_transform(self.cam_frame, "enu")
        transform = enu_to_cam.transform_point(self.center)
        rect = self.model.project3dToPixel(transform)
        image = self.bridge.imgmsg_to_cv2(image)
        height = image.shape[0]
        image = image[: int(rect[1]), :]
        msg = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
        self.contour_pub.publish(msg)

    # returns True if side a is closest, False is side b is closest
    def calculate_correct_side(
        self,
        side_a: [float],
        side_b: [float],
        position: [float],
        rotation: [[float]],
        scale: [float],
    ) -> bool:
        print("Finding ogrid center of mass")
        self.ogrid_origin = np.asarray(self.ogrid_origin)
        point1 = self.intup(
            self.ogrid_cpm
            * (
                self.get_point(
                    [-scale[0] / 2, scale[1] / 2, scale[2]], rotation, position
                )
                - self.ogrid_origin
            )[:2]
        )
        point2 = self.intup(
            self.ogrid_cpm
            * (
                self.get_point(
                    [scale[0] / 2, scale[1] / 2, scale[2]], rotation, position
                )
                - self.ogrid_origin
            )[:2]
        )
        point3 = self.intup(
            self.ogrid_cpm
            * (
                self.get_point(
                    [scale[0] / 2, -scale[1] / 2, scale[2]], rotation, position
                )
                - self.ogrid_origin
            )[:2]
        )
        point4 = self.intup(
            self.ogrid_cpm
            * (
                self.get_point(
                    [-scale[0] / 2, -scale[1] / 2, scale[2]], rotation, position
                )
                - self.ogrid_origin
            )[:2]
        )
        contours = np.array([point1, point2, point3, point4])
        center = self.calculate_center_of_mass(contours)
        center_coordinate = self.ogrid_to_position(center)
        self.center = [center_coordinate[0], center_coordinate[1], position[2] - 1]
        side_a = np.asarray(side_a)
        side_b = np.asarray(side_b)
        center = np.asarray(center)
        side_a = self.intup(self.ogrid_cpm * (side_a - self.ogrid_origin))[:2]
        side_b = self.intup(self.ogrid_cpm * (side_b - self.ogrid_origin))[:2]
        dist_a = np.linalg.norm(side_a - center)
        dist_b = np.linalg.norm(side_b - center)
        # cv2.fillPoly(self.last_image, pts=[contours], color=(255, 0, 0))
        cv2.circle(self.last_image, center, radius=3, color=(255, 0, 0))
        cv2.circle(self.last_image, side_a, radius=3, color=(0, 255, 0))
        cv2.circle(self.last_image, side_b, radius=3, color=(0, 0, 255))
        print("Center of mass found")
        self.setBool = True
        return dist_a > dist_b

    def ogrid_to_position(self, ogrid):
        ogrid = np.array(ogrid, dtype=np.float64)
        ogrid = ogrid / self.ogrid_cpm
        ogrid = ogrid + self.ogrid_origin[:2]
        return tuple(ogrid)

    def calculate_center_of_mass(self, points):
        bounding_rect = cv2.boundingRect(points)
        x, y, w, h = bounding_rect
        mask = np.zeros(self.last_image.shape, np.uint8)
        cv2.drawContours(mask, [points], -1, (255, 255, 255), -1, cv2.LINE_AA)
        masked = cv2.bitwise_and(self.last_image, mask)
        count = 0
        x_sum = 0
        y_sum = 0
        for i in range(x, x + w):
            for j in range(y, y + h):
                if (masked[j, i] == [255, 255, 255]).all():
                    x_sum = x_sum + i
                    y_sum = y_sum + j
                    count = count + 1
        x_sum = int(x_sum / count)
        y_sum = int(y_sum / count)
        return (x_sum, y_sum)

    def get_point(self, corner, rotation_matrix, center):
        return np.matmul(rotation_matrix, np.asarray(corner)) + np.asarray(center)

    async def find_dock_poi(self, hint: Optional[Pose] = None):
        print("Finding poi")

    # This function is used to find the position of the dock after the boat is near a POI
    async def find_dock(self):
        print("Searching for dock")
        msg = None
        while msg is None:
            try:
                # msgs, poses = await self.get_sorted_objects(name="UNKNOWN", n=-1)
                msg = await self.get_largest_object()
            except Exception as e:
                await self.move.forward(10).go()
        await self.pcodar_label(msg.id, "dock")
        # if no pcodar objects, throw error, exit mission
        # pose = poses[0]
        pose = pose_to_numpy(msg.pose)

        return pose

    def ogrid_cb(self, msg):
        if not self.setBool:
            self.ogrid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            self.ogrid_origin = rosmsg_to_numpy(msg.info.origin.position)
            self.ogrid_cpm = 1 / msg.info.resolution

            image = 255 * np.greater(self.ogrid, 90).astype(np.uint8)
            grayImage = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

            self.last_image = grayImage
        self.image_debug_pub.publish(
            self.bridge.cv2_to_imgmsg(self.last_image, encoding="rgb8")
        )
        return
