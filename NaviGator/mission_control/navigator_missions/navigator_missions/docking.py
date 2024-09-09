#!/usr/bin/env python3
import copy
import random
from typing import Optional

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from dynamic_reconfigure.msg import DoubleParameter
from geometry_msgs.msg import Pose
from image_geometry import PinholeCameraModel
from mil_tools import pose_to_numpy, rosmsg_to_numpy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool, SetBoolRequest
from tf.transformations import quaternion_matrix

from .navigator import NaviGatorMission

PANNEL_MAX = 0
PANNEL_MIN = 2

CAMERA_LINK_OPTICAL = "wamv/front_left_camera_link_optical"

COLOR_SEQUENCE_SERVICE = "/vrx/scan_dock/color_sequence"

TIMEOUT_SECONDS = 30


class Docking(NaviGatorMission):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.ogrid = None
        self.ogrid_origin = None
        self.ogrid_cpm = None
        self.ogrid_sub = self.nh.subscribe(
            "/ogrid",
            OccupancyGrid,
            callback=self.ogrid_cb,
        )

        # Service to save and restore the settings of PCODAR
        self.pcodar_save = self.nh.get_service_client("/pcodar/save", SetBool)
        # self.image_sub = self.nh.subscribe(
        #     "/wamv/sensors/cameras/front_left_camera/image_raw", Image
        # )
        self.image_sub = self.nh.subscribe(
            "/wamv/sensors/cameras/front_left_camera/image_raw",
            Image,
        )
        self.cam_frame = None
        # self.image_info_sub = self.nh.subscribe(
        #     "/wamv/sensors/cameras/front_left_camera/camera_info", CameraInfo
        # )
        self.image_info_sub = self.nh.subscribe(
            "/wamv/sensors/cameras/front_left_camera/camera_info",
            CameraInfo,
        )
        self.model = PinholeCameraModel()
        self.center = None
        self.intup = lambda arr: tuple(np.array(arr, dtype=np.int64))
        self.last_image = None

    @classmethod
    async def init(cls):
        cls.contour_pub = cls.nh.advertise("/contour_pub", Image)
        await cls.contour_pub.setup()

    @classmethod
    async def shutdown(cls):
        await cls.contour_pub.shutdown()
        await cls.ogrid_sub.shutdown()
        await cls.image_sub.shutdown()
        await cls.image_info_sub.shutdown()

    async def run(self, args):
        await self.ogrid_sub.setup()
        await self.image_sub.setup()
        await self.image_info_sub.setup()
        await self.pcodar_save.wait_for_service()

        self.bridge = CvBridge()
        msg = await self.image_info_sub.get_next_message()
        self.model.fromCameraInfo(msg)
        rospy.logerr("HERE")

        self.cam_frame = (await self.image_sub.get_next_message()).header.frame_id
        rospy.logerr("HERE2")

        # Save PCODAR settings
        await self.pcodar_save(SetBoolRequest(True))

        # Change cluster tolerance to make it easier to find dock
        pcodar_cluster_tol = DoubleParameter()
        pcodar_cluster_tol.name = "cluster_tolerance_m"
        pcodar_cluster_tol.value = 10
        await self.pcodar_set_params(doubles=[pcodar_cluster_tol])
        rospy.logerr("HERE3")
        await self.nh.sleep(5)

        # Get the POI (point of interest) from the config file and move to it
        # This is a predetermined position of the general location for the dock
        pos = await self.poi.get("dock")
        rospy.logerr("HERE4")
        await self.move.look_at(pos).set_position(pos).go()

        # Decrease cluster tolerance as we approach dock since lidar points are more dense
        # This helps scenario where stc buoy is really close to dock
        pcodar_cluster_tol = DoubleParameter()
        pcodar_cluster_tol.name = "cluster_tolerance_m"
        pcodar_cluster_tol.value = 4
        await self.pcodar_set_params(doubles=[pcodar_cluster_tol])
        await self.nh.sleep(5)

        # move to the open side of the dock
        await self.move_to_correct_side()

        # retry calculation to make sure we really found the open side
        await self.move_to_correct_side()

        # get the dock object from the database
        dock, pos = await self.get_sorted_objects(name="dock", n=1)

        # dock is PerceptionObject
        position, rotation, dock = self.get_dock_data(dock)

        # get LIDAR points
        points = rosmsg_to_numpy(dock.points)

        # get transform from enu to boat space
        enu_to_boat = await self.tf_listener.get_transform("wamv/base_link", "enu")
        corrected = np.empty(points.shape)

        # convert LIDAR points from enu to boat space
        for i in range(points.shape[0]):
            corrected[i] = enu_to_boat.transform_point(points[i])

        # get the centers and clusters that represent the backside of the dock
        centers, clusters = self.get_cluster_centers(corrected)

        centers = centers[centers[:, 1].argsort()][::-1]

        # crop the images to get bbox and find color
        images = await self.crop_images(clusters)
        self.find_color(images, 1)

        # temporary code that just moves boat to center of leftmost cluster
        left = copy.deepcopy(centers[0])
        rospy.logerr(centers[0])
        rospy.logerr(centers[1])
        rospy.logerr(centers[2])

        # calculate center of cluster and move towards it but at an offset distance
        left[0] = 0
        forward = copy.deepcopy(centers[0])
        forward[0] = forward[0] - 5
        boat_to_enu = await self.tf_listener.get_transform("enu", "wamv/base_link")
        centers[0] = boat_to_enu.transform_point(left)
        nextPt = boat_to_enu.transform_point(forward)
        await self.move.set_position(centers[0]).go(blind=True, move_type="skid")
        await self.move.set_position(nextPt).go(blind=True, move_type="skid")
        await self.pcodar_save(SetBoolRequest(False))

        await self.contour_pub.shutdown()
        await self.ogrid_sub.shutdown()
        await self.image_sub.shutdown()
        await self.image_info_sub.shutdown()
        await self.pcodar_save.shutdown()

    def get_dock_data(self, dock):
        dock = dock[0]
        position, quat = pose_to_numpy(dock.pose)
        rotation = quaternion_matrix(quat)
        return position, rotation, dock

    async def move_to_correct_side(self):
        await self.find_dock()

        # get a vector to the longer side of the dock
        dock, pos = None, None
        while dock is None or pos is None:
            try:
                # looks the the LIDAR cluster database and finds the object with name "dock"
                dock, pos = await self.get_sorted_objects(name="dock", n=1)
            except Exception as _:
                # retries if an exception occurs
                await self.find_dock()
                dock, pos = await self.get_sorted_objects(name="dock", n=1)

        # find the open side of the dock
        side, position = self.get_correct_side(dock)

        # move the boat to the middle of the correct side offset by 2 meters back and face the center of the dock
        await self.move.set_position(side).look_at(position).backward(2).go()

    def get_correct_side(self, dock):
        # dock is PerceptionObject
        position, rotation, dock = self.get_dock_data(dock)

        # get bounding box of the dock
        bbox_large = rosmsg_to_numpy(dock.scale)
        bbox_copy = copy.deepcopy(bbox_large)

        bbox_large[2] = 0
        bbox_small = copy.deepcopy(bbox_large)

        max_dim = np.argmax(bbox_large[:2])
        bbox_large[max_dim] = 0
        bbox_enu = np.dot(rotation[:3, :3], bbox_large)

        min_dim = np.argmin(bbox_small[:2])
        bbox_small[min_dim] = 0
        bbox_enu_small = np.dot(rotation[:3, :3], bbox_small)
        # this black magic uses the property that a rotation matrix is just a
        # rotated cartesian frame and only gets the vector that points towards
        # the longest side since the vector pointing that way will be at the
        # same index as the scale for the smaller side. This is genius!
        # - Andrew Knee

        # move to first attempt
        print("moving in front of dock")
        # curr_pose = await self.tx_pose()
        side_a = bbox_enu + position
        side_b = -bbox_enu + position
        side_c = bbox_enu_small + position
        side_d = -bbox_enu_small + position

        return (
            self.calculate_correct_side(
                copy.deepcopy(side_a),
                copy.deepcopy(side_b),
                copy.deepcopy(side_c),
                copy.deepcopy(side_d),
                position,
                rotation[:3, :3],
                bbox_copy,
            ),
            position,
        )

    # separates the clusters using k means clustering
    def get_cluster_centers(self, data):

        # cut off all points below the mean z value
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
                range(len(centroid_distances)),
                key=lambda x: centroid_distances[x],
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
        return np.asarray(means), cluster_members

    def crop_image(self, pts, transform, img):
        pts = [self.model.project3dToPixel(transform.transform_point(a)) for a in pts]
        pts = np.array([[int(a[0]), int(a[1])] for a in pts], dtype=np.int32)
        pts = np.int32([pts])
        rospy.logerr(pts)
        mask = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
        cv2.fillPoly(mask, pts, (255))
        res = cv2.bitwise_and(img, img, mask=mask)
        rect = cv2.boundingRect(pts)
        cropped = res[rect[1] : rect[1] + rect[3], rect[0] : rect[0] + rect[2]]
        return cropped

    def get_cluster_corners(self, cluster):
        avg_x = np.mean(cluster[:, 0])
        min_y = np.amin(cluster[:, 1])
        max_y = np.amax(cluster[:, 1])
        min_z = np.amin(cluster[:, 2])
        max_z = np.amax(cluster[:, 2])
        return np.asarray(
            [
                [avg_x, min_y, min_z],
                [avg_x, min_y, max_z],
                [avg_x, max_y, max_z],
                [avg_x, max_y, min_z],
            ],
        )

    async def crop_images(self, clusters):
        image = await self.image_sub.get_next_message()
        image = self.bridge.imgmsg_to_cv2(image)
        boat_to_cam = await self.tf_listener.get_transform(
            self.cam_frame,
            "wamv/base_link",
        )

        left = self.crop_image(
            self.get_cluster_corners(clusters[0]),
            boat_to_cam,
            image,
        )
        middle = self.crop_image(
            self.get_cluster_corners(clusters[1]),
            boat_to_cam,
            image,
        )
        right = self.crop_image(
            self.get_cluster_corners(clusters[2]),
            boat_to_cam,
            image,
        )
        list = [left, middle, right]

        h_min = min(a.shape[0] for a in list)
        resized = [
            cv2.resize(
                im,
                (int(im.shape[1] * h_min / im.shape[0]), h_min),
                interpolation=cv2.INTER_CUBIC,
            )
            for im in list
        ]
        concat = cv2.hconcat(resized)
        msg = self.bridge.cv2_to_imgmsg(concat, encoding="rgb8")
        self.contour_pub.publish(msg)
        return list

    def find_color(self, images, color):
        for img in images:
            img = img.astype("uint8")
            ret3, th3 = cv2.threshold(img, 0, 255, cv2.THRESH_OTSU)
            img = cv2.cvtColor(th3, cv2.COLOR_GRAY2RGB)
            # edges = cv2.Canny(img, 100, 200)
            # contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # img = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)
            # largestContour = (0, None)
            # for contour in contours:
            #     area = cv2.contourArea(contour)
            #     if area > largestContour[0]:
            #         largestContour = (area, contour)
            # cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
            msg = self.bridge.cv2_to_imgmsg(img, encoding="rgb8")
            self.contour_pub.publish(msg)

    def get_ogrid_coords(self, arr):
        return self.intup(self.ogrid_cpm * (np.asarray(arr) - self.ogrid_origin))[:2]

    def bbox_to_point(self, scale, rot, pos, xmul, ymul):
        return self.intup(
            self.ogrid_cpm
            * (
                self.get_point(
                    [(xmul * scale[0]) / 2, (ymul * scale[1]) / 2, scale[2]],
                    rot,
                    pos,
                )
                - self.ogrid_origin
            )[:2],
        )

    # returns True if side a is closest, False is side b is closest
    # This function works by drawing lines on the OGrid image. The OGrid is a 2D top down
    # image that takes the LIDAR points and colors a pixel in this grid is a point occupies it.
    # Using the bounding box of the LIDAR object, we create a bounding box around the 2D image of the dock.
    # Then, the center is calculated and lines are drawn to the middle point of each side of the bounding box.
    # The number of occupies squares are counted on the line from the object center to line middle. The line
    # which has the least amount of occupied space points to the open side of the dock.
    def calculate_correct_side(
        self,
        side_a: [float],
        side_b: [float],
        side_c: [float],
        side_d: [float],
        position: [float],
        rotation: [[float]],
        scale: [float],
    ) -> [float]:
        print("Finding ogrid center of mass")

        point1 = self.bbox_to_point(scale, rotation, position, -1, 1)
        point2 = self.bbox_to_point(scale, rotation, position, 1, 1)
        point3 = self.bbox_to_point(scale, rotation, position, 1, -1)
        point4 = self.bbox_to_point(scale, rotation, position, -1, -1)
        contours = np.array([point1, point2, point3, point4])

        center = self.calculate_center_of_mass(contours)
        mask = np.zeros(self.last_image.shape, np.uint8)

        bounding_rect = cv2.boundingRect(contours)
        x, y, w, h = bounding_rect

        image_or = np.zeros(self.last_image.shape, np.uint8)

        cv2.line(mask, self.get_ogrid_coords(side_a), center, (255, 255, 255), 2)
        side_a_and = cv2.bitwise_and(copy.deepcopy(self.last_image), mask)
        image_or = cv2.bitwise_or(image_or, side_a_and)
        mask = np.zeros(self.last_image.shape, np.uint8)

        cv2.line(mask, self.get_ogrid_coords(side_b), center, (255, 255, 255), 2)
        side_b_and = cv2.bitwise_and(copy.deepcopy(self.last_image), mask)
        image_or = cv2.bitwise_or(image_or, side_b_and)
        mask = np.zeros(self.last_image.shape, np.uint8)

        cv2.line(mask, self.get_ogrid_coords(side_c), center, (255, 255, 255), 2)
        side_c_and = cv2.bitwise_and(copy.deepcopy(self.last_image), mask)
        image_or = cv2.bitwise_or(image_or, side_c_and)
        mask = np.zeros(self.last_image.shape, np.uint8)

        cv2.line(mask, self.get_ogrid_coords(side_d), center, (255, 255, 255), 2)
        side_d_and = cv2.bitwise_and(copy.deepcopy(self.last_image), mask)
        image_or = cv2.bitwise_or(image_or, side_d_and)

        msg = self.bridge.cv2_to_imgmsg(image_or, encoding="rgb8")
        self.contour_pub.publish(msg)

        side_a_count = np.count_nonzero(side_a_and == np.asarray([255, 255, 255]))
        print(side_a_count)
        side_b_count = np.count_nonzero(side_b_and == np.asarray([255, 255, 255]))
        print(side_b_count)
        side_c_count = np.count_nonzero(side_c_and == np.asarray([255, 255, 255]))
        print(side_c_count)
        side_d_count = np.count_nonzero(side_d_and == np.asarray([255, 255, 255]))
        print(side_d_count)

        lowest = min([side_a_count, side_b_count, side_c_count, side_d_count])
        print("Center of mass found")
        if lowest == side_a_count:
            return side_a
        elif lowest == side_b_count:
            return side_b
        elif lowest == side_c_count:
            return side_c
        else:
            return side_d

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
                # gets the largest object in the clusters database
                msg = await self.get_largest_object()
            except Exception:
                await self.move.forward(10).go()
        # label the largest LIDAR object in the PCODAR database "dock"
        await self.pcodar_label(msg.id, "dock")
        # if no pcodar objects, throw error, exit mission
        pose = pose_to_numpy(msg.pose)

        # return position of dock
        return pose

    def ogrid_cb(self, msg):
        self.ogrid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.ogrid_origin = rosmsg_to_numpy(msg.info.origin.position)
        self.ogrid_cpm = 1 / msg.info.resolution

        image = 255 * np.greater(self.ogrid, 90).astype(np.uint8)
        grayImage = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        self.last_image = grayImage
        return
