#!/usr/bin/env python3

from collections import deque

import cv2
import mil_ros_tools
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Pose2D, PoseStamped
from image_geometry import PinholeCameraModel
from mil_ros_tools import Image_Publisher, Image_Subscriber, rosmsg_to_numpy
from mil_vision_tools import CircleFinder, Threshold
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_srvs.srv import SetBool, SetBoolResponse
from sub8_msgs.srv import (
    VisionRequest,
    VisionRequest2D,
    VisionRequest2DResponse,
    VisionRequestResponse,
)
from sub8_vision_tools import MultiObservation, rviz


class Buoy:
    """
    Represents one colored buoy for use in BuoyFinder. Contains
    color values to segment an image for its color and an internal
    buffer of observations to use in position estimation.
    """

    def __init__(self, color, debug_cv=False):
        self._observations = deque()
        self._pose_pairs = deque()
        self._times = deque()
        self.last_t = None
        self.debug_cv = debug_cv
        self.status = ""
        self.est = None
        self.color = color
        self.timeout = rospy.Duration(rospy.get_param("~timeout_seconds"))
        self.min_trans = rospy.get_param("~min_trans")
        self.threshold = Threshold.from_param(f"/color/buoy/{color}")
        rospy.loginfo(
            f"{color} buoy has {self.threshold}"
        )  # Print threshold for each buoy
        if debug_cv:
            self.threshold.create_trackbars(window=color)

        # Only for visualization
        if color == "red":
            self.draw_colors = (1.0, 0.0, 0.0, 1.0)
            self.cv_colors = (0, 0, 255, 0)
            self.visual_id = 0
        elif color == "green":
            self.draw_colors = (0.0, 1.0, 0.0, 1.0)
            self.cv_colors = (0, 255, 0, 0)
            self.visual_id = 1
        elif color == "yellow":
            self.draw_colors = (1.0, 1.0, 0.0, 1.0)
            self.cv_colors = (0, 255, 255, 0)
            self.visual_id = 2
        else:
            rospy.logerr(f"Unknown buoy color {color}")
            self.draw_colors = (0.0, 0.0, 0.0, 1.0)
            self.visual_id = 3

    def get_mask(self, img):
        """
        Return an image thresholded with the values and colorspace specified
        in the color calibration file.
        """
        return self.threshold(img)

    def clear_old_observations(self):
        time = rospy.Time.now()
        i = 0
        while i < len(self._times):
            if time - self._times[i] > self.timeout:
                self._times.popleft()
                self._observations.popleft()
                self._pose_pairs.popleft()
            else:
                i += 1

    def add_observation(self, obs, pose_pair, time):
        self.clear_old_observations()
        if (
            self.size() == 0
            or np.linalg.norm(self._pose_pairs[-1][0] - pose_pair[0]) > self.min_trans
        ):
            self._observations.append(obs)
            self._pose_pairs.append(pose_pair)
            self._times.append(time)

    def get_observations_and_pose_pairs(self):
        self.clear_old_observations()
        return (self._observations, self._pose_pairs)

    def size(self):
        return len(self._observations)

    def clear(self):
        self._times.clear()
        self._observations.clear()
        self._pose_pairs.clear()


class BuoyFinder:
    """
    Node to find red, green, and yellow buoys in a single camera frame.

    Combines several observations and uses a least-squares approach to get a 3D
    position estimate of a buoy when requested.

    Intended to be modular so other approaches can be tried. Adding more sophistication
    to segmentation would increase reliability.
    """

    # TODO: Use same mask for yellow/green
    def __init__(self):
        self.tf_listener = tf.TransformListener()

        self.enabled = False
        self.last_image = None
        self.last_image_time = None
        self.camera_model = None
        self.circle_finder = CircleFinder(
            1.0
        )  # Model radius doesn't matter because it's not being used for 3D pose

        # Various constants for tuning, debugging. See buoy_finder.yaml for more info
        self.min_observations = rospy.get_param("~min_observations")
        self.debug_ros = rospy.get_param("~debug/ros", True)
        self.debug_cv = rospy.get_param("~debug/cv", False)
        self.min_contour_area = rospy.get_param("~min_contour_area")
        self.max_circle_error = rospy.get_param("~max_circle_error")
        self.max_velocity = rospy.get_param("~max_velocity")
        self.roi_y = rospy.get_param("~roi_y")
        self.roi_height = rospy.get_param("~roi_height")
        camera = rospy.get_param(
            "~camera_topic", "/camera/front/right/image_rect_color"
        )

        self.buoys = {}
        for color in ["red", "yellow", "green"]:
            self.buoys[color] = Buoy(color, debug_cv=self.debug_cv)
        if self.debug_cv:
            cv2.waitKey(1)
            self.debug_images = {}

        # Keep latest odom message for sanity check
        self.last_odom = None
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=3)

        self.image_sub = Image_Subscriber(camera, self.image_cb)
        if self.debug_ros:
            self.rviz = rviz.RvizVisualizer(topic="~markers")
            self.mask_pub = Image_Publisher("~mask_image")
            rospy.Timer(rospy.Duration(1), self.print_status)

        self.camera_info = self.image_sub.wait_for_camera_info()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)
        self.frame_id = self.camera_model.tfFrame()
        self.multi_obs = MultiObservation(self.camera_model)

        rospy.Service("~enable", SetBool, self.toggle_search)
        rospy.Service("~2D", VisionRequest2D, self.request_buoy)
        rospy.Service("~pose", VisionRequest, self.request_buoy3d)

        rospy.loginfo("BUOY FINDER: initialized successfully")

    def odom_cb(self, odom):
        self.last_odom = odom

    def toggle_search(self, srv):
        """
        Callback for standard ~enable service. If true, start
        looking at frames for buoys.
        """
        if srv.data:
            rospy.loginfo("BUOY FINDER: enabled")
            self.enabled = True
        else:
            rospy.loginfo("BUOY FINDER: disabled")
            self.enabled = False

        return SetBoolResponse(success=True)

    def request_buoy(self, srv):
        """
        Callback for 2D vision request. Returns centroid
        of buoy found with color specified in target_name
        if found.
        """
        if not self.enabled or srv.target_name not in self.buoys:
            return VisionRequest2DResponse(found=False)
        response = self.find_single_buoy(srv.target_name)
        if response is False or response is None:
            return VisionRequest2DResponse(found=False)
        center, radius = response
        return VisionRequest2DResponse(
            header=Header(stamp=self.last_image_time, frame_id=self.frame_id),
            pose=Pose2D(
                x=center[0],
                y=center[1],
            ),
            max_x=self.last_image.shape[0],
            max_y=self.last_image.shape[1],
            camera_info=self.image_sub.camera_info,
            found=True,
        )

    def request_buoy3d(self, srv):
        """
        Callback for 3D vision request. Uses recent observations of buoy
        specified in target_name to attempt a least-squares position estimate.
        As buoys are spheres, orientation is meaningless.
        """
        if srv.target_name not in self.buoys or not self.enabled:
            return VisionRequestResponse(found=False)
        buoy = self.buoys[srv.target_name]
        if buoy.est is None:
            return VisionRequestResponse(found=False)
        return VisionRequestResponse(
            pose=PoseStamped(
                header=Header(stamp=self.last_image_time, frame_id="map"),
                pose=Pose(position=Point(*buoy.est)),
            ),
            found=True,
        )

    def image_cb(self, image):
        """
        Run each time an image comes in from ROS. If enabled,
        attempt to find each color buoy.
        """
        if not self.enabled:
            return

        # Crop out some of the top and bottom to exclude the floor and surface reflections
        height = image.shape[0]
        roi_y = int(self.roi_y * height)
        roi_height = height - int(self.roi_height * height)
        self.roi = (0, roi_y, roi_height, image.shape[1])
        self.last_image = image[self.roi[1] : self.roi[2], self.roi[0] : self.roi[3]]
        self.image_area = self.last_image.shape[0] * self.last_image.shape[1]

        if self.debug_ros:
            # Create a blacked out debug image for putting masks in
            self.mask_image = np.zeros(self.last_image.shape, dtype=image.dtype)
        if (
            self.last_image_time is not None
            and self.image_sub.last_image_time < self.last_image_time
        ):
            # Clear tf buffer if time went backwards (nice for playing bags in loop)
            self.tf_listener.clear()
        self.last_image_time = self.image_sub.last_image_time
        self.find_buoys()
        if self.debug_ros:
            self.mask_pub.publish(self.mask_image)

    def print_status(self, _):
        """
        Called at 1 second intervals to display the status (not found, n observations, FOUND)
        for each buoy.
        """
        if self.enabled:
            rospy.loginfo(
                "STATUS: RED='%s', GREEN='%s', YELLOW='%s'",
                self.buoys["red"].status,
                self.buoys["green"].status,
                self.buoys["yellow"].status,
            )

    def find_buoys(self):
        """
        Run find_single_buoy for each color of buoy
        """
        for buoy_name in self.buoys:
            self.find_single_buoy(buoy_name)

    def is_circular_contour(self, cnt):
        """
        Check that a contour is close enough to a circle (using hue invariants)
        to be a pottential buoy.
        """
        return self.circle_finder.verify_contour(cnt) <= self.max_circle_error

    def get_best_contour(self, contours):
        """
        Attempts to find a good buoy contour among those found within the
        thresholded mask. If a good one is found, it return (contour, centroid, area),
        otherwise returns None. Right now the best contour is just the largest.

        Args:
            contours (np.ndarray): Numpy array of contours from a particular buoy's mask

        Returns:
            tuple (contour, error) where contour will be the best contour
            in an image or None if no contours pass criteria. Error will be a string
            describing why no good contour was found, or None if contour is not None.
        """
        if len(contours) == 0:
            return None, "no contours in mask"
        circular_contours = list(filter(self.is_circular_contour, contours))
        if len(circular_contours) == 0:
            return None, "fails circularity test"
        circles_sorted = sorted(circular_contours, key=cv2.contourArea, reverse=True)
        if cv2.contourArea(circles_sorted[0]) < self.min_contour_area * self.image_area:
            return None, "fails area test"
        return (
            circles_sorted[0],
            None,
        )  # Return the largest contour that pases shape test

    def find_single_buoy(self, buoy_type):
        """
        Attempt to find one color buoy in the image.
        1) Create mask for buoy's color in colorspace specified in parameters
        2) Select the largest contour in this mask
        3) Approximate a circle around this contour
        4) Store the center of this circle and the current tf between map and camera
           as an observation
        5) If observations for this buoy is now >= min_observations, approximate buoy
           position using the least squares tool imported
        """
        assert (
            buoy_type in self.buoys.keys()
        ), f"Buoys_2d does not know buoy color: {buoy_type}"
        buoy = self.buoys[buoy_type]
        mask = buoy.get_mask(self.last_image)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)

        _, contours, _ = cv2.findContours(
            mask,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE,
            offset=(self.roi[0], self.roi[1]),
        )
        if self.debug_ros:
            cv2.add(
                self.mask_image.copy(), buoy.cv_colors, mask=mask, dst=self.mask_image
            )
        if self.debug_cv:
            self.debug_images[buoy_type] = mask.copy()

        cnt, err = self.get_best_contour(contours)
        if cnt is None:
            buoy.clear_old_observations()
            buoy.status = f"{err} w/ {buoy.size()} obs"
            return
        center, radius = cv2.minEnclosingCircle(cnt)

        if self.debug_ros:
            cv2.circle(
                self.mask_image,
                (int(center[0] - self.roi[0]), int(center[1]) - self.roi[1]),
                int(radius),
                buoy.cv_colors,
                4,
            )

        try:
            self.tf_listener.waitForTransform(
                "map", self.frame_id, self.last_image_time, rospy.Duration(0.2)
            )
        except tf.Exception as e:
            rospy.logwarn(f"Could not transform camera to map: {e}")
            return False

        if not self.sanity_check(center, self.last_image_time):
            buoy.status = "failed sanity check"
            return False

        (t, rot_q) = self.tf_listener.lookupTransform(
            "map", self.frame_id, self.last_image_time
        )
        R = mil_ros_tools.geometry_helpers.quaternion_matrix(rot_q)

        buoy.add_observation(center, (np.array(t), R), self.last_image_time)

        observations, pose_pairs = buoy.get_observations_and_pose_pairs()
        if len(observations) > self.min_observations:
            buoy.est = self.multi_obs.lst_sqr_intersection(observations, pose_pairs)
            buoy.status = "Pose found"
            if self.debug_ros:
                self.rviz.draw_sphere(
                    buoy.est,
                    color=buoy.draw_colors,
                    scaling=(0.2286, 0.2286, 0.2286),
                    frame="map",
                    _id=buoy.visual_id,
                )
        else:
            buoy.status = f"{len(observations)} observations"
        return center, radius

    def sanity_check(self, coordinate, timestamp):
        """
        Check if the observation is unreasonable. More can go here if we want.
        """
        if self.last_odom is None:
            return False

        linear_velocity = rosmsg_to_numpy(self.last_odom.twist.twist.linear)
        if np.linalg.norm(linear_velocity) > self.max_velocity:
            return False

        return True


if __name__ == "__main__":
    rospy.init_node("buoy_finder")
    b = BuoyFinder()

    if b.debug_cv:
        # Keep opencv gui alive as ros spins
        cv2.waitKey(1)
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            for k in b.debug_images:
                cv2.imshow(k, b.debug_images[k])
            cv2.waitKey(1)
            r.sleep()
    else:
        rospy.spin()
